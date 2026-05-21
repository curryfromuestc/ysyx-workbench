// =============================================================================
// ysyx_22040000  --  D6/C2 SoC top wrapper
// =============================================================================
// Multi-cycle RV32E_Zicsr CPU that speaks SimpleBus (ysyxSoC `MemBridge`).
//
// Pin names / widths match `ysyxSoC/ready-to-run/D-stage/cpu-interface.md`.
// The wrapper instantiates the same IDU / EXU / RegFile / CSR / WBU modules
// used by the DPI harness, but replaces the combinational IFU / LSU with bus
// state machines so each fetch / load / store waits for `respValid`.
//
// State machine (single FSM that sequences every instruction):
//   S_IF : drive io_ifu_reqValid=1, wait for io_ifu_respValid, latch instruction.
//   S_EX : decode/execute combinationally. For non-load/store -> go S_WB
//          directly. For load/store -> drive io_lsu_reqValid=1, wait for
//          io_lsu_respValid, latch rdata (load) / observe bready (store).
//   S_WB : commit: write regfile (if reg_wen), write CSR (if csr_we), update PC,
//          then go back to S_IF.
//
// reg_wen / csr_we are only asserted for one cycle (in S_WB) to avoid double
// writes that would otherwise happen because IDU is purely combinational.
//
// Reset vector: 0x30000000 (Flash base on the SoC).
// =============================================================================
`ifndef SOC_PC_RESET_VEC
  `define SOC_PC_RESET_VEC 32'h3000_0000
`endif

// B4a: icache 模块定义通过 `include 拉进来 (不改 Makefile 的源文件列表).
// 路径相对于 verilator 的工作目录 (npc/), 与 Makefile $(TOPDIR) 一致.
`include "vsrc/icache.v"

module ysyx_22040000(
  input         clock,
  input         reset,
  // ---- IFU master ----------------------------------------------------------
  output        io_ifu_reqValid,
  output [31:0] io_ifu_addr,
  input         io_ifu_respValid,
  input  [31:0] io_ifu_rdata,
  // ---- LSU master ----------------------------------------------------------
  output        io_lsu_reqValid,
  output [31:0] io_lsu_addr,
  output [1:0]  io_lsu_size,
  output        io_lsu_wen,
  output [31:0] io_lsu_wdata,
  output [3:0]  io_lsu_wmask,
  input         io_lsu_respValid,
  input  [31:0] io_lsu_rdata,
  // ---- B2a Access Fault -----------------------------------------------------
  // Combinational pulse from the SoC bridge: high on the same cycle that
  // io_ifu_respValid / io_lsu_respValid is high IF the underlying AXI bresp /
  // rresp != 2'b00 (SLVERR or DECERR). When asserted the CPU treats the
  // current transaction as failed: flushes any partial decode/regfile/CSR
  // write that this instruction would have caused and forces the next fetch
  // PC to 0. Wired to 0 by the DPI-C harness top so non-SoC builds are
  // unaffected.
  input         io_fault
);
  // ---- FSM ------------------------------------------------------------------
  localparam S_IF = 2'd0;
  localparam S_EX = 2'd1;
  localparam S_LS = 2'd2;
  localparam S_WB = 2'd3;

  reg [1:0]  state, next_state;

  // ---- PC and latched instruction ------------------------------------------
  reg  [31:0] pc;
  reg  [31:0] inst_r;
  // B4a: 取指数据来自 icache (而不是直接从 bus). icache 在 hit 时同周期返回,
  // miss 时填表后那一拍返回. 上游 FSM 不需要关心.
  wire [31:0] inst = (state == S_IF) ? ifu_cpu_resp_data : inst_r;

  // ---- Decode (combinational, on `inst`) -----------------------------------
  wire [3:0]  rs1_a, rs2_a, rd_a;
  wire [31:0] imm;
  wire        alu_src;
  wire [3:0]  alu_op;
  wire        alu_use_pc;
  wire        mem_re, mem_we;
  wire [2:0]  funct3;
  wire [1:0]  wb_sel;
  wire        reg_wen_dec;
  wire        is_jal;
  wire        is_jalr;
  wire        is_branch;
  wire [2:0]  branch_op;
  wire        is_ebreak;
  wire        is_mret;
  wire        is_ecall;
  wire        is_csr;
  wire [11:0] csr_addr;
  wire [4:0]  csr_uimm;
  wire        csr_use_uimm;
  wire        csr_re;
  wire        csr_we_dec;
  wire [1:0]  csr_op;

  IDU u_idu (
    .inst(inst), .pc(pc),
    .rs1(rs1_a), .rs2(rs2_a), .rd(rd_a),
    .imm(imm), .alu_src(alu_src), .alu_op(alu_op), .alu_use_pc(alu_use_pc),
    .mem_re(mem_re), .mem_we(mem_we), .funct3(funct3),
    .wb_sel(wb_sel), .reg_wen(reg_wen_dec),
    .is_jal(is_jal), .is_jalr(is_jalr),
    .is_branch(is_branch), .branch_op(branch_op),
    .is_ebreak(is_ebreak), .is_mret(is_mret), .is_ecall(is_ecall),
    .is_csr(is_csr), .csr_addr(csr_addr),
    .csr_uimm(csr_uimm), .csr_use_uimm(csr_use_uimm),
    .csr_re(csr_re), .csr_we(csr_we_dec), .csr_op(csr_op)
  );

  // ---- Register file --------------------------------------------------------
  wire [31:0] rs1_val, rs2_val, a5_val;
  wire [31:0] wb_data;
  wire        reg_wen_wb = reg_wen_dec & (state == S_WB) & ~is_ebreak;

  RegFile #(.ADDR_WIDTH(4), .DATA_WIDTH(32)) u_rf (
    .clk(clock),
    .raddr1(rs1_a), .rdata1(rs1_val),
    .raddr2(rs2_a), .rdata2(rs2_val),
    // raddr3 dedicated to a5 (x15) for the CTE trap path under ilp32e.
    .raddr3(4'd15), .rdata3(a5_val),
    .wdata(wb_data), .waddr(rd_a), .wen(reg_wen_wb)
  );

  // ---- EXU ------------------------------------------------------------------
  wire [31:0] alu_result;
  wire        branch_taken;
  EXU u_exu (
    .pc(pc),
    .rs1_val(rs1_val), .rs2_val(rs2_val), .imm(imm),
    .alu_src(alu_src), .alu_use_pc(alu_use_pc), .alu_op(alu_op),
    .branch_op(branch_op),
    .alu_result(alu_result), .branch_taken(branch_taken)
  );

  // ---- LSU: build aligned store word + wmask --------------------------------
  reg  [31:0] lsu_rdata_r;
  wire [1:0]  byte_off = alu_result[1:0];

  reg  [31:0] store_word;
  reg  [3:0]  store_wmask;
  reg  [1:0]  store_size;
  always @(*) begin
    store_word  = 32'h0;
    store_wmask = 4'b0;
    store_size  = 2'b10; // default sw
    if (mem_we) begin
      case (funct3)
        3'b010: begin                                       // sw
          store_word  = rs2_val;
          store_wmask = 4'b1111;
          store_size  = 2'b10;
        end
        3'b001: begin                                       // sh
          case (byte_off[1])
            1'b0: begin store_word = {16'h0, rs2_val[15:0]};            store_wmask = 4'b0011; end
            1'b1: begin store_word = {rs2_val[15:0], 16'h0};            store_wmask = 4'b1100; end
          endcase
          store_size = 2'b01;
        end
        3'b000: begin                                       // sb
          case (byte_off)
            2'b00: begin store_word = {24'h0, rs2_val[7:0]};                 store_wmask = 4'b0001; end
            2'b01: begin store_word = {16'h0, rs2_val[7:0], 8'h0};            store_wmask = 4'b0010; end
            2'b10: begin store_word = {8'h0,  rs2_val[7:0], 16'h0};           store_wmask = 4'b0100; end
            2'b11: begin store_word = {       rs2_val[7:0], 24'h0};           store_wmask = 4'b1000; end
          endcase
          store_size = 2'b00;
        end
        default: begin
          store_word  = rs2_val;
          store_wmask = 4'b1111;
          store_size  = 2'b10;
        end
      endcase
    end
  end

  reg [1:0] load_size;
  always @(*) begin
    case (funct3)
      3'b000: load_size = 2'b00; // lb
      3'b001: load_size = 2'b01; // lh
      3'b010: load_size = 2'b10; // lw
      3'b100: load_size = 2'b00; // lbu
      3'b101: load_size = 2'b01; // lhu
      default: load_size = 2'b10;
    endcase
  end

  // Decode load result (use latched rdata).
  reg [31:0] load_data;
  reg [7:0]  ld_byte_sel;
  reg [15:0] ld_half_sel;
  always @(*) begin
    case (byte_off)
      2'b00:   ld_byte_sel = lsu_rdata_r[ 7: 0];
      2'b01:   ld_byte_sel = lsu_rdata_r[15: 8];
      2'b10:   ld_byte_sel = lsu_rdata_r[23:16];
      2'b11:   ld_byte_sel = lsu_rdata_r[31:24];
      default: ld_byte_sel = 8'h0;
    endcase
    case (byte_off[1])
      1'b0:    ld_half_sel = lsu_rdata_r[15: 0];
      1'b1:    ld_half_sel = lsu_rdata_r[31:16];
      default: ld_half_sel = 16'h0;
    endcase
    load_data = 32'h0;
    if (mem_re) begin
      case (funct3)
        3'b000: load_data = {{24{ld_byte_sel[7]}},  ld_byte_sel};   // lb
        3'b001: load_data = {{16{ld_half_sel[15]}}, ld_half_sel};   // lh
        3'b010: load_data = lsu_rdata_r;                            // lw
        3'b100: load_data = {24'h0, ld_byte_sel};                   // lbu
        3'b101: load_data = {16'h0, ld_half_sel};                   // lhu
        default: load_data = lsu_rdata_r;
      endcase
    end
  end

  // ---- CSR ------------------------------------------------------------------
  wire [31:0] csr_rdata;
  wire [31:0] mepc_w;
  wire [31:0] mtvec_w;
  wire [31:0] csr_src = csr_use_uimm ? {27'b0, csr_uimm} : rs1_val;
  wire [31:0] csr_wdata =
        (csr_op == 2'b00) ? csr_src :              // csrrw / csrrwi
        (csr_op == 2'b01) ? (csr_rdata | csr_src): // csrrs / csrrsi
        (csr_op == 2'b10) ? (csr_rdata & ~csr_src):// csrrc / csrrci
                            csr_src;
  wire csr_we_wb = csr_we_dec & (state == S_WB) & ~is_ebreak;
  // Same one-shot rule as csr_we_wb: only commit the trap during S_WB so we
  // don't double-write mcause/mepc and don't redirect PC mid-fetch.
  wire trap_fire = is_ecall & (state == S_WB) & ~reset;
  CSR u_csr (
    .clk(clock), .rst(reset),
    .csr_raddr(csr_addr), .csr_rdata(csr_rdata),
    .csr_we(csr_we_wb),
    .csr_waddr(csr_addr), .csr_wdata(csr_wdata),
    .is_trap(trap_fire), .trap_cause(a5_val), .trap_epc(pc),
    .mepc_out(mepc_w), .mtvec_out(mtvec_w)
  );

  // ---- WBU + PC -------------------------------------------------------------
  wire [31:0] pc_next;
  WBU u_wbu (
    .alu_result(alu_result), .load_data(load_data),
    .csr_rdata(csr_rdata),
    .wb_sel(wb_sel),
    .is_jal(is_jal), .is_jalr(is_jalr),
    .is_branch(is_branch), .branch_taken(branch_taken),
    .is_mret(is_mret), .mepc(mepc_w),
    .is_trap(trap_fire), .mtvec(mtvec_w),
    .pc(pc), .imm(imm),
    .pc_next(pc_next), .wb_data(wb_data)
  );

  // ---- Bus outputs ----------------------------------------------------------
  // B4a: 在 IFU 通路里塞一个 icache. 直接映射, 16 块 x 4B = 64B, 触发器实现.
  // CPU FSM -> icache 上游 (req_valid / resp_valid 同协议)
  // icache 下游 -> 外部 SimpleBus 端口 io_ifu_* (协议不变)
  wire        ifu_cpu_req_valid  = (state == S_IF) & ~reset;
  wire [31:0] ifu_cpu_req_addr   = {pc[31:2], 2'b00};
  wire        ifu_cpu_resp_valid;
  wire [31:0] ifu_cpu_resp_data;

  icache #(.BLOCKS_LOG(4), .OFFSET_LOG(2)) u_icache (
    .clock          (clock),
    .reset          (reset),
    .req_valid      (ifu_cpu_req_valid),
    .req_addr       (ifu_cpu_req_addr),
    .resp_valid     (ifu_cpu_resp_valid),
    .resp_data      (ifu_cpu_resp_data),
    .bus_req_valid  (io_ifu_reqValid),
    .bus_req_addr   (io_ifu_addr),
    .bus_resp_valid (io_ifu_respValid),
    .bus_resp_data  (io_ifu_rdata)
  );

  wire ls_active = (state == S_LS);
  assign io_lsu_reqValid = ls_active & ~reset & (mem_re | mem_we);
  assign io_lsu_addr     = alu_result;
  assign io_lsu_size     = mem_we ? store_size : load_size;
  assign io_lsu_wen      = mem_we;
  assign io_lsu_wdata    = store_word;
  assign io_lsu_wmask    = store_wmask;

  // ---- B2a Access Fault decoding -------------------------------------------
  // io_fault is the combinational SLVERR/DECERR pulse from the SoC bridge.
  // It is valid only on a cycle when the corresponding respValid is also
  // high; we further gate by the current FSM state so that a stale fault
  // signal in some other state (which the bridge does not produce today
  // anyway) can never poison an instruction it shouldn't.
  // NOTE B4a: 引入 icache 后, IFU 的 fault 检测口径相应改成 "icache 上游
  // resp_valid". 当 miss + bus 返 fault 时, icache 把 bus_resp_valid (= 1) +
  // bus_resp_data (junk) 同步抛给上游, 此时 io_fault 在 bus 侧依然有效 ->
  // 同周期 ifu_cpu_resp_valid 也 = 1, 我们用它来 latch fault.
  wire ifu_fault = (state == S_IF) & ifu_cpu_resp_valid & io_fault;
  wire lsu_fault = (state == S_LS) & io_lsu_respValid & io_fault;
  wire any_fault = ifu_fault | lsu_fault;

  // Sticky fault counter for observability from the harness: incremented
  // every cycle that any_fault is asserted. Useful for B2b/microbench when
  // SDRAM-decoded SLVERR will start showing up; the C harness can poke
  // this via verilator's __DOT__ accessor.
  reg [31:0] fault_count;
  always @(posedge clock) begin
    if (reset)              fault_count <= 32'h0;
    else if (any_fault)     fault_count <= fault_count + 32'h1;
  end

  // ---- FSM next-state -------------------------------------------------------
  always @(*) begin
    next_state = state;
    case (state)
      // On IFU fault: skip decode entirely and restart fetch (PC <= 0).
      // B4a: 等待 icache 的 resp_valid (而不是直接看 bus respValid). 在 hit
      // 时 ifu_cpu_resp_valid 与 ifu_cpu_req_valid 同周期 -> 单拍取指完成.
      S_IF: if (ifu_fault)                  next_state = S_IF;
            else if (ifu_cpu_resp_valid)    next_state = S_EX;
      S_EX: begin
        if (is_ebreak)                 next_state = S_EX; // park forever on ebreak
        else if (mem_re | mem_we)      next_state = S_LS;
        else                           next_state = S_WB;
      end
      // On LSU fault: skip writeback (don't commit reg/CSR with junk rdata),
      // restart fetch (PC <= 0).
      S_LS: if (lsu_fault)               next_state = S_IF;
            else if (io_lsu_respValid)   next_state = S_WB;
      S_WB:                            next_state = S_IF;
      default:                         next_state = S_IF;
    endcase
  end

  // ---- Sequential updates ---------------------------------------------------
  always @(posedge clock) begin
    if (reset) begin
      state       <= S_IF;
      pc          <= `SOC_PC_RESET_VEC;
      inst_r      <= 32'h0;
      lsu_rdata_r <= 32'h0;
    end else begin
      state <= next_state;
      // Latch the IFU result only on a clean fetch; on fault, leave inst_r
      // alone so we never decode the (likely zero) rdata that came with the
      // SLVERR response.
      // B4a: 现在 latch icache 返回的数据 (hit 时来自 cache; miss 时 cache
      // 同周期透传 bus 数据), 而不是 bus rdata.
      if (state == S_IF && ifu_cpu_resp_valid && !ifu_fault) begin
        inst_r <= ifu_cpu_resp_data;
      end
      // Same idea on the load side: a fault response brings back junk
      // rdata; don't latch it, so even if some downstream consumer peeks
      // lsu_rdata_r it sees the previous (last good) value.
      if (state == S_LS && io_lsu_respValid && mem_re && !lsu_fault) begin
        lsu_rdata_r <= io_lsu_rdata;
      end
      // PC update rules:
      //   1. Any access fault on this instruction -> next PC = 0
      //      (docs/2407/b/2.md: "let NPC jump to address 0 when AXI resp
      //      reports an error, even before CTE is enabled").
      //   2. Otherwise the normal WBU-decided pc_next at S_WB commit.
      if (any_fault) begin
        pc <= 32'h0;
      end else if (state == S_WB && !is_ebreak) begin
        pc <= pc_next;
      end
    end
  end
endmodule
