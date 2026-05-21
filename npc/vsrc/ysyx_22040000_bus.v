// =============================================================================
// ysyx_22040000_bus  --  B1 full-handshake SimpleBus CPU top
// =============================================================================
// Same RV32E_Zicsr CPU core as ysyx_22040000.v (D6c), but the IFU / LSU master
// interfaces use the FULL handshake from b/1.md "更普遍的存储器(2)":
//
//   reqValid  --->          (CPU drives)
//   <--- reqReady           (MEM acks; req latched when both high in same cycle)
//   addr / wen / wdata / wmask  (paired with reqValid)
//   <--- respValid          (MEM drives when data is ready)
//   respReady --->          (CPU acks; resp consumed when both high in same cycle)
//   <--- rdata
//
// This file is INTENTIONALLY separate from vsrc/ysyx_22040000.v: that module
// is bound to ysyxSoCFull.v's MemBridge, which only speaks the half-handshake
// (reqValid/respValid) -- adding reqReady/respReady ports there would break
// the ysyxSoC port list. By living in its own file with its own top-level
// module name, this version coexists peacefully with `make sim-soc`.
//
// FSM (per access):
//   S_IF_REQ  : ifu_reqValid=1, wait for ifu_reqReady=1 -> S_IF_WAIT
//   S_IF_WAIT : ifu_respReady=1, wait for ifu_respValid=1 -> latch inst, S_EX
//   S_EX      : combinational decode/execute. Branch on op:
//                 ebreak                       -> stay in S_EX (park)
//                 load/store                   -> S_LS_REQ
//                 anything else                -> S_WB
//   S_LS_REQ  : lsu_reqValid=1, wait for lsu_reqReady=1 -> S_LS_WAIT
//   S_LS_WAIT : lsu_respReady=1, wait for lsu_respValid=1 -> latch rdata, S_WB
//   S_WB      : commit (regfile write, csr write, pc update) -> S_IF_REQ
//
// Reset vector: 0x80000000 (matches D4/D5 cpu-tests layout, since this top is
// used by `make sim-bus` with a flat .bin loaded at 0x80000000, not the SoC
// flash at 0x30000000).
// =============================================================================
`ifndef BUS_PC_RESET_VEC
  `define BUS_PC_RESET_VEC 32'h8000_0000
`endif

module ysyx_22040000_bus(
  input         clock,
  input         reset,
  // ---- IFU master (full handshake) -----------------------------------------
  output        io_ifu_reqValid,
  input         io_ifu_reqReady,
  output [31:0] io_ifu_addr,
  input         io_ifu_respValid,
  output        io_ifu_respReady,
  input  [31:0] io_ifu_rdata,
  // ---- LSU master (full handshake) -----------------------------------------
  output        io_lsu_reqValid,
  input         io_lsu_reqReady,
  output [31:0] io_lsu_addr,
  output [1:0]  io_lsu_size,
  output        io_lsu_wen,
  output [31:0] io_lsu_wdata,
  output [3:0]  io_lsu_wmask,
  input         io_lsu_respValid,
  output        io_lsu_respReady,
  input  [31:0] io_lsu_rdata,
  // ---- Probe outputs (for harness host-side trap detection) ----------------
  output [2:0]  io_dbg_state,
  output [31:0] io_dbg_pc,
  output [31:0] io_dbg_inst
);
  // ---- FSM states ----------------------------------------------------------
  localparam [2:0] S_IF_REQ  = 3'd0;
  localparam [2:0] S_IF_WAIT = 3'd1;
  localparam [2:0] S_EX      = 3'd2;
  localparam [2:0] S_LS_REQ  = 3'd3;
  localparam [2:0] S_LS_WAIT = 3'd4;
  localparam [2:0] S_WB      = 3'd5;

  reg  [2:0] state, next_state;

  // ---- PC / latched instruction / latched load data ------------------------
  reg  [31:0] pc;
  reg  [31:0] inst_r;
  wire [31:0] inst = (state == S_IF_WAIT && io_ifu_respValid) ? io_ifu_rdata : inst_r;

  // ---- Decode (combinational on `inst`) ------------------------------------
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

  // ---- Register file -------------------------------------------------------
  wire [31:0] rs1_val, rs2_val, a5_val;
  wire [31:0] wb_data;
  wire        reg_wen_wb = reg_wen_dec & (state == S_WB) & ~is_ebreak;

  RegFile #(.ADDR_WIDTH(4), .DATA_WIDTH(32)) u_rf (
    .clk(clock),
    .raddr1(rs1_a), .rdata1(rs1_val),
    .raddr2(rs2_a), .rdata2(rs2_val),
    .raddr3(4'd15), .rdata3(a5_val),
    .wdata(wb_data), .waddr(rd_a), .wen(reg_wen_wb)
  );

  // ---- EXU -----------------------------------------------------------------
  wire [31:0] alu_result;
  wire        branch_taken;
  EXU u_exu (
    .pc(pc),
    .rs1_val(rs1_val), .rs2_val(rs2_val), .imm(imm),
    .alu_src(alu_src), .alu_use_pc(alu_use_pc), .alu_op(alu_op),
    .branch_op(branch_op),
    .alu_result(alu_result), .branch_taken(branch_taken)
  );

  // ---- LSU: build aligned store word + wmask -------------------------------
  reg  [31:0] lsu_rdata_r;
  wire [1:0]  byte_off = alu_result[1:0];

  reg  [31:0] store_word;
  reg  [3:0]  store_wmask;
  reg  [1:0]  store_size;
  always @(*) begin
    store_word  = 32'h0;
    store_wmask = 4'b0;
    store_size  = 2'b10;
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
        3'b000: load_data = {{24{ld_byte_sel[7]}},  ld_byte_sel};
        3'b001: load_data = {{16{ld_half_sel[15]}}, ld_half_sel};
        3'b010: load_data = lsu_rdata_r;
        3'b100: load_data = {24'h0, ld_byte_sel};
        3'b101: load_data = {16'h0, ld_half_sel};
        default: load_data = lsu_rdata_r;
      endcase
    end
  end

  // ---- CSR -----------------------------------------------------------------
  wire [31:0] csr_rdata;
  wire [31:0] mepc_w;
  wire [31:0] mtvec_w;
  wire [31:0] csr_src = csr_use_uimm ? {27'b0, csr_uimm} : rs1_val;
  wire [31:0] csr_wdata =
        (csr_op == 2'b00) ? csr_src :
        (csr_op == 2'b01) ? (csr_rdata | csr_src):
        (csr_op == 2'b10) ? (csr_rdata & ~csr_src):
                            csr_src;
  wire csr_we_wb = csr_we_dec & (state == S_WB) & ~is_ebreak;
  wire trap_fire = is_ecall & (state == S_WB) & ~reset;
  CSR u_csr (
    .clk(clock), .rst(reset),
    .csr_raddr(csr_addr), .csr_rdata(csr_rdata),
    .csr_we(csr_we_wb),
    .csr_waddr(csr_addr), .csr_wdata(csr_wdata),
    .is_trap(trap_fire), .trap_cause(a5_val), .trap_epc(pc),
    .mepc_out(mepc_w), .mtvec_out(mtvec_w)
  );

  // ---- WBU + PC ------------------------------------------------------------
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

  // ---- Bus outputs (FULL handshake) ----------------------------------------
  // IFU: reqValid in S_IF_REQ, respReady in S_IF_WAIT.
  assign io_ifu_reqValid  = (state == S_IF_REQ) & ~reset;
  assign io_ifu_respReady = (state == S_IF_WAIT) & ~reset;
  assign io_ifu_addr      = {pc[31:2], 2'b00};

  // LSU: reqValid in S_LS_REQ, respReady in S_LS_WAIT.
  wire ls_needed = (mem_re | mem_we);
  assign io_lsu_reqValid  = (state == S_LS_REQ) & ~reset & ls_needed;
  assign io_lsu_respReady = (state == S_LS_WAIT) & ~reset & ls_needed;
  assign io_lsu_addr      = alu_result;
  assign io_lsu_size      = mem_we ? store_size : load_size;
  assign io_lsu_wen       = mem_we;
  assign io_lsu_wdata     = store_word;
  assign io_lsu_wmask     = store_wmask;

  // ---- FSM next-state ------------------------------------------------------
  always @(*) begin
    next_state = state;
    case (state)
      S_IF_REQ:  if (io_ifu_reqValid  & io_ifu_reqReady)  next_state = S_IF_WAIT;
      S_IF_WAIT: if (io_ifu_respValid & io_ifu_respReady) next_state = S_EX;
      S_EX: begin
        if (is_ebreak)            next_state = S_EX;       // park
        else if (ls_needed)       next_state = S_LS_REQ;
        else                      next_state = S_WB;
      end
      S_LS_REQ:  if (io_lsu_reqValid  & io_lsu_reqReady)  next_state = S_LS_WAIT;
      S_LS_WAIT: if (io_lsu_respValid & io_lsu_respReady) next_state = S_WB;
      S_WB:                                               next_state = S_IF_REQ;
      default:                                            next_state = S_IF_REQ;
    endcase
  end

  // ---- Sequential updates --------------------------------------------------
  always @(posedge clock) begin
    if (reset) begin
      state       <= S_IF_REQ;
      pc          <= `BUS_PC_RESET_VEC;
      inst_r      <= 32'h0;
      lsu_rdata_r <= 32'h0;
    end else begin
      state <= next_state;
      // Latch the fetched instruction at the IFU response handshake.
      if (state == S_IF_WAIT && io_ifu_respValid && io_ifu_respReady) begin
        inst_r <= io_ifu_rdata;
      end
      // Latch the loaded data at the LSU response handshake (load only;
      // stores ignore rdata).
      if (state == S_LS_WAIT && io_lsu_respValid && io_lsu_respReady && mem_re) begin
        lsu_rdata_r <= io_lsu_rdata;
      end
      if (state == S_WB && !is_ebreak) begin
        pc <= pc_next;
      end
    end
  end

  // ---- Probes --------------------------------------------------------------
  assign io_dbg_state = state;
  assign io_dbg_pc    = pc;
  assign io_dbg_inst  = inst_r;
endmodule
