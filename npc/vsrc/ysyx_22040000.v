// =============================================================================
// ysyx_22040000  --  B5a 经典 5 段流水线 (SoC top wrapper)
// =============================================================================
// 替代之前 4 状态 FSM 多周期实现 (S_IF / S_EX / S_LS / S_WB).
// 5 段: IF -> ID -> EX -> MEM -> WB. 每段之间放 pipeline register.
//
// 端口签名 (io_ifu_* / io_lsu_* / io_fault) 跟 SoC bridge 完全不变, 不影响
// ysyxSoC 顶层 / MemBridge / 外设. icache 仍是 B4c 1KB 2-way 32B 块.
//
// 流水线结构 (ASCII):
//
//   +----------+ IF/ID +----------+ ID/EX +----------+ EX/MEM +-----------+ MEM/WB +----------+
//   |   IF     | latch |    ID    | latch |    EX    | latch  |   MEM     | latch  |    WB    |
//   | icache + |======>| IDU      |======>| EXU      |=======>| LSU bus   |=======>| wb_data  |
//   | PC reg   |       | rf read  |       | branch   |        | rdata pick|        | csr we   |
//   |          |       | hazard   |       | jalr tgt |        |           |        | rf write |
//   +----------+       +----------+       +----------+        +-----------+        +----------+
//      ^                                       |                                        |
//      |                                       v                                        v
//      |   <===== redirect_pc (EX 段 branch/jal/jalr) =========                          |
//      |                                                                                |
//      +======== trap_redirect_pc (WB 段 ecall/mret) ==================================+
//
// 每个 pipeline register (id_ex_*, ex_mem_*, mem_wb_*) 都有 valid 位:
//   valid=0 => bubble (= NOP, 无副作用), 来自 reset/flush/stall 注入.
//
// Stall / Flush 决策矩阵 (在 always @(posedge clock) 内根据组合 stall/flush
// 信号决定本拍 latch 谁的值):
//
//   决策 / latch        | IF/ID         | ID/EX         | EX/MEM        | MEM/WB        | PC reg
//   --------------------+---------------+---------------+---------------+---------------+----------
//   正常推进            | IF -> IF/ID   | ID -> ID/EX   | EX -> EX/MEM  | MEM -> MEM/WB | pc+4 / redirect
//   IF miss (no resp)   | bubble        | ID -> ID/EX   | EX -> EX/MEM  | MEM -> MEM/WB | hold
//   ID hazard (interlock)| hold         | bubble        | EX -> EX/MEM  | MEM -> MEM/WB | hold
//   MEM stall (lsu wait)| hold          | hold          | hold          | bubble        | hold
//   EX branch flush     | bubble        | bubble        | EX -> EX/MEM  | MEM -> MEM/WB | <- redirect
//   WB trap/mret flush  | bubble        | bubble        | bubble        | MEM -> MEM/WB | <- redirect
//   EBREAK in WB        | hold          | hold          | hold          | hold          | hold (parked)
//
// Hazard (B5a 采用最简单的 in-order interlock, 无 bypass):
//   ID stage 看 rs1_a / rs2_a, 与 ID/EX, EX/MEM, MEM/WB 中所有 reg_wen=1 且
//   valid=1 的 rd 比较. 若有命中 (且 rs 非 0), 则 ID 段 stall (id_hazard=1).
//   保险起见: 即使 wb_sel != load 也一律 interlock (开销最小, 正确性最高).
//   等到 WB 写回完成后下一拍, ID 才能继续.
//
// 分支预测: 不预测 (静态 not-taken), 也即 PC 默认走 PC+4. 分支结果在 EX 段
// 决议; 若 taken 或 jal / jalr -> 2 cycle bubble (flush IF/ID 与 ID/EX).
//
// 异常 (ecall/mret/ebreak): 在 WB 段处理, 最晚.
//   - ecall : flush 上游 (IF/ID, ID/EX, EX/MEM); PC <- mtvec; csr 写 mcause/mepc
//   - mret  : flush 上游; PC <- mepc
//   - ebreak: 与原 multi-cycle 实现一致, 不写回, 把 valid 一路保持. host (npc-soc
//     main_soc.cpp) 通过 inst_r (= MEM/WB.inst) 检测 0x00100073 并 ebreak.
//
// Access fault (io_fault):
//   ifu_fault: IF 段当拍 ifu_cpu_resp_valid & io_fault. 处理: 注入 bubble 到
//     IF/ID, PC <- 0 (沿用 D6a 的 "AXI bresp != OKAY -> jump 0" 协议).
//   lsu_fault: MEM 段当拍 io_lsu_respValid & io_fault. 处理: flush 整条流水线
//     上游, 把当前 in-flight 指令的 MEM/WB latch 置为 bubble (不让它写回),
//     PC <- 0.
//
// 性能计数器: icache 自带 (B4c). 这里只关心 cycle 总数.
// =============================================================================
`ifndef SOC_PC_RESET_VEC
  `define SOC_PC_RESET_VEC 32'h3000_0000
`endif

// B4a/B4b/B4c: icache 通过 `include 拉进来 (Makefile 不重写源文件列表).
`include "vsrc/icache.v"

module ysyx_22040000(
  input         clock,
  input         reset,
  // ---- IFU master (SimpleBus) ----------------------------------------------
  output        io_ifu_reqValid,
  output [31:0] io_ifu_addr,
  input         io_ifu_respValid,
  input  [31:0] io_ifu_rdata,
  // ---- LSU master (SimpleBus) ----------------------------------------------
  output        io_lsu_reqValid,
  output [31:0] io_lsu_addr,
  output [1:0]  io_lsu_size,
  output        io_lsu_wen,
  output [31:0] io_lsu_wdata,
  output [3:0]  io_lsu_wmask,
  input         io_lsu_respValid,
  input  [31:0] io_lsu_rdata,
  // ---- Access fault (B2a) --------------------------------------------------
  input         io_fault
);

  // ===========================================================================
  // PC register + redirect mux
  // ===========================================================================
  reg  [31:0] pc;
  wire [31:0] next_pc;

  // Redirect priority (highest -> lowest):
  //   1. trap_redirect (WB ecall  -> mtvec)
  //   2. mret_redirect (WB mret   -> mepc)
  //   3. branch_redirect (EX taken / jal / jalr)
  //   4. fault_redirect (any access fault -> 0)
  //   5. PC + 4 (advance) / hold (stall)
  wire        trap_redirect;
  wire        mret_redirect;
  wire [31:0] mtvec_w;
  wire [31:0] mepc_w;
  wire        branch_redirect;
  wire [31:0] branch_target;
  wire        any_fault;

  // ===========================================================================
  // Pipeline registers (declare up front so flush/stall logic can reference)
  // ===========================================================================
  // ---- IF/ID ---------------------------------------------------------------
  reg         if_id_valid;
  reg  [31:0] if_id_pc;
  reg  [31:0] if_id_inst;

  // ---- ID/EX ---------------------------------------------------------------
  reg         id_ex_valid;
  reg  [31:0] id_ex_pc;
  reg  [31:0] id_ex_inst;    // kept for debug + ebreak detect
  reg  [31:0] id_ex_rs1_val;
  reg  [31:0] id_ex_rs2_val;
  reg  [31:0] id_ex_imm;
  reg         id_ex_alu_src;
  reg  [3:0]  id_ex_alu_op;
  reg         id_ex_alu_use_pc;
  reg  [2:0]  id_ex_branch_op;
  reg         id_ex_mem_re;
  reg         id_ex_mem_we;
  reg  [2:0]  id_ex_funct3;
  reg  [1:0]  id_ex_wb_sel;
  reg         id_ex_reg_wen;
  reg  [3:0]  id_ex_rd;
  reg         id_ex_is_jal;
  reg         id_ex_is_jalr;
  reg         id_ex_is_branch;
  reg         id_ex_is_ebreak;
  reg         id_ex_is_mret;
  reg         id_ex_is_ecall;
  reg         id_ex_is_csr;
  reg  [11:0] id_ex_csr_addr;
  reg  [4:0]  id_ex_csr_uimm;
  reg         id_ex_csr_use_uimm;
  reg         id_ex_csr_we;
  reg  [1:0]  id_ex_csr_op;
  reg  [31:0] id_ex_csr_rdata;
  reg  [31:0] id_ex_a5_val;

  // ---- EX/MEM --------------------------------------------------------------
  reg         ex_mem_valid;
  reg  [31:0] ex_mem_pc;
  reg  [31:0] ex_mem_target_pc /*verilator public_flat_rd*/;   // B5b difftest
  reg  [31:0] ex_mem_inst;
  reg  [31:0] ex_mem_alu_result;
  reg         ex_mem_branch_taken;
  reg  [31:0] ex_mem_rs2_val;        // for store data
  reg  [31:0] ex_mem_imm;            // for jal/branch target (used in WB)
  reg         ex_mem_mem_re;
  reg         ex_mem_mem_we;
  reg  [2:0]  ex_mem_funct3;
  reg  [1:0]  ex_mem_wb_sel;
  reg         ex_mem_reg_wen;
  reg  [3:0]  ex_mem_rd;
  reg         ex_mem_is_jal;
  reg         ex_mem_is_jalr;
  reg         ex_mem_is_branch;
  reg         ex_mem_is_ebreak;
  reg         ex_mem_is_mret;
  reg         ex_mem_is_ecall;
  reg         ex_mem_is_csr;
  reg  [11:0] ex_mem_csr_addr;
  reg         ex_mem_csr_we;
  reg  [31:0] ex_mem_csr_wdata;      // computed in EX
  reg  [31:0] ex_mem_csr_rdata;
  reg  [31:0] ex_mem_a5_val;

  // ---- MEM/WB --------------------------------------------------------------
  reg         mem_wb_valid;
  reg  [31:0] mem_wb_pc;
  reg  [31:0] mem_wb_target_pc /*verilator public_flat_rd*/;  // B5b difftest
  reg  [31:0] mem_wb_inst;
  reg  [31:0] mem_wb_alu_result;
  reg  [31:0] mem_wb_load_data;
  reg  [31:0] mem_wb_csr_rdata;
  reg  [1:0]  mem_wb_wb_sel;
  reg         mem_wb_reg_wen;
  reg  [3:0]  mem_wb_rd;
  reg         mem_wb_is_jal;
  reg         mem_wb_is_jalr;
  reg         mem_wb_is_ebreak;
  reg         mem_wb_is_mret;
  reg         mem_wb_is_ecall;
  reg         mem_wb_csr_we;
  reg  [11:0] mem_wb_csr_addr;
  reg  [31:0] mem_wb_csr_wdata;
  reg  [31:0] mem_wb_a5_val;

  // B5b difftest: 本拍是否真正完成一次 retire (即 MEM/WB 刚 latched 进新指令).
  // 在 pipe_freeze (mem_stall/ebreak_park) 拍, mem_wb_* 全部 hold,
  // mem_wb_valid 看着是 1 但其实没新内容 — difftest 不能再 step REF.
  // 此信号在 pipe_freeze / inject_bubble 拍为 0, 其余拍 = 上拍 ex_mem_valid.
  reg         wb_advanced /*verilator public_flat_rd*/;

  // EX/MEM 段是否需要发起一个 bus 事务?
  wire mem_active = ex_mem_valid & (ex_mem_mem_re | ex_mem_mem_we);

  // ===========================================================================
  // IF stage: icache + (combinational) inst output
  // ===========================================================================
  // IF 段一直拉 req_valid, req_addr = PC. icache 内部:
  //   - hit  : 同 cycle resp_valid=1, resp_data=instr -> 立刻可 latch
  //   - miss : resp_valid=0 数拍, fill 完返回 resp_valid=1
  // 进入 reset 期间不拉请求, 避免触发 cache fill.
  // MemBridge 的 IFU/LSU 端口没有 reqReady，也不区分 AXI read response 属于
  // 哪个端口。MEM 段已有 load/store 等待时，不启动新的取指 miss；若取指
  // miss 已经先启动，则 icache 继续完成该 fill，LSU 在下面等待 IFU 空闲。
  wire        ifu_cpu_req_valid  = ~reset & ~mem_active;
  wire [31:0] ifu_cpu_req_addr   = {pc[31:2], 2'b00};
  wire        ifu_cpu_resp_valid;
  wire [31:0] ifu_cpu_resp_data;
  wire        ifu_flush;

  icache #(.BLOCKS_LOG(5), .WAYS_LOG(1), .OFFSET_LOG(5)) u_icache (
    .clock          (clock),
    .reset          (reset),
    .req_valid      (ifu_cpu_req_valid),
    .req_addr       (ifu_cpu_req_addr),
    .flush          (ifu_flush),
    .resp_valid     (ifu_cpu_resp_valid),
    .resp_data      (ifu_cpu_resp_data),
    .bus_req_valid  (io_ifu_reqValid),
    .bus_req_addr   (io_ifu_addr),
    .bus_resp_valid (io_ifu_respValid),
    .bus_resp_data  (io_ifu_rdata)
  );

  // IFU 段 fault: icache 上游 resp_valid=1 同周期 io_fault=1.
  // 在 fault 那拍仍然 "返回了 resp", 只是数据是 junk; 我们注入 bubble 到
  // IF/ID 并把 PC 跳到 0 (沿用 D6a 协议).
  wire ifu_fault = ifu_cpu_resp_valid & io_fault;

  // IF stage "本拍是否拿到一条有效指令" -- 决定下一拍 IF/ID 是 bubble 还是
  // 真实指令.
  wire if_has_inst = ifu_cpu_resp_valid & ~ifu_fault;
  wire [31:0] if_inst = ifu_cpu_resp_data;

  // ===========================================================================
  // ID stage: decode latched IF/ID inst + regfile read
  // ===========================================================================
  wire [31:0] id_inst = if_id_inst;
  wire [31:0] id_pc   = if_id_pc;

  // IDU outputs
  wire [3:0]  id_rs1_a, id_rs2_a, id_rd_a;
  wire [31:0] id_imm;
  wire        id_alu_src;
  wire [3:0]  id_alu_op;
  wire        id_alu_use_pc;
  wire        id_mem_re, id_mem_we;
  wire [2:0]  id_funct3;
  wire [1:0]  id_wb_sel;
  wire        id_reg_wen_dec;
  wire        id_is_jal;
  wire        id_is_jalr;
  wire        id_is_branch;
  wire [2:0]  id_branch_op;
  wire        id_is_ebreak;
  wire        id_is_mret;
  wire        id_is_ecall;
  wire        id_is_csr;
  wire [11:0] id_csr_addr;
  wire [4:0]  id_csr_uimm;
  wire        id_csr_use_uimm;
  wire        id_csr_re;
  wire        id_csr_we;
  wire [1:0]  id_csr_op;

  IDU u_idu (
    .inst(id_inst), .pc(id_pc),
    .rs1(id_rs1_a), .rs2(id_rs2_a), .rd(id_rd_a),
    .imm(id_imm), .alu_src(id_alu_src), .alu_op(id_alu_op), .alu_use_pc(id_alu_use_pc),
    .mem_re(id_mem_re), .mem_we(id_mem_we), .funct3(id_funct3),
    .wb_sel(id_wb_sel), .reg_wen(id_reg_wen_dec),
    .is_jal(id_is_jal), .is_jalr(id_is_jalr),
    .is_branch(id_is_branch), .branch_op(id_branch_op),
    .is_ebreak(id_is_ebreak), .is_mret(id_is_mret), .is_ecall(id_is_ecall),
    .is_csr(id_is_csr), .csr_addr(id_csr_addr),
    .csr_uimm(id_csr_uimm), .csr_use_uimm(id_csr_use_uimm),
    .csr_re(id_csr_re), .csr_we(id_csr_we), .csr_op(id_csr_op)
  );

  // ---- Register file (read in ID, write in WB) -----------------------------
  wire [31:0] id_rs1_val, id_rs2_val, id_a5_val;
  // wb_data computed below in WB section, but RegFile needs it as input -> use
  // a forward wire reference declared via "wire" early.
  wire [31:0] wb_data;
  wire        wb_reg_wen;
  wire [3:0]  wb_rd;

  RegFile #(.ADDR_WIDTH(4), .DATA_WIDTH(32)) u_rf (
    .clk(clock),
    .raddr1(id_rs1_a), .rdata1(id_rs1_val),
    .raddr2(id_rs2_a), .rdata2(id_rs2_val),
    .raddr3(4'd15),    .rdata3(id_a5_val),     // a5 for ecall cause
    .wdata(wb_data), .waddr(wb_rd), .wen(wb_reg_wen)
  );

  // ---- CSR (read in ID, write in WB) ---------------------------------------
  wire [31:0] id_csr_rdata;
  wire        wb_csr_we;
  wire [11:0] wb_csr_addr;
  wire [31:0] wb_csr_wdata;
  wire        wb_is_trap;
  wire [31:0] wb_trap_cause;
  wire [31:0] wb_trap_epc;

  CSR u_csr (
    .clk(clock), .rst(reset),
    .csr_raddr(id_csr_addr), .csr_rdata(id_csr_rdata),
    .csr_we(wb_csr_we),
    .csr_waddr(wb_csr_addr), .csr_wdata(wb_csr_wdata),
    .is_trap(wb_is_trap), .trap_cause(wb_trap_cause), .trap_epc(wb_trap_epc),
    .mepc_out(mepc_w), .mtvec_out(mtvec_w)
  );

  // ---- ID hazard detection (interlock, no bypass) --------------------------
  // 任何 rs1/rs2 与 ID/EX, EX/MEM, MEM/WB 中尚未写回的 rd 冲突, 都 stall ID.
  // 例外: rs0 永远是 0, 不算 hazard. id_reg_wen_dec=0 (e.g. ebreak/ecall) 时
  // 也不会产生 hazard 来源.
  // 注意: 必须把 valid 位算进去, bubble 的 reg_wen 即使是 1 也不算.
  wire id_uses_rs1 = (id_rs1_a != 4'd0);
  wire id_uses_rs2 = (id_rs2_a != 4'd0);

  wire ex_writes  = id_ex_valid  & id_ex_reg_wen  & (id_ex_rd  != 4'd0);
  wire mem_writes = ex_mem_valid & ex_mem_reg_wen & (ex_mem_rd != 4'd0);
  wire wb_writes  = mem_wb_valid & mem_wb_reg_wen & (mem_wb_rd != 4'd0);

  wire hz_ex_rs1  = ex_writes  & id_uses_rs1 & (id_ex_rd  == id_rs1_a);
  wire hz_ex_rs2  = ex_writes  & id_uses_rs2 & (id_ex_rd  == id_rs2_a);
  wire hz_mem_rs1 = mem_writes & id_uses_rs1 & (ex_mem_rd == id_rs1_a);
  wire hz_mem_rs2 = mem_writes & id_uses_rs2 & (ex_mem_rd == id_rs2_a);
  wire hz_wb_rs1  = wb_writes  & id_uses_rs1 & (mem_wb_rd == id_rs1_a);
  wire hz_wb_rs2  = wb_writes  & id_uses_rs2 & (mem_wb_rd == id_rs2_a);

  // 只有 IF/ID 有效时才需要触发 hazard (空 bubble 走 ID 是无害的).
  wire id_hazard = if_id_valid
                 & (hz_ex_rs1 | hz_ex_rs2 | hz_mem_rs1 | hz_mem_rs2 | hz_wb_rs1 | hz_wb_rs2);

  // ===========================================================================
  // EX stage: ALU + branch decision + CSR wdata compute
  // ===========================================================================
  wire [31:0] ex_alu_result;
  wire        ex_branch_taken;
  EXU u_exu (
    .pc(id_ex_pc),
    .rs1_val(id_ex_rs1_val), .rs2_val(id_ex_rs2_val), .imm(id_ex_imm),
    .alu_src(id_ex_alu_src), .alu_use_pc(id_ex_alu_use_pc), .alu_op(id_ex_alu_op),
    .branch_op(id_ex_branch_op),
    .alu_result(ex_alu_result), .branch_taken(ex_branch_taken)
  );

  // CSR wdata computed in EX (using id_ex_csr_rdata that was read in ID).
  wire [31:0] ex_csr_src =
        id_ex_csr_use_uimm ? {27'b0, id_ex_csr_uimm} : id_ex_rs1_val;
  wire [31:0] ex_csr_wdata =
        (id_ex_csr_op == 2'b00) ? ex_csr_src :                          // csrrw / csrrwi
        (id_ex_csr_op == 2'b01) ? (id_ex_csr_rdata | ex_csr_src) :      // csrrs / csrrsi
        (id_ex_csr_op == 2'b10) ? (id_ex_csr_rdata & ~ex_csr_src) :     // csrrc / csrrci
                                  ex_csr_src;

  // ---- EX branch redirect --------------------------------------------------
  // taken 条件: jal | jalr | (branch & branch_taken)
  // 注意 ebreak 不会 redirect (它在 WB 被 parked). ecall/mret 在 WB 段 redirect,
  // EX 段不动 PC.
  wire ex_take_jump =
        id_ex_valid &
        (id_ex_is_jal | id_ex_is_jalr | (id_ex_is_branch & ex_branch_taken));

  // branch_target 计算:
  //   - jal     : pc + imm_j
  //   - jalr    : (rs1 + imm) & ~1  -- 由 EXU alu_result 提供 (alu_op=1100)
  //   - branch  : pc + imm_b
  // ALU 输出对于 jalr 已经是 target; 对于 jal/branch 我们直接 pc + imm.
  wire [31:0] ex_pc_plus_imm  = id_ex_pc + id_ex_imm;
  wire [31:0] ex_jump_target =
        id_ex_is_jalr ? ex_alu_result :
                        ex_pc_plus_imm;

  assign branch_redirect = ex_take_jump;
  assign branch_target   = ex_jump_target;

  // B5b difftest: 计算 retire 段的 "next PC" (= 该指令执行完后处理器接下来要取的 PC).
  // 顺序指令: pc + 4. 控制转移: branch_target. ecall/mret 在 WB 段后续 override.
  wire [31:0] ex_inst_next_pc =
        ex_take_jump ? ex_jump_target : (id_ex_pc + 32'd4);

  // ===========================================================================
  // MEM stage: SimpleBus LSU master + load decode
  // ===========================================================================
  // ---- store word / wmask / size (D6a logic, recomputed from EX/MEM regs) --
  wire [1:0] mem_byte_off = ex_mem_alu_result[1:0];
  reg  [31:0] mem_store_word;
  reg  [3:0]  mem_store_wmask;
  reg  [1:0]  mem_store_size;
  always @(*) begin
    mem_store_word  = 32'h0;
    mem_store_wmask = 4'b0;
    mem_store_size  = 2'b10;
    if (ex_mem_mem_we) begin
      case (ex_mem_funct3)
        3'b010: begin                                            // sw
          mem_store_word  = ex_mem_rs2_val;
          mem_store_wmask = 4'b1111;
          mem_store_size  = 2'b10;
        end
        3'b001: begin                                            // sh
          case (mem_byte_off[1])
            1'b0: begin mem_store_word = {16'h0, ex_mem_rs2_val[15:0]};        mem_store_wmask = 4'b0011; end
            1'b1: begin mem_store_word = {ex_mem_rs2_val[15:0], 16'h0};        mem_store_wmask = 4'b1100; end
          endcase
          mem_store_size = 2'b01;
        end
        3'b000: begin                                            // sb
          case (mem_byte_off)
            2'b00: begin mem_store_word = {24'h0, ex_mem_rs2_val[7:0]};                 mem_store_wmask = 4'b0001; end
            2'b01: begin mem_store_word = {16'h0, ex_mem_rs2_val[7:0], 8'h0};            mem_store_wmask = 4'b0010; end
            2'b10: begin mem_store_word = {8'h0,  ex_mem_rs2_val[7:0], 16'h0};           mem_store_wmask = 4'b0100; end
            2'b11: begin mem_store_word = {       ex_mem_rs2_val[7:0], 24'h0};           mem_store_wmask = 4'b1000; end
          endcase
          mem_store_size = 2'b00;
        end
        default: begin
          mem_store_word  = ex_mem_rs2_val;
          mem_store_wmask = 4'b1111;
          mem_store_size  = 2'b10;
        end
      endcase
    end
  end

  reg [1:0] mem_load_size;
  always @(*) begin
    case (ex_mem_funct3)
      3'b000: mem_load_size = 2'b00;     // lb
      3'b001: mem_load_size = 2'b01;     // lh
      3'b010: mem_load_size = 2'b10;     // lw
      3'b100: mem_load_size = 2'b00;     // lbu
      3'b101: mem_load_size = 2'b01;     // lhu
      default: mem_load_size = 2'b10;
    endcase
  end

  // B5a fix: IFU 和 LSU 共享 SoC MemBridge 的 AXI master 通道. icache 在
  // 多 beat fill 期间持续拉 io_ifu_reqValid (8 拍), 若同时 LSU 发起 reqValid,
  // MemBridge stateI / stateD 会一起进 R_WAIT, 争抢同一个 io_master_rvalid
  // 导致 icache fill_cnt 卡住. 简化办法: LSU 在 IFU 占用 AXI 通道期间等待.
  assign io_lsu_reqValid = mem_active & ~reset & ~io_ifu_reqValid;
  assign io_lsu_addr     = ex_mem_alu_result;
  assign io_lsu_size     = ex_mem_mem_we ? mem_store_size : mem_load_size;
  assign io_lsu_wen      = ex_mem_mem_we;
  assign io_lsu_wdata    = mem_store_word;
  assign io_lsu_wmask    = mem_store_wmask;

  // LSU fault: 当 MEM 段在等 resp 且 io_fault=1.
  wire lsu_fault = mem_active & io_lsu_respValid & io_fault;
  // MEM stall: 当本段需要 bus 事务但 resp 还没回. (lsu_fault 那拍也算 resp 回了)
  wire mem_stall = mem_active & ~io_lsu_respValid;

  // 加载结果: 用本拍 io_lsu_rdata 做 align + 符号扩展.
  reg [31:0] mem_load_data;
  reg [7:0]  mem_ld_byte_sel;
  reg [15:0] mem_ld_half_sel;
  always @(*) begin
    case (mem_byte_off)
      2'b00:   mem_ld_byte_sel = io_lsu_rdata[ 7: 0];
      2'b01:   mem_ld_byte_sel = io_lsu_rdata[15: 8];
      2'b10:   mem_ld_byte_sel = io_lsu_rdata[23:16];
      2'b11:   mem_ld_byte_sel = io_lsu_rdata[31:24];
      default: mem_ld_byte_sel = 8'h0;
    endcase
    case (mem_byte_off[1])
      1'b0:    mem_ld_half_sel = io_lsu_rdata[15: 0];
      1'b1:    mem_ld_half_sel = io_lsu_rdata[31:16];
      default: mem_ld_half_sel = 16'h0;
    endcase
    mem_load_data = 32'h0;
    if (ex_mem_mem_re) begin
      case (ex_mem_funct3)
        3'b000: mem_load_data = {{24{mem_ld_byte_sel[7]}},  mem_ld_byte_sel};   // lb
        3'b001: mem_load_data = {{16{mem_ld_half_sel[15]}}, mem_ld_half_sel};   // lh
        3'b010: mem_load_data = io_lsu_rdata;                                    // lw
        3'b100: mem_load_data = {24'h0, mem_ld_byte_sel};                        // lbu
        3'b101: mem_load_data = {16'h0, mem_ld_half_sel};                        // lhu
        default: mem_load_data = io_lsu_rdata;
      endcase
    end
  end

  // ===========================================================================
  // WB stage: write regfile / CSR / handle trap-mret-ebreak / select next PC
  // ===========================================================================
  // 本拍 WB 指令是否还有效? 在 ebreak 之后 mem_wb_valid 还会保持, 但我们在
  // ebreak 拍及之后冻结整条流水线, 也不重复写回 (用 ebreak_park 控制).
  wire wb_v = mem_wb_valid;

  // wb_data mux (与 D6a WBU 实例一致).
  assign wb_data =
      (mem_wb_wb_sel == 2'b01) ? mem_wb_load_data :
      (mem_wb_wb_sel == 2'b10) ? (mem_wb_pc + 32'd4) :  // jal/jalr return addr
      (mem_wb_wb_sel == 2'b11) ? mem_wb_csr_rdata :
                                 mem_wb_alu_result;

  // ebreak 处理: 进入 WB 之后 park; 不写回 regfile / CSR / PC.
  // ebreak_park 是一个 sticky reg: 一旦看到 ebreak 在 WB 当拍, 之后所有
  // pipeline 行为都 freeze.
  reg ebreak_park;
  always @(posedge clock) begin
    if (reset)                                     ebreak_park <= 1'b0;
    else if (wb_v & mem_wb_is_ebreak & ~any_fault) ebreak_park <= 1'b1;
  end

  assign wb_reg_wen = wb_v & mem_wb_reg_wen & ~mem_wb_is_ebreak & ~ebreak_park;
  assign wb_rd      = mem_wb_rd;

  assign wb_csr_we    = wb_v & mem_wb_csr_we & ~mem_wb_is_ebreak & ~ebreak_park;
  assign wb_csr_addr  = mem_wb_csr_addr;
  assign wb_csr_wdata = mem_wb_csr_wdata;
  assign wb_is_trap   = wb_v & mem_wb_is_ecall & ~ebreak_park & ~reset;
  assign wb_trap_cause = mem_wb_a5_val;
  assign wb_trap_epc   = mem_wb_pc;

  assign trap_redirect = wb_is_trap;
  assign mret_redirect = wb_v & mem_wb_is_mret & ~ebreak_park;

  // ===========================================================================
  // Stall / Flush combinational
  // ===========================================================================
  // IF miss: icache 还没回 resp_valid -> IF 没拿到指令.
  wire if_miss = ~ifu_cpu_resp_valid;

  // 全局: pipeline 在 ebreak_park 时不再推进
  wire global_stall = ebreak_park;

  // ===========================================================================
  // PC update logic
  // ===========================================================================
  // next_pc 决策 (从最高优先级开始):
  //   reset                -> PC_RESET_VEC
  //   ebreak_park          -> hold
  //   any_fault            -> 0
  //   trap_redirect (WB)   -> mtvec
  //   mret_redirect (WB)   -> mepc
  //   branch_redirect (EX) -> branch_target
  //   mem_stall            -> hold
  //   id_hazard            -> hold
  //   if_miss              -> hold
  //   else                 -> PC + 4
  // pipe_freeze (mem_stall) 强制 hold; 否则按下面优先级决定.
  assign next_pc =
       pipe_freeze        ? pc         :
       any_fault          ? 32'h0      :
       trap_redirect      ? mtvec_w    :
       mret_redirect      ? mepc_w     :
       branch_redirect    ? branch_target :
       (id_hazard | if_miss) ? pc      :
                            (pc + 32'd4);

  // ===========================================================================
  // Access fault aggregation
  // ===========================================================================
  assign any_fault = ifu_fault | lsu_fault;

  // Sticky fault counter, kept for parity with D6a.
  reg [31:0] fault_count;
  always @(posedge clock) begin
    if (reset)              fault_count <= 32'h0;
    else if (any_fault)     fault_count <= fault_count + 32'h1;
  end

  // ===========================================================================
  // Pipeline register update (sequential)
  // ===========================================================================
  // 综合 stall/flush 信号:
  //   pipe_freeze = ebreak_park | mem_stall
  //     - mem_stall 时整条流水线全部 freeze (PC 不动, 所有 latch 不变).
  //   id_freeze (= pipe_freeze | id_hazard | if_miss):
  //     - 当 id_hazard, IF/ID 不更新 (IF 段当拍取到的指令保留, 下一拍重读).
  //     - 当 if_miss, IF 段没拿到指令, IF/ID 也不更新 (保持旧条目, 或保持 bubble).
  //   id_ex_bubble (= pipe_freeze | id_hazard | flush_ex | flush_wb):
  //     - id_hazard 时 ID/EX 注入 bubble.
  //     - branch_redirect (= flush_ex) 时 ID/EX 注入 bubble (清掉 ID 已经
  //       decode 完的下一条指令).
  //     - trap/mret (= flush_wb) 时 ID/EX 注入 bubble.
  //   ex_mem_bubble (= flush_wb):
  //     - branch_redirect 不影响 EX/MEM (那一拍 EX 是真正分支指令本身, 要让它
  //       继续往下走 -> 不 bubble. flush_ex 只杀更年轻的 ID/EX 和 IF/ID).
  //     - trap/mret 在 WB 段触发 -> 必须把 EX/MEM (年轻于 WB 当前指令) 杀掉.
  //   mem_wb_bubble (= 0, 因为 mem_stall 时整条 freeze 就不会 latch 新东西)
  //
  // 单独处理 fault: any_fault 那一拍, mem_wb 的 valid 应该被强制 bubble (防止
  // 让带 junk 的 in-flight 指令写回). 我们直接让 mem_wb_valid <= 0 来实现.
  wire flush_ex = branch_redirect & ~ebreak_park;
  wire flush_wb = (trap_redirect | mret_redirect) & ~ebreak_park;
  wire flush_fault = any_fault;
  assign ifu_flush = flush_ex | flush_wb | flush_fault;

  wire pipe_freeze = ebreak_park | mem_stall;
  wire id_ex_inject_bubble = pipe_freeze ? 1'b0 :
                             (id_hazard | flush_ex | flush_wb | flush_fault);
  wire ex_mem_inject_bubble = pipe_freeze ? 1'b0 :
                              (flush_wb | flush_fault);
  wire mem_wb_inject_bubble = pipe_freeze ? 1'b0 :
                              (flush_fault);

  // ===========================================================================
  // PC sequential update
  // ===========================================================================
  always @(posedge clock) begin
    if (reset) begin
      pc <= `SOC_PC_RESET_VEC;
    end else if (ebreak_park) begin
      pc <= pc;
    end else begin
      pc <= next_pc;
    end
  end

  // ===========================================================================
  // Pipeline register sequential update
  // ===========================================================================
  // --- IF/ID --------------------------------------------------------------
  always @(posedge clock) begin
    if (reset) begin
      if_id_valid <= 1'b0;
      if_id_pc    <= 32'h0;
      if_id_inst  <= 32'h0;
    end else if (pipe_freeze) begin
      // freeze (mem_stall / ebreak_park): keep IF/ID intact
    end else if (flush_ex | flush_wb | flush_fault) begin
      // branch/trap/fault flush: bubble out IF/ID
      if_id_valid <= 1'b0;
      // 仍然写一份 pc/inst, 方便调试 trace (不影响功能, 反正 valid=0).
      if_id_pc    <= pc;
      if_id_inst  <= 32'h0;
    end else if (id_hazard) begin
      // ID stall: 保持 IF/ID 内容. 不接受 IF 当拍的新指令.
      // 注意: 这一拍 IF 段如果 ifu_cpu_resp_valid=1, 我们还是会读到指令, 但
      // 没有 latch 进 IF/ID, 下一拍 IF 段会重新发请求 (PC hold) 再读一次. 由于
      // icache hit 一拍出, 这没问题; miss 时 PC hold + req_valid 持续, 也兼容.
    end else if (if_has_inst) begin
      // 正常推进: 把当拍 IF 取到的指令 latch 到 IF/ID
      if_id_valid <= 1'b1;
      if_id_pc    <= pc;
      if_id_inst  <= if_inst;
    end else begin
      // if_miss / ifu_fault: 注入 bubble
      if_id_valid <= 1'b0;
      if_id_pc    <= pc;
      if_id_inst  <= 32'h0;
    end
  end

  // --- ID/EX --------------------------------------------------------------
  always @(posedge clock) begin
    if (reset) begin
      id_ex_valid       <= 1'b0;
      id_ex_pc          <= 32'h0;
      id_ex_inst        <= 32'h0;
      id_ex_rs1_val     <= 32'h0;
      id_ex_rs2_val     <= 32'h0;
      id_ex_imm         <= 32'h0;
      id_ex_alu_src     <= 1'b0;
      id_ex_alu_op      <= 4'b0;
      id_ex_alu_use_pc  <= 1'b0;
      id_ex_branch_op   <= 3'b0;
      id_ex_mem_re      <= 1'b0;
      id_ex_mem_we      <= 1'b0;
      id_ex_funct3      <= 3'b0;
      id_ex_wb_sel      <= 2'b0;
      id_ex_reg_wen     <= 1'b0;
      id_ex_rd          <= 4'b0;
      id_ex_is_jal      <= 1'b0;
      id_ex_is_jalr     <= 1'b0;
      id_ex_is_branch   <= 1'b0;
      id_ex_is_ebreak   <= 1'b0;
      id_ex_is_mret     <= 1'b0;
      id_ex_is_ecall    <= 1'b0;
      id_ex_is_csr      <= 1'b0;
      id_ex_csr_addr    <= 12'b0;
      id_ex_csr_uimm    <= 5'b0;
      id_ex_csr_use_uimm<= 1'b0;
      id_ex_csr_we      <= 1'b0;
      id_ex_csr_op      <= 2'b0;
      id_ex_csr_rdata   <= 32'h0;
      id_ex_a5_val      <= 32'h0;
    end else if (pipe_freeze) begin
      // freeze
    end else if (id_ex_inject_bubble) begin
      id_ex_valid     <= 1'b0;
      // 控制信号清干净 (防止 X 通过组合到 EX -- 实际上 EX 在 valid=0 时不
      // 影响下游, 但 verilator 默认 X 检测会噪).
      id_ex_reg_wen   <= 1'b0;
      id_ex_mem_re    <= 1'b0;
      id_ex_mem_we    <= 1'b0;
      id_ex_is_jal    <= 1'b0;
      id_ex_is_jalr   <= 1'b0;
      id_ex_is_branch <= 1'b0;
      id_ex_is_ebreak <= 1'b0;
      id_ex_is_mret   <= 1'b0;
      id_ex_is_ecall  <= 1'b0;
      id_ex_is_csr    <= 1'b0;
      id_ex_csr_we    <= 1'b0;
      id_ex_rd        <= 4'b0;
    end else begin
      // 正常 latch: 把 ID 段组合结果存进来. 如果 IF/ID 是 bubble, 那么 ID 段
      // decode 的也是 bubble (inst=0 -> opcode=0 -> 无副作用); 我们用 valid
      // = if_id_valid 来表达, 这样 IDU 即使 decode 出 reg_wen=1 也不会被下游
      // 看到 (hazard / wb_reg_wen 都要看 valid).
      id_ex_valid       <= if_id_valid;
      id_ex_pc          <= if_id_pc;
      id_ex_inst        <= if_id_inst;
      id_ex_rs1_val     <= id_rs1_val;
      id_ex_rs2_val     <= id_rs2_val;
      id_ex_imm         <= id_imm;
      id_ex_alu_src     <= id_alu_src;
      id_ex_alu_op      <= id_alu_op;
      id_ex_alu_use_pc  <= id_alu_use_pc;
      id_ex_branch_op   <= id_branch_op;
      id_ex_mem_re      <= id_mem_re;
      id_ex_mem_we      <= id_mem_we;
      id_ex_funct3      <= id_funct3;
      id_ex_wb_sel      <= id_wb_sel;
      id_ex_reg_wen     <= id_reg_wen_dec;
      id_ex_rd          <= id_rd_a;
      id_ex_is_jal      <= id_is_jal;
      id_ex_is_jalr     <= id_is_jalr;
      id_ex_is_branch   <= id_is_branch;
      id_ex_is_ebreak   <= id_is_ebreak;
      id_ex_is_mret     <= id_is_mret;
      id_ex_is_ecall    <= id_is_ecall;
      id_ex_is_csr      <= id_is_csr;
      id_ex_csr_addr    <= id_csr_addr;
      id_ex_csr_uimm    <= id_csr_uimm;
      id_ex_csr_use_uimm<= id_csr_use_uimm;
      id_ex_csr_we      <= id_csr_we;
      id_ex_csr_op      <= id_csr_op;
      id_ex_csr_rdata   <= id_csr_rdata;
      id_ex_a5_val      <= id_a5_val;
    end
  end

  // --- EX/MEM -------------------------------------------------------------
  always @(posedge clock) begin
    if (reset) begin
      ex_mem_valid        <= 1'b0;
      ex_mem_pc           <= 32'h0;
      ex_mem_target_pc    <= 32'h0;
      ex_mem_inst         <= 32'h0;
      ex_mem_alu_result   <= 32'h0;
      ex_mem_branch_taken <= 1'b0;
      ex_mem_rs2_val      <= 32'h0;
      ex_mem_imm          <= 32'h0;
      ex_mem_mem_re       <= 1'b0;
      ex_mem_mem_we       <= 1'b0;
      ex_mem_funct3       <= 3'b0;
      ex_mem_wb_sel       <= 2'b0;
      ex_mem_reg_wen      <= 1'b0;
      ex_mem_rd           <= 4'b0;
      ex_mem_is_jal       <= 1'b0;
      ex_mem_is_jalr      <= 1'b0;
      ex_mem_is_branch    <= 1'b0;
      ex_mem_is_ebreak    <= 1'b0;
      ex_mem_is_mret      <= 1'b0;
      ex_mem_is_ecall     <= 1'b0;
      ex_mem_is_csr       <= 1'b0;
      ex_mem_csr_addr     <= 12'b0;
      ex_mem_csr_we       <= 1'b0;
      ex_mem_csr_wdata    <= 32'h0;
      ex_mem_csr_rdata    <= 32'h0;
      ex_mem_a5_val       <= 32'h0;
    end else if (pipe_freeze) begin
      // freeze
    end else if (ex_mem_inject_bubble) begin
      ex_mem_valid     <= 1'b0;
      ex_mem_reg_wen   <= 1'b0;
      ex_mem_mem_re    <= 1'b0;
      ex_mem_mem_we    <= 1'b0;
      ex_mem_is_ebreak <= 1'b0;
      ex_mem_is_mret   <= 1'b0;
      ex_mem_is_ecall  <= 1'b0;
      ex_mem_is_csr    <= 1'b0;
      ex_mem_csr_we    <= 1'b0;
      ex_mem_rd        <= 4'b0;
    end else begin
      ex_mem_valid        <= id_ex_valid;
      ex_mem_pc           <= id_ex_pc;
      ex_mem_target_pc    <= ex_inst_next_pc;
      ex_mem_inst         <= id_ex_inst;
      ex_mem_alu_result   <= ex_alu_result;
      ex_mem_branch_taken <= ex_branch_taken;
      ex_mem_rs2_val      <= id_ex_rs2_val;
      ex_mem_imm          <= id_ex_imm;
      ex_mem_mem_re       <= id_ex_mem_re;
      ex_mem_mem_we       <= id_ex_mem_we;
      ex_mem_funct3       <= id_ex_funct3;
      ex_mem_wb_sel       <= id_ex_wb_sel;
      ex_mem_reg_wen      <= id_ex_reg_wen;
      ex_mem_rd           <= id_ex_rd;
      ex_mem_is_jal       <= id_ex_is_jal;
      ex_mem_is_jalr      <= id_ex_is_jalr;
      ex_mem_is_branch    <= id_ex_is_branch;
      ex_mem_is_ebreak    <= id_ex_is_ebreak;
      ex_mem_is_mret      <= id_ex_is_mret;
      ex_mem_is_ecall     <= id_ex_is_ecall;
      ex_mem_is_csr       <= id_ex_is_csr;
      ex_mem_csr_addr     <= id_ex_csr_addr;
      ex_mem_csr_we       <= id_ex_csr_we;
      ex_mem_csr_wdata    <= ex_csr_wdata;
      ex_mem_csr_rdata    <= id_ex_csr_rdata;
      ex_mem_a5_val       <= id_ex_a5_val;
    end
  end

  // --- MEM/WB -------------------------------------------------------------
  always @(posedge clock) begin
    if (reset) begin
      mem_wb_valid      <= 1'b0;
      mem_wb_pc         <= 32'h0;
      mem_wb_target_pc  <= 32'h0;
      mem_wb_inst       <= 32'h0;
      mem_wb_alu_result <= 32'h0;
      mem_wb_load_data  <= 32'h0;
      mem_wb_csr_rdata  <= 32'h0;
      mem_wb_wb_sel     <= 2'b0;
      mem_wb_reg_wen    <= 1'b0;
      mem_wb_rd         <= 4'b0;
      mem_wb_is_jal     <= 1'b0;
      mem_wb_is_jalr    <= 1'b0;
      mem_wb_is_ebreak  <= 1'b0;
      mem_wb_is_mret    <= 1'b0;
      mem_wb_is_ecall   <= 1'b0;
      mem_wb_csr_we     <= 1'b0;
      mem_wb_csr_addr   <= 12'b0;
      mem_wb_csr_wdata  <= 32'h0;
      mem_wb_a5_val     <= 32'h0;
    end else if (pipe_freeze) begin
      // freeze
    end else if (mem_wb_inject_bubble) begin
      mem_wb_valid    <= 1'b0;
      mem_wb_reg_wen  <= 1'b0;
      mem_wb_is_ebreak<= 1'b0;
      mem_wb_is_mret  <= 1'b0;
      mem_wb_is_ecall <= 1'b0;
      mem_wb_csr_we   <= 1'b0;
      mem_wb_rd       <= 4'b0;
    end else begin
      // 把 MEM 段计算好的 load_data + 把 EX/MEM 控制信号传到 MEM/WB.
      // mem_wb_load_data 只对 mem_re=1 的指令有意义.
      mem_wb_valid      <= ex_mem_valid;
      mem_wb_pc         <= ex_mem_pc;
      mem_wb_target_pc  <= ex_mem_target_pc;
      mem_wb_inst       <= ex_mem_inst;
      mem_wb_alu_result <= ex_mem_alu_result;
      mem_wb_load_data  <= mem_load_data;
      mem_wb_csr_rdata  <= ex_mem_csr_rdata;
      mem_wb_wb_sel     <= ex_mem_wb_sel;
      mem_wb_reg_wen    <= ex_mem_reg_wen;
      mem_wb_rd         <= ex_mem_rd;
      mem_wb_is_jal     <= ex_mem_is_jal;
      mem_wb_is_jalr    <= ex_mem_is_jalr;
      mem_wb_is_ebreak  <= ex_mem_is_ebreak;
      mem_wb_is_mret    <= ex_mem_is_mret;
      mem_wb_is_ecall   <= ex_mem_is_ecall;
      mem_wb_csr_we     <= ex_mem_csr_we;
      mem_wb_csr_addr   <= ex_mem_csr_addr;
      mem_wb_csr_wdata  <= ex_mem_csr_wdata;
      mem_wb_a5_val     <= ex_mem_a5_val;
    end
  end

  // wb_advanced: 本拍 MEM/WB 是否真的吸入新指令. freeze 时整条流水线 hold,
  // mem_wb_valid 会保持高但没有新指令到位, difftest 不能据此 step REF.
  always @(posedge clock) begin
    wb_advanced <= ~reset & ~pipe_freeze & ~mem_wb_inject_bubble & ex_mem_valid;
  end

endmodule
