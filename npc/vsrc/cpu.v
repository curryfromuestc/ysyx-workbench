// Top-level single-cycle RV32E_Zicsr CPU.
//
// PC reset is configurable via the PC_RESET_VEC macro (default 0x80000000).
// The SoC wrapper (ysyx_22040000.v) provides its own 0x30000000 instance.
//
// Pipeline (still single-cycle):
//   IFU -> IDU -> EXU -> LSU -> WBU -> PC update
//
// PC redirect (handled in WBU):
//   mret      -> mepc
//   jal/jalr  -> imm-relative / (rs1+imm)&~1
//   branch    -> rs1?rs2 comparison
//   default   -> pc + 4
//
// `SYNTHESIS` strips the DPI-C trap hook and exposes ebreak status as ports
// so yosys can lint the top.

`ifndef PC_RESET_VEC
  `define PC_RESET_VEC 32'h8000_0000
`endif

module cpu(
  input clk,
  input rst
`ifdef SYNTHESIS
  ,
  output [31:0] ifu_pmem_raddr,
  input  [31:0] ifu_pmem_rdata,
  output [31:0] lsu_pmem_raddr,
  input  [31:0] lsu_pmem_rdata,
  output        lsu_pmem_wen,
  output [31:0] lsu_pmem_waddr,
  output [31:0] lsu_pmem_wdata,
  output [7:0]  lsu_pmem_wmask,
  output        npc_ebreak,
  output [31:0] npc_a0
`endif
);
  reg  [31:0] pc;
  wire [31:0] pc_next;
  wire [31:0] inst;

  wire [3:0]  rs1, rs2, rd;
  wire [31:0] imm;
  wire        alu_src;
  wire [3:0]  alu_op;
  wire        alu_use_pc;
  wire        mem_re, mem_we;
  wire [2:0]  funct3;
  wire [1:0]  wb_sel;
  wire        reg_wen;
  wire        is_jal;
  wire        is_jalr;
  wire        is_branch;
  wire [2:0]  branch_op;
  wire        is_ebreak;
  wire        is_mret;

  wire        is_csr;
  wire [11:0] csr_addr_i;
  wire [4:0]  csr_uimm;
  wire        csr_use_uimm;
  wire        csr_re_i;
  wire        csr_we_i;
  wire [1:0]  csr_op;
  wire [31:0] csr_rdata;
  wire [31:0] mepc_w;

  wire [31:0] rs1_val, rs2_val;
  wire [31:0] alu_result;
  wire        branch_taken;
  wire [31:0] load_data;
  wire [31:0] wb_data;

  // PC register
  always @(posedge clk) begin
    if (rst) pc <= `PC_RESET_VEC;
    else     pc <= pc_next;
  end

  IFU u_ifu (.pc(pc), .inst(inst)
`ifdef SYNTHESIS
    , .pmem_raddr(ifu_pmem_raddr), .pmem_rdata(ifu_pmem_rdata)
`endif
  );

  IDU u_idu (
    .inst(inst), .pc(pc),
    .rs1(rs1), .rs2(rs2), .rd(rd),
    .imm(imm), .alu_src(alu_src), .alu_op(alu_op), .alu_use_pc(alu_use_pc),
    .mem_re(mem_re), .mem_we(mem_we), .funct3(funct3),
    .wb_sel(wb_sel), .reg_wen(reg_wen),
    .is_jal(is_jal), .is_jalr(is_jalr),
    .is_branch(is_branch), .branch_op(branch_op),
    .is_ebreak(is_ebreak), .is_mret(is_mret),
    .is_csr(is_csr), .csr_addr(csr_addr_i),
    .csr_uimm(csr_uimm), .csr_use_uimm(csr_use_uimm),
    .csr_re(csr_re_i), .csr_we(csr_we_i), .csr_op(csr_op)
  );

  RegFile #(.ADDR_WIDTH(4), .DATA_WIDTH(32)) u_rf (
    .clk(clk),
    .raddr1(rs1), .rdata1(rs1_val),
    .raddr2(rs2), .rdata2(rs2_val),
    .wdata(wb_data), .waddr(rd), .wen(reg_wen & ~rst & ~is_ebreak)
  );

  EXU u_exu (
    .pc(pc),
    .rs1_val(rs1_val), .rs2_val(rs2_val), .imm(imm),
    .alu_src(alu_src), .alu_use_pc(alu_use_pc), .alu_op(alu_op),
    .branch_op(branch_op),
    .alu_result(alu_result), .branch_taken(branch_taken)
  );

  LSU u_lsu (
    .addr(alu_result), .store_data(rs2_val),
    .mem_re(mem_re & ~rst), .mem_we(mem_we & ~rst),
    .funct3(funct3),
    .load_data(load_data)
`ifdef SYNTHESIS
    , .pmem_raddr(lsu_pmem_raddr), .pmem_rdata(lsu_pmem_rdata),
    .pmem_wen(lsu_pmem_wen), .pmem_waddr(lsu_pmem_waddr),
    .pmem_wdata(lsu_pmem_wdata), .pmem_wmask(lsu_pmem_wmask)
`endif
  );

  // ---- CSR ------------------------------------------------------------------
  wire [31:0] csr_src = csr_use_uimm ? {27'b0, csr_uimm} : rs1_val;
  wire [31:0] csr_wdata =
        (csr_op == 2'b00) ? csr_src :              // csrrw / csrrwi
        (csr_op == 2'b01) ? (csr_rdata | csr_src): // csrrs / csrrsi
        (csr_op == 2'b10) ? (csr_rdata & ~csr_src):// csrrc / csrrci
                            csr_src;
  CSR u_csr (
    .clk(clk), .rst(rst),
    .csr_raddr(csr_addr_i), .csr_rdata(csr_rdata),
    .csr_we(csr_we_i & ~rst & ~is_ebreak),
    .csr_waddr(csr_addr_i), .csr_wdata(csr_wdata),
    .mepc_out(mepc_w)
  );

  WBU u_wbu (
    .alu_result(alu_result), .load_data(load_data),
    .csr_rdata(csr_rdata),
    .wb_sel(wb_sel),
    .is_jal(is_jal), .is_jalr(is_jalr),
    .is_branch(is_branch), .branch_taken(branch_taken),
    .is_mret(is_mret), .mepc(mepc_w),
    .pc(pc), .imm(imm),
    .pc_next(pc_next), .wb_data(wb_data)
  );

  // ebreak hookup: synchronously notify the host.
`ifdef SYNTHESIS
  RegFile_PORT_X10 u_rf_x10 (
    .clk(clk),
    .wen(reg_wen & ~rst & ~is_ebreak),
    .waddr(rd), .wdata(wb_data),
    .x10(npc_a0)
  );
  assign npc_ebreak = is_ebreak & ~rst;
`else
  import "DPI-C" function void npc_trap(input int code);
  always @(posedge clk) begin
    if (!rst && is_ebreak) begin
      npc_trap(u_rf.rf[10]);
    end
  end
`endif
endmodule

`ifdef SYNTHESIS
// Shadow of regfile x10 used so the top-level can expose `a0` as a port for
// synthesis without poking into the regfile internals.
module RegFile_PORT_X10(
  input         clk,
  input         wen,
  input  [3:0]  waddr,
  input  [31:0] wdata,
  output [31:0] x10
);
  reg [31:0] x10_r;
  always @(posedge clk) begin
    if (wen && waddr == 4'd10) x10_r <= wdata;
  end
  assign x10 = x10_r;
endmodule
`endif
