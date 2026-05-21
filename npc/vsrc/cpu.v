// Top-level single-cycle minirv CPU.
// Resets PC to 0x80000000 (minirv-npc convention).
// On posedge clk after reset:
//   - inst is fetched combinationally via IFU (DPI-C pmem_read)
//   - regfile/lsu update on this clock edge
//   - ebreak triggers DPI-C npc_trap (called from top via always @(posedge clk))
module cpu(
  input clk,
  input rst
);
  reg  [31:0] pc;
  wire [31:0] pc_next;
  wire [31:0] inst;

  wire [3:0]  rs1, rs2, rd;
  wire [31:0] imm;
  wire        alu_src;
  wire [1:0]  alu_op;
  wire        mem_re, mem_we;
  wire [2:0]  funct3;
  wire [1:0]  wb_sel;
  wire        reg_wen;
  wire        is_jalr;
  wire        is_ebreak;

  wire [31:0] rs1_val, rs2_val;
  wire [31:0] alu_result;
  wire [31:0] load_data;
  wire [31:0] wb_data;

  // PC register
  always @(posedge clk) begin
    if (rst) pc <= 32'h8000_0000;
    else     pc <= pc_next;
  end

  IFU u_ifu (.pc(pc), .inst(inst));

  IDU u_idu (
    .inst(inst),
    .rs1(rs1), .rs2(rs2), .rd(rd),
    .imm(imm), .alu_src(alu_src), .alu_op(alu_op),
    .mem_re(mem_re), .mem_we(mem_we), .funct3(funct3),
    .wb_sel(wb_sel), .reg_wen(reg_wen),
    .is_jalr(is_jalr), .is_ebreak(is_ebreak)
  );

  RegFile #(.ADDR_WIDTH(4), .DATA_WIDTH(32)) u_rf (
    .clk(clk),
    .raddr1(rs1), .rdata1(rs1_val),
    .raddr2(rs2), .rdata2(rs2_val),
    .wdata(wb_data), .waddr(rd), .wen(reg_wen & ~rst & ~is_ebreak)
  );

  EXU u_exu (
    .rs1_val(rs1_val), .rs2_val(rs2_val), .imm(imm),
    .alu_src(alu_src), .alu_op(alu_op),
    .alu_result(alu_result)
  );

  LSU u_lsu (
    .addr(alu_result), .store_data(rs2_val),
    .mem_re(mem_re & ~rst), .mem_we(mem_we & ~rst),
    .funct3(funct3),
    .load_data(load_data)
  );

  WBU u_wbu (
    .alu_result(alu_result), .load_data(load_data),
    .wb_sel(wb_sel), .is_jalr(is_jalr),
    .pc(pc),
    .pc_next(pc_next), .wb_data(wb_data)
  );

  // ebreak hookup: synchronously notify simulator. a0 = x10 (regfile index 10).
  import "DPI-C" function void npc_trap(input int code);
  always @(posedge clk) begin
    if (!rst && is_ebreak) begin
      npc_trap(u_rf.rf[10]);
    end
  end
endmodule
