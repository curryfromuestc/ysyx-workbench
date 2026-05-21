// EXU: ALU for minirv. only "add", lui passthrough, jalr address compute.
module EXU(
  input  [31:0] rs1_val,
  input  [31:0] rs2_val,
  input  [31:0] imm,
  input         alu_src,    // 0 reg, 1 imm
  input  [1:0]  alu_op,     // 0 add, 1 lui (pass imm), 2 jalr addr ((rs1+imm)&~1)
  output [31:0] alu_result
);
  wire [31:0] op2 = alu_src ? imm : rs2_val;
  wire [31:0] sum = rs1_val + op2;

  assign alu_result =
    (alu_op == 2'b01) ? imm :
    (alu_op == 2'b10) ? (sum & 32'hFFFF_FFFE) :
                        sum;
endmodule
