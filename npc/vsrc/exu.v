// EXU: full RV32 ALU + branch evaluation.
//
// alu_op encoding (4 bits, matches IDU):
//   0000 add, 0001 sub, 0010 sll, 0011 slt, 0100 sltu, 0101 xor,
//   0110 srl, 0111 sra, 1000 or, 1001 and,
//   1010 lui  (pass op2 through),
//   1011 auipc(pc + imm; op1 = pc, op2 = imm),
//   1100 jalr ((rs1+imm) & ~1).
//
// branch_op is the funct3 of a BRANCH-opcode instruction:
//   000 beq, 001 bne, 100 blt, 101 bge, 110 bltu, 111 bgeu.
// branch_taken is meaningful only when WBU is told is_branch=1.
module EXU(
  input  [31:0] pc,
  input  [31:0] rs1_val,
  input  [31:0] rs2_val,
  input  [31:0] imm,
  input         alu_src,       // 0 reg (op2 = rs2), 1 imm (op2 = imm)
  input         alu_use_pc,    // 1 -> op1 = pc (auipc), 0 -> op1 = rs1
  input  [3:0]  alu_op,
  input  [2:0]  branch_op,
  output [31:0] alu_result,
  output        branch_taken
);
  wire [31:0] op1 = alu_use_pc ? pc : rs1_val;
  wire [31:0] op2 = alu_src    ? imm : rs2_val;

  // sub used both by SUB and by branch comparisons / slt(u).
  wire [31:0] sum    = op1 + op2;
  wire [31:0] sub    = op1 - op2;
  wire [4:0]  shamt  = op2[4:0];
  wire        signed_lt   = ($signed(op1) <  $signed(op2));
  wire        unsigned_lt = (op1 < op2);

  // Arithmetic right shift: cast op1 to signed.
  wire [31:0] sra_v = $signed(op1) >>> shamt;

  reg [31:0] alu_r;
  always @(*) begin
    case (alu_op)
      4'b0000: alu_r = sum;
      4'b0001: alu_r = sub;
      4'b0010: alu_r = op1 << shamt;
      4'b0011: alu_r = {31'b0, signed_lt};
      4'b0100: alu_r = {31'b0, unsigned_lt};
      4'b0101: alu_r = op1 ^ op2;
      4'b0110: alu_r = op1 >> shamt;
      4'b0111: alu_r = sra_v;
      4'b1000: alu_r = op1 | op2;
      4'b1001: alu_r = op1 & op2;
      4'b1010: alu_r = op2;                       // lui: op2 = imm_u
      4'b1011: alu_r = sum;                       // auipc: op1=pc, op2=imm_u
      4'b1100: alu_r = sum & 32'hFFFF_FFFE;       // jalr: (rs1+imm) & ~1
      default: alu_r = sum;
    endcase
  end
  assign alu_result = alu_r;

  // Branch condition: compare rs1_val vs rs2_val (the original operands,
  // independent of imm path) per RISC-V spec.
  wire eq  = (rs1_val == rs2_val);
  wire lt_s  = ($signed(rs1_val) <  $signed(rs2_val));
  wire lt_u  = (rs1_val < rs2_val);

  reg br_r;
  always @(*) begin
    case (branch_op)
      3'b000: br_r =  eq;     // beq
      3'b001: br_r = ~eq;     // bne
      3'b100: br_r =  lt_s;   // blt
      3'b101: br_r = ~lt_s;   // bge
      3'b110: br_r =  lt_u;   // bltu
      3'b111: br_r = ~lt_u;   // bgeu
      default: br_r = 1'b0;
    endcase
  end
  assign branch_taken = br_r;
endmodule
