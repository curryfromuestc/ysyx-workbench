// IDU: decode minirv instruction
// alu_op encoding (we only need add):
//   0 = add (rs1 + rs2_or_imm)
//   1 = lui (imm)
//   2 = jalr (rs1 + imm; pc<-result&~1; rd<-pc+4)
// wb_sel:
//   2'b00 = alu result, 2'b01 = load data, 2'b10 = pc+4
module IDU(
  input  [31:0] inst,
  output [3:0]  rs1,
  output [3:0]  rs2,
  output [3:0]  rd,
  output [31:0] imm,
  output        alu_src,        // 0 = reg, 1 = imm
  output [1:0]  alu_op,         // 0 add, 1 passthrough imm (lui), 2 jalr addr
  output        mem_re,
  output        mem_we,
  output [2:0]  funct3,
  output [1:0]  wb_sel,
  output        reg_wen,
  output        is_jalr,
  output        is_ebreak
);
  wire [6:0] opcode = inst[6:0];
  wire [2:0] f3     = inst[14:12];
  wire [6:0] f7     = inst[31:25];

  // RV32E uses 4-bit register encodings; high bit must be 0 per spec.
  assign rs1 = inst[18:15];
  assign rs2 = inst[23:20];
  assign rd  = inst[10:7];
  assign funct3 = f3;

  wire [31:0] imm_i = {{20{inst[31]}}, inst[31:20]};
  wire [31:0] imm_u = {inst[31:12], 12'b0};
  wire [31:0] imm_s = {{20{inst[31]}}, inst[31:25], inst[11:7]};

  wire is_addi  = (opcode == 7'b0010011) && (f3 == 3'b000);
  wire is_add   = (opcode == 7'b0110011) && (f3 == 3'b000) && (f7 == 7'b0000000);
  wire is_lui   = (opcode == 7'b0110111);
  wire is_lw    = (opcode == 7'b0000011) && (f3 == 3'b010);
  wire is_lbu   = (opcode == 7'b0000011) && (f3 == 3'b100);
  wire is_sw    = (opcode == 7'b0100011) && (f3 == 3'b010);
  wire is_sb    = (opcode == 7'b0100011) && (f3 == 3'b000);
  wire is_load  = is_lw | is_lbu;
  wire is_store = is_sw | is_sb;

  assign is_jalr   = (opcode == 7'b1100111) && (f3 == 3'b000);
  assign is_ebreak = (inst == 32'h00100073);

  assign mem_re = is_load;
  assign mem_we = is_store;

  assign imm = is_lui   ? imm_u :
               is_store ? imm_s :
                          imm_i;

  // add uses both register operands; everything else uses imm as op2.
  assign alu_src = is_add ? 1'b0 : 1'b1;

  assign alu_op = is_lui  ? 2'b01 :
                  is_jalr ? 2'b10 :
                            2'b00;

  assign wb_sel = is_load ? 2'b01 :
                  is_jalr ? 2'b10 :
                            2'b00;

  assign reg_wen = (is_add | is_addi | is_lui | is_load | is_jalr) & ~is_ebreak;
endmodule
