// IDU: decode full RV32E + Zicsr instruction set.
//
// alu_op (4 bits):
//   0000 add  (rs1 + op2)
//   0001 sub  (rs1 - op2)
//   0010 sll  (rs1 << op2[4:0])
//   0011 slt  (signed rs1 < op2)
//   0100 sltu (unsigned rs1 < op2)
//   0101 xor  (rs1 ^ op2)
//   0110 srl  (rs1 >> op2[4:0] logical)
//   0111 sra  (rs1 >>> op2[4:0] arithmetic)
//   1000 or   (rs1 | op2)
//   1001 and  (rs1 & op2)
//   1010 lui  (passthrough op2 = imm_u)
//   1011 auipc (pc + imm_u; computed in EXU with pc input)
//   1100 jalr  ((rs1 + imm) & ~1)
// wb_sel:
//   2'b00 alu_result
//   2'b01 load_data
//   2'b10 pc+4   (jal / jalr return address)
//   2'b11 csr_rdata
// Branch is decoded into is_branch + branch_op[2:0] (equals funct3 in BRANCH).
// is_jal / is_jalr drive PC redirection in WBU; jal/jalr also reg-write pc+4.
//
// RV32E uses 4-bit register encodings; rd/rs1/rs2 are taken from inst[10:7]/
// inst[18:15]/inst[23:20] (low 4 bits of the 5-bit ISA fields). The toolchain
// guarantees the top bit of every register operand is 0.
module IDU(
  input  [31:0] inst,
  input  [31:0] pc,             // pc passed in for auipc (and jal/jalr return addr)
  output [3:0]  rs1,
  output [3:0]  rs2,
  output [3:0]  rd,
  output [31:0] imm,
  output        alu_src,        // 0 = reg, 1 = imm
  output [3:0]  alu_op,
  output        alu_use_pc,     // 1 -> op1 is pc (used by auipc)
  output        mem_re,
  output        mem_we,
  output [2:0]  funct3,
  output [1:0]  wb_sel,
  output        reg_wen,
  output        is_jal,
  output        is_jalr,
  output        is_branch,
  output [2:0]  branch_op,      // == funct3 of BRANCH opcode
  output        is_ebreak,
  output        is_mret,
  output        is_ecall,
  // CSR side
  output        is_csr,
  output [11:0] csr_addr,
  output [4:0]  csr_uimm,
  output        csr_use_uimm,
  output        csr_re,
  output        csr_we,
  output [1:0]  csr_op
);
  wire [6:0] opcode = inst[6:0];
  wire [2:0] f3     = inst[14:12];
  wire [6:0] f7     = inst[31:25];

  // RV32E: 4-bit reg fields; toolchain guarantees high bit is 0.
  assign rs1 = inst[18:15];
  assign rs2 = inst[23:20];
  assign rd  = inst[10:7];
  assign funct3 = f3;

  // ---- immediate decode (all 5 forms) --------------------------------------
  wire [31:0] imm_i = {{20{inst[31]}}, inst[31:20]};
  wire [31:0] imm_s = {{20{inst[31]}}, inst[31:25], inst[11:7]};
  wire [31:0] imm_b = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
  wire [31:0] imm_u = {inst[31:12], 12'b0};
  wire [31:0] imm_j = {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};

  // ---- opcode classes -------------------------------------------------------
  wire is_op_imm   = (opcode == 7'b0010011); // I-type ALU
  wire is_op       = (opcode == 7'b0110011); // R-type ALU
  wire is_lui_op   = (opcode == 7'b0110111);
  wire is_auipc_op = (opcode == 7'b0010111);
  wire is_load_op  = (opcode == 7'b0000011);
  wire is_store_op = (opcode == 7'b0100011);
  wire is_branch_op= (opcode == 7'b1100011);
  wire is_jal_op   = (opcode == 7'b1101111);
  wire is_jalr_op  = (opcode == 7'b1100111) && (f3 == 3'b000);
  wire is_system   = (opcode == 7'b1110011);
  wire is_misc_mem = (opcode == 7'b0001111); // fence / fence.i -> NOP

  // ---- Zicsr decode --------------------------------------------------------
  wire is_csrrw    = is_system && (f3 == 3'b001);
  wire is_csrrs    = is_system && (f3 == 3'b010);
  wire is_csrrc    = is_system && (f3 == 3'b011);
  wire is_csrrwi   = is_system && (f3 == 3'b101);
  wire is_csrrsi   = is_system && (f3 == 3'b110);
  wire is_csrrci   = is_system && (f3 == 3'b111);
  wire is_csr_w    = is_csrrwi | is_csrrsi | is_csrrci;
  wire is_csr_r    = is_csrrw  | is_csrrs  | is_csrrc;
  assign is_csr    = is_csr_r | is_csr_w;

  assign is_ebreak = (inst == 32'h00100073);
  assign is_mret   = (inst == 32'h30200073);
  // ecall encoding: imm=0, rs1=0, funct3=000, rd=0, opcode=SYSTEM. Match the
  // exact bit pattern so we don't confuse it with ebreak (imm=1) or any other
  // SYSTEM-class instruction (mret, csr*).
  assign is_ecall  = (inst == 32'h00000073);

  assign csr_addr     = inst[31:20];
  assign csr_uimm     = inst[19:15];
  assign csr_use_uimm = is_csr_w;
  assign csr_re       = is_csr;
  wire rs1_zero  = (inst[18:15] == 4'd0);
  wire uimm_zero = (inst[19:15] == 5'd0);
  assign csr_we =
       (is_csrrw)
     | (is_csrrwi)
     | ((is_csrrs | is_csrrc)   & ~rs1_zero)
     | ((is_csrrsi | is_csrrci) & ~uimm_zero);
  assign csr_op =
      (is_csrrs | is_csrrsi) ? 2'b01 :
      (is_csrrc | is_csrrci) ? 2'b10 :
                               2'b00;

  // ---- jump / branch --------------------------------------------------------
  assign is_jal    = is_jal_op;
  assign is_jalr   = is_jalr_op;
  assign is_branch = is_branch_op;
  assign branch_op = f3;

  // ---- memory --------------------------------------------------------------
  assign mem_re = is_load_op;
  assign mem_we = is_store_op;

  // ---- immediate mux -------------------------------------------------------
  assign imm =
      is_jal_op    ? imm_j :
      is_branch_op ? imm_b :
      is_store_op  ? imm_s :
      (is_lui_op | is_auipc_op) ? imm_u :
                                  imm_i; // OP_IMM / load / jalr / system fallback

  // ---- ALU source select --------------------------------------------------
  // ALU op1 is rs1 by default; auipc uses pc instead.
  // ALU op2 is rs2 only for R-type (OP) and BRANCH (sub for compare).
  assign alu_use_pc = is_auipc_op;
  // OP (R-type) reads rs2 from register file; BRANCH compares rs1 vs rs2 (alu does rs1-rs2).
  // Everything else uses imm.
  assign alu_src = (is_op | is_branch_op) ? 1'b0 : 1'b1;

  // ---- ALU op decode --------------------------------------------------------
  // R-type / I-type ALU funct3-funct7 table.
  // OP_IMM (I-type) ignores funct7 EXCEPT slli/srli/srai where funct7 selects sra vs srl.
  // Encoding chosen so it composes nicely below.
  reg [3:0] alu_op_r;
  always @(*) begin
    alu_op_r = 4'b0000; // default add
    if (is_op) begin
      // R-type: funct3 + funct7 fully decode.
      case (f3)
        3'b000: alu_op_r = (f7 == 7'b0100000) ? 4'b0001 : 4'b0000; // sub : add
        3'b001: alu_op_r = 4'b0010; // sll
        3'b010: alu_op_r = 4'b0011; // slt
        3'b011: alu_op_r = 4'b0100; // sltu
        3'b100: alu_op_r = 4'b0101; // xor
        3'b101: alu_op_r = (f7 == 7'b0100000) ? 4'b0111 : 4'b0110; // sra : srl
        3'b110: alu_op_r = 4'b1000; // or
        3'b111: alu_op_r = 4'b1001; // and
        default: alu_op_r = 4'b0000;
      endcase
    end else if (is_op_imm) begin
      case (f3)
        3'b000: alu_op_r = 4'b0000; // addi
        3'b001: alu_op_r = 4'b0010; // slli
        3'b010: alu_op_r = 4'b0011; // slti
        3'b011: alu_op_r = 4'b0100; // sltiu
        3'b100: alu_op_r = 4'b0101; // xori
        3'b101: alu_op_r = (f7 == 7'b0100000) ? 4'b0111 : 4'b0110; // srai : srli
        3'b110: alu_op_r = 4'b1000; // ori
        3'b111: alu_op_r = 4'b1001; // andi
        default: alu_op_r = 4'b0000;
      endcase
    end else if (is_lui_op) begin
      alu_op_r = 4'b1010; // lui passthrough
    end else if (is_auipc_op) begin
      alu_op_r = 4'b1011; // auipc -> pc + imm
    end else if (is_jalr_op) begin
      alu_op_r = 4'b1100; // jalr addr = (rs1+imm) & ~1
    end else if (is_branch_op) begin
      alu_op_r = 4'b0001; // sub for compare; condition computed separately
    end else if (is_load_op | is_store_op) begin
      alu_op_r = 4'b0000; // base + offset
    end else begin
      alu_op_r = 4'b0000;
    end
  end
  assign alu_op = alu_op_r;

  // ---- writeback select ----------------------------------------------------
  assign wb_sel =
      is_csr                       ? 2'b11 :
      is_load_op                   ? 2'b01 :
      (is_jal_op | is_jalr_op)     ? 2'b10 :
                                     2'b00;

  // ---- writeback enable ----------------------------------------------------
  // Any instruction that has an architectural destination. RV spec quirks:
  // - x0 in regfile is silently ignored (RegFile checks waddr).
  // - ebreak / mret / ecall / fence / store / branch / system-no-csr do not write.
  wire wb_arith   = is_op | is_op_imm | is_lui_op | is_auipc_op;
  wire wb_load    = is_load_op;
  wire wb_jump    = is_jal_op | is_jalr_op;
  wire wb_csr     = is_csr;
  assign reg_wen = (wb_arith | wb_load | wb_jump | wb_csr)
                   & ~is_ebreak & ~is_mret & ~is_ecall;
endmodule
