// WBU: writeback data mux + next-PC selector.
//
// PC redirect priority (highest to lowest):
//   1. is_mret           : pc_next = mepc (from CSR file)
//   2. is_jal            : pc_next = pc + imm_j  (consumed via jal_target)
//   3. is_jalr           : pc_next = alu_result  (already (rs1+imm)&~1)
//   4. is_branch & taken : pc_next = pc + imm_b  (consumed via branch_target)
//   5. default           : pc_next = pc + 4
//
// wb_sel selects the write-back DATA only; PC redirect is independent.
module WBU(
  input  [31:0] alu_result,
  input  [31:0] load_data,
  input  [31:0] csr_rdata,
  input  [1:0]  wb_sel,
  input         is_jal,
  input         is_jalr,
  input         is_branch,
  input         branch_taken,
  input         is_mret,
  input  [31:0] mepc,
  input  [31:0] pc,
  input  [31:0] imm,           // imm_j when is_jal, imm_b when is_branch
  output [31:0] pc_next,
  output [31:0] wb_data
);
  assign wb_data =
    (wb_sel == 2'b01) ? load_data :
    (wb_sel == 2'b10) ? (pc + 32'd4) :       // jal / jalr return address
    (wb_sel == 2'b11) ? csr_rdata :
                        alu_result;

  wire [31:0] pc_plus4   = pc + 32'd4;
  wire [31:0] pc_branch  = pc + imm;          // imm_b
  wire [31:0] pc_jal     = pc + imm;          // imm_j

  assign pc_next =
      is_mret                    ? mepc      :
      is_jal                     ? pc_jal    :
      is_jalr                    ? alu_result :
      (is_branch & branch_taken) ? pc_branch :
                                   pc_plus4;
endmodule
