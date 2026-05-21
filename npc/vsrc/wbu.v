// WBU: writeback mux + PC next-state.
// PC reset = 0x80000000.
module WBU(
  input  [31:0] alu_result,
  input  [31:0] load_data,
  input  [1:0]  wb_sel,
  input         is_jalr,
  input  [31:0] pc,
  output [31:0] pc_next,
  output [31:0] wb_data
);
  assign wb_data =
    (wb_sel == 2'b01) ? load_data :
    (wb_sel == 2'b10) ? (pc + 32'd4) :
                        alu_result;

  assign pc_next = is_jalr ? alu_result : (pc + 32'd4);
endmodule
