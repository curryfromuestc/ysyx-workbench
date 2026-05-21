// =============================================================================
// APB master driver tasks shared between tb_psram.v and tb_sdram.v.
// =============================================================================
// Expects the enclosing testbench module to declare these regs/wires:
//   reg         clock, psel, penable, pwrite;
//   reg  [31:0] paddr, pwdata;
//   reg  [3:0]  pstrb;
//   wire        pready;
//   wire [31:0] prdata;
//
// EF_PSRAM_CTRL_wb (and sdram_axi_core via sdram_top_apb) treat every cycle
// where psel=1 as a potentially fresh request -- they only inspect penable
// at the top of their FSM. The tasks therefore drop psel inside the same
// cycle that pready goes high so the slave does not see a duplicate
// back-to-back trigger.
// =============================================================================

task automatic apb_write32(input [31:0] addr, input [31:0] data);
  begin
    @(posedge clock); #1;
    paddr   = addr;
    pwdata  = data;
    pwrite  = 1'b1;
    pstrb   = 4'b1111;
    psel    = 1'b1;
    penable = 1'b0;
    @(posedge clock); #1;
    penable = 1'b1;
    while (!pready) @(posedge clock);
    #1;
    psel    = 1'b0;
    penable = 1'b0;
    pwrite  = 1'b0;
    pstrb   = 4'b0;
  end
endtask

task automatic apb_read32(input [31:0] addr, output [31:0] data);
  begin
    @(posedge clock); #1;
    paddr   = addr;
    pwrite  = 1'b0;
    pstrb   = 4'b1111;
    psel    = 1'b1;
    penable = 1'b0;
    @(posedge clock); #1;
    penable = 1'b1;
    while (!pready) @(posedge clock);
    data    = prdata;
    #1;
    psel    = 1'b0;
    penable = 1'b0;
    pstrb   = 4'b0;
  end
endtask
