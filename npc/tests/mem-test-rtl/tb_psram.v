// =============================================================================
// B2b: stand-alone testbench for the PSRAM die-level model (psram.v) wired
// to its real controller pair (EF_PSRAM_CTRL_wb -> EF_PSRAM_CTRL). We act
// as an APB master and run a 4 KByte write/read pattern test.
//
// The patched ysyxSoCFull.v under npc/vsrc-soc/ does NOT instantiate
// psram_top_apb (the APB fanout routes the whole 0x80000000-0xbfffffff
// window to the SDRAM channel), so we cannot exercise the model through
// `make sim-soc`. This testbench provides the missing end-to-end check by
// pairing the same die model with the same controller it would meet in a
// fully wired SoC.
// =============================================================================

`timescale 1ns / 1ps

module tb_psram;

  reg clock;
  reg reset;

  // APB master signals.
  reg  [31:0] paddr;
  reg         psel;
  reg         penable;
  reg         pwrite;
  reg  [31:0] pwdata;
  reg  [3:0]  pstrb;
  wire        pready;
  wire [31:0] prdata;
  wire        pslverr;

  // QSPI link.
  wire        sck;
  wire        ce_n;
  wire [3:0]  dio;

  // 100 MHz clock.
  initial clock = 0;
  always  #5 clock = ~clock;

  // Controller wrapped as APB.
  psram_top_apb u_ctrl (
    .clock       (clock),
    .reset       (reset),
    .in_paddr    (paddr),
    .in_psel     (psel),
    .in_penable  (penable),
    .in_pprot    (3'b0),
    .in_pwrite   (pwrite),
    .in_pwdata   (pwdata),
    .in_pstrb    (pstrb),
    .in_pready   (pready),
    .in_prdata   (prdata),
    .in_pslverr  (pslverr),
    .qspi_sck    (sck),
    .qspi_ce_n   (ce_n),
    .qspi_dio    (dio)
  );

  // Die-level model under test.
  psram u_die (
    .sck   (sck),
    .ce_n  (ce_n),
    .dio   (dio)
  );

  // ---------------------------------------------------------------------------
  // APB master tasks.
  // ---------------------------------------------------------------------------
  // APB write. The controller (EF_PSRAM_CTRL_wb) treats every cycle where
  // psel=1 as a potentially fresh request (it only inspects penable to
  // distinguish setup vs access at the very top of its FSM), so we MUST
  // deassert psel within the same cycle that pready goes high -- otherwise
  // it would treat the held-high psel as a back-to-back duplicate request.
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
      // Wait for slave-side pready=1, then drop psel immediately.
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

  // ---------------------------------------------------------------------------
  // Stimulus. 4 KByte write/read sweep using two patterns.
  // ---------------------------------------------------------------------------
  localparam SIZE_BYTES = 32'd4096;
  localparam BASE_ADDR  = 32'h80000000; // word offset only matters bits[23:0]

  integer       i;
  integer       errors;
  reg  [31:0]   exp;
  reg  [31:0]   got;

  function [31:0] pat1(input [31:0] off);
    pat1 = off ^ 32'hdeadbeef;
  endfunction

  function [31:0] pat2(input [31:0] off);
    pat2 = {off[15:0], ~off[15:0]};
  endfunction

  initial begin
`ifdef DUMP
    $dumpfile("psram.vcd");
    $dumpvars(0, tb_psram);
`endif
    // Init.
    paddr   = 0;
    psel    = 0;
    penable = 0;
    pwrite  = 0;
    pwdata  = 0;
    pstrb   = 0;
    reset   = 1;
    errors  = 0;
    repeat (10) @(posedge clock);
    #1; reset = 0;
    @(posedge clock);

    $display("== PSRAM mem-test: starting (size=%0d bytes) ==", SIZE_BYTES);

    // Pass 1: pat1 write + read.
    for (i = 0; i < SIZE_BYTES; i = i + 4) begin
      apb_write32(BASE_ADDR + i, pat1(i));
    end
    for (i = 0; i < SIZE_BYTES; i = i + 4) begin
      exp = pat1(i);
      apb_read32(BASE_ADDR + i, got);
      if (got !== exp) begin
        $display("FAIL pat1: addr=%08h got=%08h exp=%08h", BASE_ADDR+i, got, exp);
        errors = errors + 1;
        if (errors > 8) begin $display("aborting after 8 mismatches"); $finish; end
      end
    end
    $display("-- pat1 pass --");

    // Pass 2: pat2 overwrite + read.
    for (i = 0; i < SIZE_BYTES; i = i + 4) begin
      apb_write32(BASE_ADDR + i, pat2(i));
    end
    for (i = 0; i < SIZE_BYTES; i = i + 4) begin
      exp = pat2(i);
      apb_read32(BASE_ADDR + i, got);
      if (got !== exp) begin
        $display("FAIL pat2: addr=%08h got=%08h exp=%08h", BASE_ADDR+i, got, exp);
        errors = errors + 1;
        if (errors > 8) begin $display("aborting after 8 mismatches"); $finish; end
      end
    end
    $display("-- pat2 pass --");

    if (errors == 0) $display("PSRAM mem-test PASS");
    else             $display("PSRAM mem-test FAIL (errors=%0d)", errors);

    $finish;
  end

  // Watchdog -- a stuck APB handshake should never block forever.
  initial begin
    #20_000_000;
    $display("PSRAM mem-test FAIL: watchdog timeout");
    $finish;
  end

endmodule
