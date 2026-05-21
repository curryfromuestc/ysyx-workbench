// =============================================================================
// B2c: extended PSRAM mem-test (1 MiB instead of the 4 KiB B2b smoke test).
//
// Identical RTL setup to tb_psram.v -- we just bump SIZE_BYTES to exercise
// the die-level model across a much larger address span. With the QSPI
// Quad-IO timing (8 SCK cmd + 24 SCK addr + 6 SCK dummy + 8 SCK/byte) each
// 32-bit access costs ~50 SCK = 100 clk cycles, so 256 K word accesses *
// 2 passes = ~100 M clk cycles. At 10 ns/cycle that is ~1 s simulated;
// in wall time verilator chews through it in well under a minute.
//
// Patterns are the same as the B2b test so a regression diff is trivial.
// =============================================================================

`timescale 1ns / 1ps

module tb_psram_1m;

  reg clock;
  reg reset;

  reg  [31:0] paddr;
  reg         psel;
  reg         penable;
  reg         pwrite;
  reg  [31:0] pwdata;
  reg  [3:0]  pstrb;
  wire        pready;
  wire [31:0] prdata;
  wire        pslverr;

  wire        sck;
  wire        ce_n;
  wire [3:0]  dio;

  initial clock = 0;
  always  #5 clock = ~clock;  // 100 MHz

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

  psram u_die (
    .sck   (sck),
    .ce_n  (ce_n),
    .dio   (dio)
  );

  `include "apb_master.vh"

  // -------------------------------------------------------------------------
  // 1 MiB stress test. Two patterns, write-then-read each.
  // -------------------------------------------------------------------------
  localparam SIZE_BYTES = 32'h00100000;        // 1 MiB
  localparam BASE_ADDR  = 32'h80000000;        // psram die addresses bits[23:0]

  integer     i;
  integer     errors;
  integer     report_step;
  reg [31:0]  exp;
  reg [31:0]  got;

  function [31:0] pat1(input [31:0] off);
    pat1 = off ^ 32'hdeadbeef;
  endfunction

  // Different from B2b's pat2 to also catch byte-strobe collapse. Same
  // upper-half/inverted-lower-half property but with a different mask.
  function [31:0] pat2(input [31:0] off);
    pat2 = {off[15:0], ~off[15:0]};
  endfunction

  initial begin
`ifdef DUMP
    $dumpfile("psram_1m.vcd");
    $dumpvars(0, tb_psram_1m);
`endif
    paddr   = 0;
    psel    = 0;
    penable = 0;
    pwrite  = 0;
    pwdata  = 0;
    pstrb   = 0;
    reset   = 1;
    errors  = 0;
    // Report every 64 KiB so a stuck FSM is obvious without dumping VCD.
    report_step = 32'h00010000;

    repeat (10) @(posedge clock);
    #1; reset = 0;
    @(posedge clock);

    $display("== PSRAM mem-test 1M: starting (size=%0d bytes = %0d KiB) ==",
             SIZE_BYTES, SIZE_BYTES / 1024);

    // --- pat1 write -----
    for (i = 0; i < SIZE_BYTES; i = i + 4) begin
      apb_write32(BASE_ADDR + i, pat1(i));
      if ((i % report_step) == 0 && i != 0)
        $display("   pat1 wr: %0d KiB done", i / 1024);
    end
    $display("   pat1 wr: %0d KiB done", SIZE_BYTES / 1024);

    // --- pat1 read -----
    for (i = 0; i < SIZE_BYTES; i = i + 4) begin
      exp = pat1(i);
      apb_read32(BASE_ADDR + i, got);
      if (got !== exp) begin
        $display("FAIL pat1: addr=%08h got=%08h exp=%08h", BASE_ADDR+i, got, exp);
        errors = errors + 1;
        if (errors > 8) begin $display("aborting after 8 mismatches"); $finish; end
      end
      if ((i % report_step) == 0 && i != 0)
        $display("   pat1 rd: %0d KiB done", i / 1024);
    end
    $display("-- pat1 pass --");

    // --- pat2 write -----
    for (i = 0; i < SIZE_BYTES; i = i + 4) begin
      apb_write32(BASE_ADDR + i, pat2(i));
      if ((i % report_step) == 0 && i != 0)
        $display("   pat2 wr: %0d KiB done", i / 1024);
    end

    // --- pat2 read -----
    for (i = 0; i < SIZE_BYTES; i = i + 4) begin
      exp = pat2(i);
      apb_read32(BASE_ADDR + i, got);
      if (got !== exp) begin
        $display("FAIL pat2: addr=%08h got=%08h exp=%08h", BASE_ADDR+i, got, exp);
        errors = errors + 1;
        if (errors > 8) begin $display("aborting after 8 mismatches"); $finish; end
      end
      if ((i % report_step) == 0 && i != 0)
        $display("   pat2 rd: %0d KiB done", i / 1024);
    end
    $display("-- pat2 pass --");

    if (errors == 0) $display("PSRAM mem-test 1M PASS");
    else             $display("PSRAM mem-test 1M FAIL (errors=%0d)", errors);

    $finish;
  end

  // Generous watchdog: a 1 MiB sweep with QSPI overhead can take a few
  // hundred million simulated clock cycles. We cap at ~2 s simulated (= 2 s
  // * 1e9 ps = 2_000_000_000 ps). Bump if the wall-clock takes longer.
  initial begin
    #2_000_000_000;
    $display("PSRAM mem-test 1M FAIL: watchdog timeout");
    $finish;
  end

endmodule
