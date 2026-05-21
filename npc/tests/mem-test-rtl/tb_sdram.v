// =============================================================================
// B2b: stand-alone testbench for the SDRAM die-level model (sdram.v) wired
// to its real controller pair (sdram_top_apb -> sdram_axi_core). We act as
// an APB master and run a 4 KByte write/read pattern sweep.
//
// As with PSRAM, the patched ysyxSoCFull.v under npc/vsrc-soc/ does not use
// our sdram.v file (it instantiates the Chisel-emitted sdramChisel model),
// so we cannot exercise it through `make sim-soc`. This testbench provides
// the missing end-to-end check: same controller (sdram_axi_core) talking to
// our die-level behavioural model.
// =============================================================================

`timescale 1ns / 1ps

module tb_sdram;

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

  // SDRAM die-controller link.
  wire        sdram_clk;
  wire        sdram_cke;
  wire        sdram_cs;
  wire        sdram_ras;
  wire        sdram_cas;
  wire        sdram_we;
  wire [12:0] sdram_a;
  wire [1:0]  sdram_ba;
  wire [1:0]  sdram_dqm;
  wire [15:0] sdram_dq;

  // 100 MHz clock. (sdram_axi_core defaults to SDRAM_MHZ=100, matching the
  // sdram_top_apb override.)
  initial clock = 0;
  always  #5 clock = ~clock;

  // Controller wrapped as APB.
  sdram_top_apb u_ctrl (
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
    .sdram_clk   (sdram_clk),
    .sdram_cke   (sdram_cke),
    .sdram_cs    (sdram_cs),
    .sdram_ras   (sdram_ras),
    .sdram_cas   (sdram_cas),
    .sdram_we    (sdram_we),
    .sdram_a     (sdram_a),
    .sdram_ba    (sdram_ba),
    .sdram_dqm   (sdram_dqm),
    .sdram_dq    (sdram_dq)
  );

  // Die under test.
  sdram u_die (
    .clk  (sdram_clk),
    .cke  (sdram_cke),
    .cs   (sdram_cs),
    .ras  (sdram_ras),
    .cas  (sdram_cas),
    .we   (sdram_we),
    .a    (sdram_a),
    .ba   (sdram_ba),
    .dqm  (sdram_dqm),
    .dq   (sdram_dq)
  );

  `include "apb_master.vh"

  // ---------------------------------------------------------------------------
  // Stimulus. 4 KByte write/read sweep using two patterns.
  // ---------------------------------------------------------------------------
  localparam SIZE_BYTES = 32'd4096;
  localparam BASE_ADDR  = 32'ha0000000;   // top 8 bits ignored by sdram_axi_core (latches 24 bits)

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
    // Init.
    paddr   = 0;
    psel    = 0;
    penable = 0;
    pwrite  = 0;
    pwdata  = 0;
    pstrb   = 0;
    reset   = 1;
    errors  = 0;

    // sdram_axi_core needs a longish power-up: the START_DELAY config is
    // 100uS at 100MHz, but the controller uses a much smaller START_DELAY
    // when run-time SDRAM_MHZ matches the verilator timescale; we still
    // give it plenty of slack -- 20000 cycles of NOP traffic from us.
    repeat (10) @(posedge clock);
    #1; reset = 0;
    // Wait for the controller's INIT FSM to finish (PRECHARGE, 2x REFRESH,
    // LOAD_MODE) before issuing real traffic.
    repeat (20000) @(posedge clock);

    $display("== SDRAM mem-test: starting (size=%0d bytes) ==", SIZE_BYTES);

    // Pass 1.
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

    if (errors == 0) $display("SDRAM mem-test PASS");
    else             $display("SDRAM mem-test FAIL (errors=%0d)", errors);

    $finish;
  end

  // Watchdog.
  initial begin
    #80_000_000;
    $display("SDRAM mem-test FAIL: watchdog timeout");
    $finish;
  end

endmodule
