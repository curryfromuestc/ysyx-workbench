// =============================================================================
// cpu_pipeline  --  B5b D4/D5 harness top for the 5-stage pipeline NPC.
// =============================================================================
// Wraps ysyx_22040000.v (the B5a pipeline SoC top) and bridges its SimpleBus
// IFU/LSU master interfaces to the legacy DPI pmem (single-cycle response),
// so the same `make sim` harness used by D4/D5/difftest can drive the
// pipelined NPC without going through the full ysyxSoCFull AXI fabric.
//
// Why this exists:
//   B5a's pipeline implementation lives in ysyx_22040000.v with SimpleBus
//   master ports. D4/D5's cpu.v top exposes flat pmem_read/pmem_write DPI
//   calls. To run `make sim PIPELINE=1` and difftest against the pipeline,
//   we need a top module that:
//     1. Speaks DPI pmem on one side (so main.cpp doesn't change protocol)
//     2. Instantiates ysyx_22040000 on the other side
//     3. Has reset vector at 0x80000000 (not SoC 0x30000000)
//
// Bridge protocol (single-cycle response):
//   io_ifu_respValid = io_ifu_reqValid      (same-cycle return)
//   io_ifu_rdata     = pmem_read(io_ifu_addr) (combinational)
//   io_lsu_respValid = io_lsu_reqValid      (same-cycle return)
//   io_lsu_rdata     = pmem_read(io_lsu_addr) if !wen
//   pmem_write(io_lsu_addr, io_lsu_wdata, io_lsu_wmask) on posedge clk
//
// This means icache fill (8-beat) completes in 8 cycles (one beat per cycle)
// because every bus_req_valid pulse gets bus_resp_valid back in the same
// cycle. SoC behaviour is preserved (multi-cycle fill still works) but flat
// mem access is no longer stalled by the AXI fabric.
//
// ebreak detection:
//   ysyx_22040000.v exposes ebreak_park (sticky once WB sees ebreak). We
//   detect rising edge of ebreak_park and fire npc_trap(rf[10]) once. After
//   that ebreak_park stays high to freeze the pipeline.

`ifndef PC_RESET_VEC_PIPELINE
  `define PC_RESET_VEC_PIPELINE 32'h8000_0000
`endif

// Override ysyx_22040000.v's default SoC reset vec before its module is
// elaborated. This file is listed first in the Makefile PIPELINE=1 VSRCS so
// the macro is in scope when ysyx_22040000.v's `ifndef SOC_PC_RESET_VEC` is
// evaluated.
`ifndef SOC_PC_RESET_VEC
  `define SOC_PC_RESET_VEC `PC_RESET_VEC_PIPELINE
`endif

module cpu_pipeline(
  input clk,
  input rst
);

  // --- Bridge wires --------------------------------------------------------
  wire        io_ifu_reqValid;
  wire [31:0] io_ifu_addr;
  wire        io_ifu_respValid;
  wire [31:0] io_ifu_rdata;

  wire        io_lsu_reqValid;
  wire [31:0] io_lsu_addr;
  wire [1:0]  io_lsu_size;
  wire        io_lsu_wen;
  wire [31:0] io_lsu_wdata;
  wire [3:0]  io_lsu_wmask;
  wire        io_lsu_respValid;
  wire [31:0] io_lsu_rdata;

  // --- DPI memory glue -----------------------------------------------------
  import "DPI-C" function int pmem_read(input int raddr);
  import "DPI-C" function void pmem_write(input int waddr, input int wdata, input byte wmask);
  import "DPI-C" function void npc_trap(input int code);

  // IFU / LSU 数据通路: 单拍 DPI mem 桥. reqValid 拉低时不调 pmem_read,
  // 避免 verilator 每次 eval 都跨 SV/C 边界 (clk=0/clk=1 两拍 = 4 次 DPI
  // per cycle), 在 reqValid=0 那些拍纯属浪费.
  assign io_ifu_respValid = io_ifu_reqValid & ~rst;
  assign io_ifu_rdata     = io_ifu_reqValid
                          ? pmem_read({io_ifu_addr[31:2], 2'b00})
                          : 32'h0;

  assign io_lsu_respValid = io_lsu_reqValid & ~rst;
  assign io_lsu_rdata     = (io_lsu_reqValid & ~io_lsu_wen)
                          ? pmem_read({io_lsu_addr[31:2], 2'b00})
                          : 32'h0;

  always @(posedge clk) begin
    if (!rst && io_lsu_reqValid && io_lsu_wen) begin
      pmem_write({io_lsu_addr[31:2], 2'b00}, io_lsu_wdata,
                 {4'b0, io_lsu_wmask});
    end
  end

  // --- Pipeline NPC core ---------------------------------------------------
  ysyx_22040000 u_cpu_top (
    .clock           (clk),
    .reset           (rst),
    .io_ifu_reqValid (io_ifu_reqValid),
    .io_ifu_addr     (io_ifu_addr),
    .io_ifu_respValid(io_ifu_respValid),
    .io_ifu_rdata    (io_ifu_rdata),
    .io_lsu_reqValid (io_lsu_reqValid),
    .io_lsu_addr     (io_lsu_addr),
    .io_lsu_size     (io_lsu_size),
    .io_lsu_wen      (io_lsu_wen),
    .io_lsu_wdata    (io_lsu_wdata),
    .io_lsu_wmask    (io_lsu_wmask),
    .io_lsu_respValid(io_lsu_respValid),
    .io_lsu_rdata    (io_lsu_rdata),
    .io_fault        (1'b0)
  );

  // --- ebreak detection (sticky ebreak_park edge) --------------------------
  // ebreak_park rises once when WB segment sees an ebreak and stays high.
  // npc_trap must fire exactly once on that rising edge.
  reg ebreak_park_prev;
  always @(posedge clk) begin
    if (rst) ebreak_park_prev <= 1'b0;
    else     ebreak_park_prev <= u_cpu_top.ebreak_park;
  end

  wire ebreak_fire = u_cpu_top.ebreak_park & ~ebreak_park_prev;

  always @(posedge clk) begin
    if (!rst && ebreak_fire) begin
      npc_trap(u_cpu_top.u_rf.rf[10]);
    end
  end

endmodule
