// 16-entry GPR file (RV32E). x0 hardwired to 0.
// Async read (three independent read ports), sync write on posedge clk.
//
// Read port 3 (raddr3 / rdata3) is added so the CTE trap path can sample the
// syscall-number register (a5 under ilp32e) without poking into the regfile
// internals via a hierarchical reference. Synthesisable.
module RegFile #(
  parameter ADDR_WIDTH = 4,
  parameter DATA_WIDTH = 32
) (
  input                    clk,
  input  [ADDR_WIDTH-1:0]  raddr1,
  output [DATA_WIDTH-1:0]  rdata1,
  input  [ADDR_WIDTH-1:0]  raddr2,
  output [DATA_WIDTH-1:0]  rdata2,
  input  [ADDR_WIDTH-1:0]  raddr3,
  output [DATA_WIDTH-1:0]  rdata3,
  input  [DATA_WIDTH-1:0]  wdata,
  input  [ADDR_WIDTH-1:0]  waddr,
  input                    wen
);
  reg [DATA_WIDTH-1:0] rf [0:(1<<ADDR_WIDTH)-1];

  integer i;
  initial begin
    for (i = 0; i < (1<<ADDR_WIDTH); i = i + 1) rf[i] = {DATA_WIDTH{1'b0}};
  end

  always @(posedge clk) begin
    if (wen && (waddr != {ADDR_WIDTH{1'b0}})) rf[waddr] <= wdata;
  end

  assign rdata1 = (raddr1 == {ADDR_WIDTH{1'b0}}) ? {DATA_WIDTH{1'b0}} : rf[raddr1];
  assign rdata2 = (raddr2 == {ADDR_WIDTH{1'b0}}) ? {DATA_WIDTH{1'b0}} : rf[raddr2];
  assign rdata3 = (raddr3 == {ADDR_WIDTH{1'b0}}) ? {DATA_WIDTH{1'b0}} : rf[raddr3];
endmodule
