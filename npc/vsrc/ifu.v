// IFU: fetch instruction from physical memory via DPI-C.
//
// `SYNTHESIS` strips the DPI-C call and exposes a flat pmem_read-shaped
// interface so yosys can lint the data path without choking on DPI syntax.
// The simulator build never defines SYNTHESIS, so functional behaviour stays
// identical there.
module IFU(
  input  [31:0] pc,
  output [31:0] inst
`ifdef SYNTHESIS
  ,
  output [31:0] pmem_raddr,
  input  [31:0] pmem_rdata
`endif
);
`ifdef SYNTHESIS
  assign pmem_raddr = {pc[31:2], 2'b00};
  assign inst       = pmem_rdata;
`else
  import "DPI-C" function int pmem_read(input int raddr);
  reg [31:0] rdata;
  always @(*) begin
    rdata = pmem_read({pc[31:2], 2'b00});
  end
  assign inst = rdata;
`endif
endmodule
