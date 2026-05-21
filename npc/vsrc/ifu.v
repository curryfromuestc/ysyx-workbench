// IFU: fetch instruction from physical memory via DPI-C.
module IFU(
  input  [31:0] pc,
  output [31:0] inst
);
  import "DPI-C" function int pmem_read(input int raddr);
  reg [31:0] rdata;
  always @(*) begin
    rdata = pmem_read({pc[31:2], 2'b00});
  end
  assign inst = rdata;
endmodule
