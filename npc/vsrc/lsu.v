// LSU: memory access via DPI-C.
// addr = rs1 + imm (alu_result for loads/stores).
// Word-aligned read returns 32 bits; we shift/mask based on byte offset and funct3.
// Writes use wmask (1 bit per byte).
// `SYNTHESIS` strips the DPI-C calls and exposes raw bus-shaped wires so yosys
// can lint the data path. Simulator builds never define SYNTHESIS.
module LSU(
  input  [31:0] addr,
  input  [31:0] store_data,
  input         mem_re,
  input         mem_we,
  input  [2:0]  funct3,
  output [31:0] load_data
`ifdef SYNTHESIS
  ,
  output [31:0] pmem_raddr,
  input  [31:0] pmem_rdata,
  output        pmem_wen,
  output [31:0] pmem_waddr,
  output [31:0] pmem_wdata,
  output [7:0]  pmem_wmask
`endif
);
`ifndef SYNTHESIS
  import "DPI-C" function int pmem_read(input int raddr);
  import "DPI-C" function void pmem_write(input int waddr, input int wdata, input byte wmask);
`endif

  wire [1:0]  byte_off = addr[1:0];
  wire [31:0] aligned  = {addr[31:2], 2'b00};

  reg [31:0] raw_word;
  reg [31:0] ld;

  // store mux
  reg [31:0] write_word;
  reg [7:0]  wmask_byte;

  always @(*) begin
    raw_word = 32'h0;
    ld       = 32'h0;
    write_word = 32'h0;
    wmask_byte = 8'h00;

    if (mem_re || mem_we) begin
`ifdef SYNTHESIS
      raw_word = pmem_rdata;
`else
      raw_word = pmem_read(aligned);
`endif
    end

    // load decode
    if (mem_re) begin
      case (funct3)
        3'b010: ld = raw_word;                                   // lw
        3'b100: begin                                            // lbu
          case (byte_off)
            2'b00: ld = {24'h0, raw_word[ 7: 0]};
            2'b01: ld = {24'h0, raw_word[15: 8]};
            2'b10: ld = {24'h0, raw_word[23:16]};
            2'b11: ld = {24'h0, raw_word[31:24]};
          endcase
        end
        default: ld = raw_word;
      endcase
    end

    // store: build aligned word + wmask
    if (mem_we) begin
      case (funct3)
        3'b010: begin                                            // sw
          write_word = store_data;
          wmask_byte = 8'b0000_1111;
        end
        3'b000: begin                                            // sb
          case (byte_off)
            2'b00: begin write_word = {24'h0, store_data[7:0]};                 wmask_byte = 8'b0000_0001; end
            2'b01: begin write_word = {16'h0, store_data[7:0], 8'h0};            wmask_byte = 8'b0000_0010; end
            2'b10: begin write_word = {8'h0,  store_data[7:0], 16'h0};           wmask_byte = 8'b0000_0100; end
            2'b11: begin write_word = {       store_data[7:0], 24'h0};           wmask_byte = 8'b0000_1000; end
          endcase
        end
        default: begin
          write_word = store_data;
          wmask_byte = 8'b0000_1111;
        end
      endcase
`ifndef SYNTHESIS
      pmem_write(aligned, write_word, wmask_byte);
`endif
    end
  end

  assign load_data = ld;

`ifdef SYNTHESIS
  assign pmem_raddr = aligned;
  assign pmem_wen   = mem_we;
  assign pmem_waddr = aligned;
  assign pmem_wdata = write_word;
  assign pmem_wmask = wmask_byte;
`endif
endmodule
