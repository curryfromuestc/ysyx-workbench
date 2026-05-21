// LSU: memory access via DPI-C (or pmem bus when SYNTHESIS).
//
// addr = alu_result. Word-aligned 32-bit read; byte/half extracted + sign/zero
// extended per funct3:
//   000 lb  (signed byte)
//   001 lh  (signed half)
//   010 lw  (word)
//   100 lbu (unsigned byte)
//   101 lhu (unsigned half)
// Stores:
//   000 sb (byte), 001 sh (half), 010 sw (word).
// Writes use wmask (1 bit per byte) of the aligned word.
//
// `SYNTHESIS` swaps the DPI-C call for a flat pmem-shaped port set so yosys
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
  reg [31:0] write_word;
  reg [7:0]  wmask_byte;

  // -- helpers: select byte / half from a 32-bit word ------------------------
  reg [7:0]  byte_sel;
  reg [15:0] half_sel;
  always @(*) begin
    case (byte_off)
      2'b00:   byte_sel = raw_word[ 7: 0];
      2'b01:   byte_sel = raw_word[15: 8];
      2'b10:   byte_sel = raw_word[23:16];
      2'b11:   byte_sel = raw_word[31:24];
      default: byte_sel = 8'h0;
    endcase
    // halfword may straddle words in principle; AM tests use aligned access
    // (addr[0]==0). We handle byte_off = 0 or 2 here; 1/3 we cope with by
    // selecting the upper/lower 16 within the SAME aligned word — only valid
    // when the access is naturally aligned. RV spec says misaligned LH/SH is
    // implementation defined; tests do not stress it.
    case (byte_off[1])
      1'b0:    half_sel = raw_word[15: 0];
      1'b1:    half_sel = raw_word[31:16];
      default: half_sel = 16'h0;
    endcase
  end

  always @(*) begin
    raw_word   = 32'h0;
    ld         = 32'h0;
    write_word = 32'h0;
    wmask_byte = 8'h00;

    if (mem_re || mem_we) begin
`ifdef SYNTHESIS
      raw_word = pmem_rdata;
`else
      raw_word = pmem_read(aligned);
`endif
    end

    // ---- load decode --------------------------------------------------------
    if (mem_re) begin
      case (funct3)
        3'b000: ld = {{24{byte_sel[7]}}, byte_sel};       // lb  (signed)
        3'b001: ld = {{16{half_sel[15]}}, half_sel};      // lh  (signed)
        3'b010: ld = raw_word;                             // lw
        3'b100: ld = {24'h0, byte_sel};                    // lbu
        3'b101: ld = {16'h0, half_sel};                    // lhu
        default: ld = raw_word;
      endcase
    end

    // ---- store decode (build aligned word + per-byte mask) -----------------
    if (mem_we) begin
      case (funct3)
        3'b010: begin                                      // sw
          write_word = store_data;
          wmask_byte = 8'b0000_1111;
        end
        3'b001: begin                                      // sh
          case (byte_off[1])
            1'b0: begin
              write_word = {16'h0, store_data[15:0]};
              wmask_byte = 8'b0000_0011;
            end
            1'b1: begin
              write_word = {store_data[15:0], 16'h0};
              wmask_byte = 8'b0000_1100;
            end
            default: begin
              write_word = store_data;
              wmask_byte = 8'b0000_1111;
            end
          endcase
        end
        3'b000: begin                                      // sb
          case (byte_off)
            2'b00: begin write_word = {24'h0, store_data[7:0]};                wmask_byte = 8'b0000_0001; end
            2'b01: begin write_word = {16'h0, store_data[7:0], 8'h0};          wmask_byte = 8'b0000_0010; end
            2'b10: begin write_word = {8'h0,  store_data[7:0], 16'h0};         wmask_byte = 8'b0000_0100; end
            2'b11: begin write_word = {       store_data[7:0], 24'h0};         wmask_byte = 8'b0000_1000; end
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
