// CSR file for NPC.
//
// Supports the 6 Zicsr instructions: csrrw / csrrs / csrrc / csrrwi / csrrsi / csrrci.
//
// Implemented CSRs:
//   0x300 mstatus     - plain R/W (no side effects; not used for exceptions yet)
//   0x305 mtvec       - plain R/W (used by C5a CTE; harmless until then)
//   0x341 mepc        - plain R/W; exposed as `mepc_out` so WBU/mret can redirect PC
//   0x342 mcause      - plain R/W
//   0xb00 mcycle      - free-running cycle counter (low 32 bits)
//   0xf11 mvendorid   - student ID, fixed 32'h2204_0000
//   0xf13 mimpid      - alias for student id (also fixed 0x22040000)
//
// Notes on mret:
//   This file does NOT auto-trigger anything on mret; the wrapping CPU detects
//   is_mret from the IDU and feeds mepc into the WBU's pc_next mux. mret in
//   ilp32e RV32E does not push/pop mstatus.MPIE/MIE here (no interrupts yet).
//
// Behaviour summary (unchanged from D6):
//   - mcycle increments every clock cycle while !reset.
//   - On a CSR instruction with csr_we=1, the destination CSR is overwritten with
//     csr_wdata (after the read happens, so rd already sees the OLD value).

module CSR(
  input         clk,
  input         rst,
  // Read port (combinational)
  input  [11:0] csr_raddr,
  output [31:0] csr_rdata,
  // Write port (clocked)
  input         csr_we,
  input  [11:0] csr_waddr,
  input  [31:0] csr_wdata,
  // mepc continuously exposed for WBU's mret redirect path.
  output [31:0] mepc_out
);
  reg [31:0] mcycle;
  reg [31:0] mstatus;
  reg [31:0] mtvec;
  reg [31:0] mepc;
  reg [31:0] mcause;

  localparam [31:0] STUDENT_ID = 32'h2204_0000;

  reg [31:0] rdata_r;
  always @(*) begin
    case (csr_raddr)
      12'h300: rdata_r = mstatus;
      12'h305: rdata_r = mtvec;
      12'h341: rdata_r = mepc;
      12'h342: rdata_r = mcause;
      12'hb00: rdata_r = mcycle;
      12'hf11: rdata_r = STUDENT_ID; // mvendorid
      12'hf13: rdata_r = STUDENT_ID; // mimpid
      default: rdata_r = 32'h0;
    endcase
  end
  assign csr_rdata = rdata_r;
  assign mepc_out  = mepc;

  always @(posedge clk) begin
    if (rst) begin
      mcycle  <= 32'h0;
      mstatus <= 32'h0;
      mtvec   <= 32'h0;
      mepc    <= 32'h0;
      mcause  <= 32'h0;
    end else begin
      // mcycle: write takes precedence over auto-increment.
      if (csr_we && csr_waddr == 12'hb00) begin
        mcycle <= csr_wdata;
      end else begin
        mcycle <= mcycle + 32'd1;
      end
      // Plain R/W CSRs.
      if (csr_we) begin
        case (csr_waddr)
          12'h300: mstatus <= csr_wdata;
          12'h305: mtvec   <= csr_wdata;
          12'h341: mepc    <= csr_wdata;
          12'h342: mcause  <= csr_wdata;
          default: ;
        endcase
      end
    end
  end
  // mvendorid / mimpid intentionally ignore writes.
endmodule
