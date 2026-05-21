// CSR file for NPC (D6).
// Supports the 6 Zicsr instructions: csrrw / csrrs / csrrc / csrrwi / csrrsi / csrrci.
//
// Implemented CSRs:
//   0xb00 mcycle      - free-running cycle counter (low 32 bits)
//   0xf11 mvendorid   - student ID, fixed 32'h2204_0000 (== "ysyx_22040000" as integer
//                       0x22040000 since the ASCII full string does not fit in 32 bits)
//   0xf13 mimpid      - alias for student id (also fixed 0x22040000)
//
// Behaviour summary:
//   - mcycle increments every clock cycle while !reset.
//   - On a CSR instruction with csr_we=1, the destination CSR is overwritten with
//     csr_wdata (after the read happens, so rd already sees the OLD value).
//   - csr_rdata is combinational and reflects the CURRENT register value, so the
//     consumer can latch it on the same posedge.
//   - The wrapping CPU is responsible for synthesising csr_wdata from the
//     csrr* variant (rs1 / imm, set / clear / write).
//
// Note: this module is intentionally small. There is no support yet for
// exceptions, mstatus, mepc, mtvec etc.; D6 only needs the counter and ID.

module CSR(
  input         clk,
  input         rst,
  // Read port (combinational)
  input  [11:0] csr_raddr,
  output [31:0] csr_rdata,
  // Write port (clocked)
  input         csr_we,
  input  [11:0] csr_waddr,
  input  [31:0] csr_wdata
);
  // ---- registers ------------------------------------------------------------
  reg [31:0] mcycle;

  // student ID constant: ysyx_22040000 -> we expose 32'h2204_0000 in mvendorid /
  // mimpid so software can identify which student's core is running.
  localparam [31:0] STUDENT_ID = 32'h2204_0000;

  // ---- read mux (combinational) --------------------------------------------
  reg [31:0] rdata_r;
  always @(*) begin
    case (csr_raddr)
      12'hb00: rdata_r = mcycle;
      12'hf11: rdata_r = STUDENT_ID; // mvendorid
      12'hf13: rdata_r = STUDENT_ID; // mimpid
      default: rdata_r = 32'h0;
    endcase
  end
  assign csr_rdata = rdata_r;

  // ---- mcycle: free-running counter ----------------------------------------
  // Increments every cycle unless software is writing mcycle this cycle.
  always @(posedge clk) begin
    if (rst) begin
      mcycle <= 32'h0;
    end else if (csr_we && csr_waddr == 12'hb00) begin
      mcycle <= csr_wdata;
    end else begin
      mcycle <= mcycle + 32'd1;
    end
  end

  // Read-only CSRs (mvendorid / mimpid) intentionally ignore writes.
endmodule
