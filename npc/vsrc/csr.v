// CSR file for NPC.
//
// Supports the 6 Zicsr instructions: csrrw / csrrs / csrrc / csrrwi / csrrsi / csrrci.
//
// Implemented CSRs:
//   0x300 mstatus     - plain R/W (no side effects; not used for exceptions yet)
//   0x305 mtvec       - plain R/W; exposed as `mtvec_out` so WBU can redirect
//                       PC on ecall trap.
//   0x341 mepc        - plain R/W; exposed as `mepc_out` so WBU/mret can redirect PC
//   0x342 mcause      - plain R/W
//   0xb00 mcycle      - free-running cycle counter (low 32 bits)
//   0xf11 mvendorid   - ASCII "ysyx" = 0x79737978
//   0xf12 marchid     - student-id decimal 22040000 = 0x01504DC0
//   0xf13 mimpid      - alias of mvendorid (kept for backwards compat)
//
// Trap port:
//   When `is_trap` is asserted, mcause <= trap_cause and mepc <= trap_epc at the
//   next clock edge. The trap port races CSR explicit writes; the trap wins so
//   the architectural state matches what the WBU is doing to the PC the same
//   cycle (we never issue an ecall and a csrw on the same instruction anyway).
//
// Notes on mret:
//   This file does NOT auto-trigger anything on mret; the wrapping CPU detects
//   is_mret from the IDU and feeds mepc into the WBU's pc_next mux. mret in
//   ilp32e RV32E does not push/pop mstatus.MPIE/MIE here (no interrupts yet).
//
// Behaviour summary:
//   - mcycle increments every clock cycle while !reset.
//   - On a CSR instruction with csr_we=1, the destination CSR is overwritten with
//     csr_wdata (after the read happens, so rd already sees the OLD value).
//   - On `is_trap`, mcause/mepc are written from the trap_cause/trap_epc inputs
//     and any concurrent csr_we to those CSRs is masked.

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
  // Trap port (one-cycle pulse)
  input         is_trap,
  input  [31:0] trap_cause,
  input  [31:0] trap_epc,
  // mepc/mtvec continuously exposed for WBU's mret / trap redirect path.
  output [31:0] mepc_out,
  output [31:0] mtvec_out
);
  reg [31:0] mcycle;
  reg [31:0] mstatus;
  reg [31:0] mtvec;
  reg [31:0] mepc;
  reg [31:0] mcause;

  localparam [31:0] VENDOR_ID = 32'h7973_7978; // "ysyx" ASCII
  localparam [31:0] ARCH_ID   = 32'h0150_4DC0; // 22040000 decimal

  reg [31:0] rdata_r;
  always @(*) begin
    case (csr_raddr)
      12'h300: rdata_r = mstatus;
      12'h305: rdata_r = mtvec;
      12'h341: rdata_r = mepc;
      12'h342: rdata_r = mcause;
      12'hb00: rdata_r = mcycle;
      12'hf11: rdata_r = VENDOR_ID; // mvendorid
      12'hf12: rdata_r = ARCH_ID;   // marchid
      12'hf13: rdata_r = VENDOR_ID; // mimpid (alias of vendor id)
      default: rdata_r = 32'h0;
    endcase
  end
  assign csr_rdata = rdata_r;
  assign mepc_out  = mepc;
  assign mtvec_out = mtvec;

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
      // Trap wins over explicit csrw for mcause/mepc.
      if (is_trap) begin
        mcause <= trap_cause;
        mepc   <= trap_epc;
      end
      // Plain R/W CSRs. Skip mcause/mepc when a trap is being committed so we
      // do not race the trap-side write.
      if (csr_we) begin
        case (csr_waddr)
          12'h300: mstatus <= csr_wdata;
          12'h305: mtvec   <= csr_wdata;
          12'h341: if (!is_trap) mepc   <= csr_wdata;
          12'h342: if (!is_trap) mcause <= csr_wdata;
          default: ;
        endcase
      end
    end
  end
  // mvendorid / marchid / mimpid intentionally ignore writes.
endmodule
