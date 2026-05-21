// =============================================================================
// TRM for the NPC target -- D5 (DPI MMIO) + D6c (ysyxSoC UART16550).
// =============================================================================
// We use the UART16550 register layout from `ysyxSoC/perip/uart16550/rtl/`:
//   base = 0x10000000
//   +0   THR (DLAB=0) / DLL (DLAB=1)
//   +1   IER (DLAB=0) / DLM (DLAB=1)
//   +2   IIR / FCR
//   +3   LCR        bit 7 = DLAB
//   +5   LSR        bit 5 = THRE (transmit FIFO empty)
//                   bit 6 = TEMT (transmitter empty)
//
// The D5 DPI harness (`npc/csrc/main.cpp`) emulates exactly this layout: it
// honours LCR/DLL/DLM writes silently and returns LSR=0x60 (TFE|TEMT=1) so
// the polling loop below succeeds immediately. The same `putch()` therefore
// works for both the DPI harness AND the ysyxSoC verilator harness, which
// keeps `make sim` (D4/D5) and `make sim-soc` (D6c) on a single code path.
//
// `_trm_init` initialises the UART exactly once: 8N1, divisor=1 (fastest the
// model allows -- divisor 0 disables the transmitter entirely; see
// uart_regs.v's "if (|dl & ~(|dlc))" guard).
// =============================================================================

#include <am.h>
#include <klib-macros.h>
#include <npc.h>
#include "../riscv.h"

#define UART_BASE   0x10000000u
#define UART_THR    (UART_BASE + 0)
#define UART_DLL    (UART_BASE + 0)
#define UART_DLM    (UART_BASE + 1)
#define UART_LCR    (UART_BASE + 3)
#define UART_LSR    (UART_BASE + 5)

#define LCR_8N1     0x03u   // 8 data bits, no parity, 1 stop bit
#define LCR_DLAB    0x80u
#define LSR_THRE    0x20u   // transmit FIFO empty -- safe to write THR

extern char _heap_start;
int main(const char *args);

Area heap = RANGE(&_heap_start, PMEM_END);
static const char mainargs[MAINARGS_MAX_LEN] = TOSTRING(MAINARGS_PLACEHOLDER); // defined in CFLAGS

static void uart_init(void) {
  // Unlock divisor latch.
  outb(UART_LCR, LCR_8N1 | LCR_DLAB);
  outb(UART_DLL, 1);   // smallest non-zero divisor (uart_regs.v: dl==0 stalls)
  outb(UART_DLM, 0);
  // Lock back to 8N1 so subsequent writes hit THR/IER/etc.
  outb(UART_LCR, LCR_8N1);
}

void putch(char ch) {
  // Wait for the transmit FIFO to be empty. The DPI harness pins THRE high so
  // this is a single read; the ysyxSoC harness will block until the SoC's
  // UART16550 finishes draining the FIFO.
  while ((inb(UART_LSR) & LSR_THRE) == 0) { }
  outb(UART_THR, ch);
}

void halt(int code) {
  // Pass exit code via a0, then ebreak so the NPC simulator stops with
  // HIT GOOD / BAD TRAP.
  asm volatile("mv a0, %0; ebreak" : : "r"(code));
  while (1);
}

void _trm_init() {
  uart_init();
  int ret = main(mainargs);
  halt(ret);
}
