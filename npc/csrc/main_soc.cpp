// =============================================================================
// NPC SoC verilator harness (D6a).
// =============================================================================
// Top module: ysyxSoCFull. We drive only clock + reset; the SoC pulls
// the CPU (ysyx_22040000) via MemBridge and the AXI fabric. UART output goes
// to externalPins_uart_tx but we ignore that pin for D6a -- we only need to
// boot the CPU far enough to fetch the first instruction from Flash, which
// will trigger flash_read() in flash.v.
//
// For D6a we deliberately leave flash_read() as `assert(0)`: this is the
// expected stopping condition documented in docs-md/2407/d/6.md step 12.
// =============================================================================

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <verilated.h>
#include "VysyxSoCFull.h"

static VysyxSoCFull *top        = nullptr;
static uint64_t      cycle_cnt  = 0;
static uint64_t      max_cycles = 200000ull; // D6a: only need a handful

// DPI-C stub required by flash.v. The real implementation lands in D6c.
extern "C" void flash_read(int32_t addr, int32_t *data) {
  (void)addr;
  (void)data;
  fprintf(stderr, "npc-soc: flash_read() called at cycle %llu (addr=0x%08x).\n",
          (unsigned long long)cycle_cnt, (uint32_t)addr);
  fprintf(stderr, "npc-soc: D6a stop point reached -- assert(0) per d/6.md step 12.\n");
  fflush(stderr);
  assert(0 && "flash_read placeholder: D6a expected stop point");
}

static void clock_pulse() {
  top->clock = 0;
  top->eval();
  top->clock = 1;
  top->eval();
  cycle_cnt++;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);

  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    if (!strncmp(a, "--max-cycles=", 13))
      max_cycles = strtoull(a + 13, nullptr, 0);
  }

  top = new VysyxSoCFull;
  top->externalPins_uart_rx = 0;

  // Reset must be held at least 10 cycles (d/6.md step 10).
  top->reset = 1;
  for (int i = 0; i < 16; ++i) clock_pulse();
  top->reset = 0;
  fprintf(stderr, "npc-soc: reset released at cycle %llu\n",
          (unsigned long long)cycle_cnt);

  while (!Verilated::gotFinish() && cycle_cnt < max_cycles) {
    clock_pulse();
  }

  if (cycle_cnt >= max_cycles) {
    fprintf(stderr,
            "npc-soc: WARNING: hit max cycles (%llu) without reaching "
            "flash_read(). Bus wiring may be wrong.\n",
            (unsigned long long)max_cycles);
  }

  delete top;
  return 0;
}
