// =============================================================================
// NPC SoC verilator harness (D6a + D6c).
// =============================================================================
// Top module: ysyxSoCFull. We drive only clock + reset; the SoC pulls the CPU
// (ysyx_22040000) via MemBridge and the AXI fabric.
//
// D6a: flash_read() was just assert(0) -- the first instruction fetch from
//      0x30000000 triggered it, which validated the bus wiring.
// D6c: flash_read() now reads from a 16 MiB host-side flash array that is
//      pre-loaded from a .bin file. The CPU therefore actually executes the
//      boot loader at 0x30000000, which in turn copies the hello image into
//      SDRAM at 0x80000000 and jumps to it. UART output is produced by
//      uart_tfifo.v's `$write("%c", ...)` so we don't need to model the wire.
//
// CLI:
//   --flash=<path>     .bin to load into Flash[0..]. Default: the prebuilt
//                      hello-minirv-ysyxsoc.bin shipped with ysyxSoC.
//   --max-cycles=<n>   hard timeout in cycles (the simulated hello image
//                      never halts on its own once it finishes printing).
// =============================================================================

#include <cassert>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <verilated.h>
#include "VysyxSoCFull.h"
#include "VysyxSoCFull___024root.h"

// ---- Flash backing store ----------------------------------------------------
// flash.v drives a 23-bit address (plus mosi for bit 0), so the addressable
// range is 0..0xFFFFFF (16 MiB). We back it with a host array of int32 because
// flash_read() returns one 32-bit word per call.
static constexpr size_t FLASH_BYTES = 16 * 1024 * 1024;
static int32_t flash_mem[FLASH_BYTES / sizeof(int32_t)];

static const char *default_flash_path =
    "../ysyxSoC/ready-to-run/D-stage/hello-minirv-ysyxsoc.bin";

static VysyxSoCFull *top        = nullptr;
static uint64_t      cycle_cnt  = 0;
// Default deep enough to print "Hello World!" via the slow SPI-flash boot path
// (loader copies ~290 KiB over the SPI bus, then hello runs). Empirically the
// prebuilt hello finishes printing around cycle 870M. Programs that ebreak
// (e.g. dummy) finish much sooner. Override via --max-cycles=.
static uint64_t      max_cycles = 1500000000ull;

static void load_flash(const char *path) {
  memset(flash_mem, 0, sizeof(flash_mem));

  FILE *fp = fopen(path, "rb");
  if (!fp) {
    fprintf(stderr, "npc-soc: cannot open flash image '%s': %s\n", path,
            strerror(errno));
    exit(1);
  }
  fseek(fp, 0, SEEK_END);
  long sz = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  if (sz < 0 || (size_t)sz > FLASH_BYTES) {
    fprintf(stderr,
            "npc-soc: flash image '%s' is %ld bytes, max is %zu\n",
            path, sz, FLASH_BYTES);
    fclose(fp);
    exit(1);
  }
  size_t n = fread(flash_mem, 1, (size_t)sz, fp);
  fclose(fp);
  if (n != (size_t)sz) {
    fprintf(stderr, "npc-soc: short read on '%s' (%zu/%ld)\n", path, n, sz);
    exit(1);
  }
  fprintf(stderr, "npc-soc: loaded %ld bytes from '%s' into Flash\n", sz, path);
}

// DPI-C entry from flash.v. `addr` is the 23-bit byte address shifted in via
// SPI (range 0..0xFFFFFF). We return the 32-bit little-endian word that lives
// at that 4-byte boundary -- the surrounding RTL byte-swaps and serialises it
// back over miso, so the byte order seen by the CPU matches what we stored.
extern "C" void flash_read(int32_t addr, int32_t *data) {
  uint32_t a = (uint32_t)addr & 0x00FFFFFFu;
  // flash.v always fires reads on word boundaries (the loader and AXI master
  // request 4 bytes at a time), so masking the bottom two bits is sufficient.
  *data = flash_mem[(a & ~0x3u) / 4];
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

  // Long-running images (D6d litenes/Mario) emit ~2 KiB of UART traffic per
  // frame separated by long quiet periods. Without unbuffered stdout the
  // first frames disappear from the user's terminal until a few KiB of
  // newlines accumulate, which makes "is anything happening?" debugging
  // miserable. Disable buffering up front so $write("%c", ...) from the
  // uart_tfifo model shows up immediately.
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  const char *flash_path = default_flash_path;
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    if (!strncmp(a, "--flash=", 8))            flash_path = a + 8;
    else if (!strncmp(a, "--max-cycles=", 13)) max_cycles = strtoull(a + 13, nullptr, 0);
  }

  load_flash(flash_path);

  top = new VysyxSoCFull;
  top->externalPins_uart_rx = 0;

  // Reset must be held at least 10 cycles (d/6.md step 10).
  top->reset = 1;
  for (int i = 0; i < 16; ++i) clock_pulse();
  top->reset = 0;
  fprintf(stderr, "npc-soc: reset released at cycle %llu\n",
          (unsigned long long)cycle_cnt);
  fflush(stderr);

  // ysyx_22040000.v parks the FSM in S_EX on ebreak (no exception support
  // yet), so we detect it host-side: if inst_r decodes to ebreak (0x00100073)
  // and state == S_EX (== 1) we emit HIT GOOD/BAD TRAP using a0 and stop.
  auto *r = top->rootp;
  bool trap_hit = false;
  int  trap_code = 0;
  // B2a: keep tally of Access Fault events. ysyx_22040000.v raises any_fault
  // (for one cycle) whenever AXI bresp/rresp != 2'b00 and the CPU
  // simultaneously had an in-flight IFU/LSU request. Today no live ysyxSoC
  // slave returns SLVERR/DECERR (every APB slave hard-ties pslverr=0), but
  // tracking it here makes the metric available the moment a future
  // microbench (B2c) or out-of-range probe trips one.
  uint64_t fault_events = 0;
  while (!Verilated::gotFinish() && cycle_cnt < max_cycles) {
    clock_pulse();
    if (r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__any_fault) {
      ++fault_events;
    }
    if (r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__state == 1 &&
        r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__inst_r == 0x00100073u) {
      trap_hit  = true;
      // a0 is x10. With ABI=ilp32e RV32E the regfile has 16 entries; a0 is rf[10].
      trap_code = (int)r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__u_rf__DOT__rf[10];
      break;
    }
  }

  if (trap_hit) {
    fflush(stdout);
    if (trap_code == 0) {
      printf("\nHIT GOOD TRAP\n");
    } else {
      printf("\nHIT BAD TRAP (a0=%d)\n", trap_code);
    }
    fflush(stdout);
    fprintf(stderr, "npc-soc: ebreak hit at cycle %llu\n",
            (unsigned long long)cycle_cnt);
  } else if (cycle_cnt >= max_cycles) {
    fflush(stdout);
    fprintf(stderr,
            "\nnpc-soc: hit max cycles (%llu) without ebreak -- "
            "for the prebuilt hello image this is the expected idle loop.\n",
            (unsigned long long)max_cycles);
  }
  fprintf(stderr, "npc-soc: B2a access-fault events: %llu\n",
          (unsigned long long)fault_events);

  // B4a: 退出前打印 icache 性能计数器 (verilator 通过 public_flat_rd 把它们
  // 抬到 root scope 的扁平符号表里).
  uint64_t ic_access = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__u_icache__DOT__cnt_access;
  uint64_t ic_hit    = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__u_icache__DOT__cnt_hit;
  uint64_t ic_miss   = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__u_icache__DOT__cnt_miss;
  double hit_rate = (ic_access == 0) ? 0.0 : (double)ic_hit * 100.0 / (double)ic_access;
  fprintf(stderr, "npc-soc: icache access=%llu hit=%llu miss=%llu hit_rate=%.2f%%\n",
          (unsigned long long)ic_access,
          (unsigned long long)ic_hit,
          (unsigned long long)ic_miss,
          hit_rate);

  delete top;
  return 0;
}
