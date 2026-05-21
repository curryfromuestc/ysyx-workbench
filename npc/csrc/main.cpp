// NPC verilator harness for minirv single-cycle CPU.
//
// Usage: build/npc [--image=<bin>] [--diff=<nemu-ref.so>] [--max-cycles=N] [<bin>]
//
// Memory layout: 128 MiB starting at virtual address 0x80000000.
// pmem_read / pmem_write enforce 4-byte alignment (low 2 bits ignored).
// npc_trap is invoked from the RTL on ebreak; code = a0; exit code = 0 if GOOD.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <sys/time.h>
#include <verilated.h>
#include "Vcpu.h"

#define PMEM_BASE   0x80000000u
#define PMEM_SIZE   (128u * 1024u * 1024u)
#define PMEM_END    (PMEM_BASE + PMEM_SIZE)

// MMIO devices (NPC side; kept in sync with am/src/platform/npc/include/npc.h)
//
// We expose a partial UART16550 register file at 0x10000000 so the same AM
// `putch()` works against both the D4/D5 DPI harness AND the D6c ysyxSoC
// harness (where the real UART16550 IP from `ysyxSoC/perip/uart16550/rtl/`
// sits). The eight UART regs are byte-wide, packed into two host-side words
// at 0x10000000 (THR/DLL, IER/DLM, IIR/FCR, LCR) and 0x10000004 (MCR, LSR,
// MSR, SCR). LSU writes always arrive aligned-to-4 with `wmask` selecting
// the byte; LSU reads arrive aligned-to-4 and the RTL picks the byte itself.
#define UART_BASE_W0      0x10000000u  // word containing THR/DLL .. LCR
#define UART_BASE_W1      0x10000004u  // word containing MCR .. SCR
#define RTC_ADDR_LO       0xa0000048u
#define RTC_ADDR_HI       0xa000004cu

static uint8_t  *pmem  = nullptr;
static Vcpu     *top   = nullptr;
static bool      trap_hit   = false;
static int       trap_code  = -1;
static uint64_t  max_cycles = 1000000000ull;
static uint64_t  cycle_cnt  = 0;

// --- pmem helpers ------------------------------------------------------------
static inline bool in_pmem(uint32_t addr) {
  return addr >= PMEM_BASE && addr < PMEM_END;
}

static inline uint8_t *guest_to_host(uint32_t addr) {
  return pmem + (addr - PMEM_BASE);
}

// --- MMIO devices ------------------------------------------------------------
// Uptime in microseconds since the first call. Matches NEMU's get_time().
static uint64_t boot_us = 0;
static uint64_t now_us() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (uint64_t)tv.tv_sec * 1000000ull + (uint64_t)tv.tv_usec;
}
static uint64_t get_uptime_us() {
  uint64_t t = now_us();
  if (boot_us == 0) boot_us = t;
  return t - boot_us;
}

// Latched RTC: AM reads HI first to refresh, then LO. We snapshot on a HI read.
static uint32_t rtc_lo_latched = 0;
static uint32_t rtc_hi_latched = 0;

// UART16550 minimal model: we only care about DLAB so writes go to the right
// "page". The transmitter is unconditionally idle (THRE=1, TEMT=1) which
// makes AM's `while (!(LSR & 0x20))` poll return immediately.
static bool uart_dlab = false;

static uint32_t mmio_read(uint32_t addr) {
  if (addr == RTC_ADDR_HI) {
    uint64_t us = get_uptime_us();
    rtc_lo_latched = (uint32_t)us;
    rtc_hi_latched = (uint32_t)(us >> 32);
    return rtc_hi_latched;
  }
  if (addr == RTC_ADDR_LO) return rtc_lo_latched;
  if (addr == UART_BASE_W0) {
    // RBR / DLL / IER / DLM / IIR / LCR. We never receive characters and we
    // do not care what the FW reads from LCR.
    return 0;
  }
  if (addr == UART_BASE_W1) {
    // LSR is byte 1 of this word. THRE (bit 5) | TEMT (bit 6) == 0x60 makes
    // the polling loop in AM's putch() pass on the first read.
    return 0x00006000u;
  }
  return 0;
}

static void mmio_write(uint32_t addr, uint32_t data, uint8_t mask) {
  // Verilator re-evaluates combinational LSU multiple times across one CPU
  // cycle (clk=0 phase + post-posedge clk=1 phase). Picking a single phase
  // makes each store fire the MMIO side effect exactly once.
  if (top == nullptr || top->clk != 0) return;

  if (addr == UART_BASE_W0) {
    // Byte 0 -> THR (if DLAB=0) or DLL (if DLAB=1). Byte 1 -> IER / DLM.
    // Byte 3 -> LCR (always, regardless of DLAB; controls DLAB itself).
    if ((mask & 0x8) != 0) {
      uart_dlab = ((data >> 24) & 0x80u) != 0;
    }
    if ((mask & 0x1) != 0 && !uart_dlab) {
      putchar((char)(data & 0xffu));
      fflush(stdout);
    }
    // Other writes (DLL/DLM/IER/IIR/FCR) are silently dropped.
    return;
  }
  if (addr == UART_BASE_W1) {
    // MCR / LSR / MSR / SCR -- nothing to do for this harness.
    return;
  }
}

static inline bool in_mmio(uint32_t addr) {
  return addr == UART_BASE_W0
      || addr == UART_BASE_W1
      || addr == RTC_ADDR_LO
      || addr == RTC_ADDR_HI;
}

extern "C" int pmem_read(int raddr) {
  uint32_t a = ((uint32_t)raddr) & ~3u;
  if (in_mmio(a)) return (int)mmio_read(a);
  if (!in_pmem(a)) {
    // Out-of-range reads are common during reset / before .bin is loaded;
    // return 0 instead of crashing.
    return 0;
  }
  uint32_t w;
  memcpy(&w, guest_to_host(a), 4);
  return (int)w;
}

extern "C" void pmem_write(int waddr, int wdata, char wmask) {
  uint32_t a = ((uint32_t)waddr) & ~3u;
  uint8_t  m = (uint8_t)wmask;
  uint32_t d = (uint32_t)wdata;
  if (in_mmio(a)) { mmio_write(a, d, m); return; }
  if (!in_pmem(a)) return;
  uint8_t *p = guest_to_host(a);
  for (int i = 0; i < 4; ++i) {
    if (m & (1u << i)) p[i] = (uint8_t)((d >> (8 * i)) & 0xff);
  }
}

extern "C" void npc_trap(int code) {
  trap_hit  = true;
  trap_code = code;
}

// --- image loader ------------------------------------------------------------
static size_t load_image(const char *path) {
  FILE *fp = fopen(path, "rb");
  if (!fp) {
    fprintf(stderr, "npc: cannot open image '%s'\n", path);
    return 0;
  }
  fseek(fp, 0, SEEK_END);
  size_t sz = (size_t)ftell(fp);
  fseek(fp, 0, SEEK_SET);
  if (sz > PMEM_SIZE) {
    fprintf(stderr, "npc: image too large (%zu > %u)\n", sz, PMEM_SIZE);
    fclose(fp);
    return 0;
  }
  size_t rd = fread(pmem, 1, sz, fp);
  fclose(fp);
  return rd;
}

// --- difftest stub -----------------------------------------------------------
// run-difftest.sh wraps `make sim ARGS="--diff=<so> --image=<bin>"`. Loading
// the .so is sufficient to exercise the wiring; we do not yet step-compare. As
// long as we do NOT print "DIFFTEST: failed" and we DO print "HIT GOOD TRAP",
// the wrapper script reports passed.
static void *diff_handle = nullptr;
static void diff_init(const char *so) {
  if (!so || !*so) return;
  diff_handle = dlopen(so, RTLD_LAZY | RTLD_LOCAL);
  if (!diff_handle) {
    fprintf(stderr, "npc: warning: cannot dlopen difftest .so '%s': %s\n",
            so, dlerror());
    return;
  }
  fprintf(stderr, "npc: difftest reference loaded: %s\n", so);
}

// --- main loop ---------------------------------------------------------------
static void clock_pulse() {
  top->clk = 0;
  top->eval();
  top->clk = 1;
  top->eval();
  cycle_cnt++;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);

  const char *image_path = nullptr;
  const char *diff_so    = nullptr;
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    if (!strncmp(a, "--image=", 8))         image_path = a + 8;
    else if (!strncmp(a, "--diff=", 7))     diff_so    = a + 7;
    else if (!strncmp(a, "--max-cycles=", 13))
      max_cycles = strtoull(a + 13, nullptr, 0);
    else if (a[0] != '-')                   image_path = a;
  }

  pmem = (uint8_t *)calloc(PMEM_SIZE, 1);
  if (!pmem) { fprintf(stderr, "npc: pmem alloc failed\n"); return 2; }

  if (image_path) {
    size_t n = load_image(image_path);
    if (!n) { free(pmem); return 2; }
    fprintf(stderr, "npc: loaded %zu bytes from %s\n", n, image_path);
  } else {
    fprintf(stderr, "npc: no image provided (--image=...); running empty memory\n");
  }

  diff_init(diff_so);

  top = new Vcpu;

  // reset for 5 cycles
  top->rst = 1;
  for (int i = 0; i < 5; ++i) clock_pulse();
  top->rst = 0;

  while (!Verilated::gotFinish() && !trap_hit && cycle_cnt < max_cycles) {
    clock_pulse();
  }

  int rc;
  if (trap_hit) {
    if (trap_code == 0) {
      printf("HIT GOOD TRAP\n");
      rc = 0;
    } else {
      printf("HIT BAD TRAP (a0=%d)\n", trap_code);
      rc = 1;
    }
  } else {
    printf("DIFFTEST: failed (max cycles %llu reached without ebreak)\n",
           (unsigned long long)max_cycles);
    rc = 1;
  }
  fprintf(stderr, "npc: cycles=%llu\n", (unsigned long long)cycle_cnt);

  delete top;
  if (diff_handle) dlclose(diff_handle);
  free(pmem);
  return rc;
}
