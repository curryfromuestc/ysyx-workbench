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
#include "Vcpu___024root.h"
#ifdef VCD_TRACE
#include <verilated_vcd_c.h>
#endif

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

// --- difftest ----------------------------------------------------------------
// C2b: dlopen the NEMU REF .so (built from
// nemu/configs/riscv32-nemu-share_defconfig), then on every retired
// instruction step REF by one and compare PC + 16 GPRs against the NPC RTL.
//
// NPC is a single-cycle CPU: each posedge clk retires exactly one instruction
// (unless `rst` is asserted or the trap signal has fired). The trap fires
// asynchronously from the DPI npc_trap() call on ebreak; the cycle that
// retires the ebreak should NOT step REF (REF would do an additional ebreak
// instead of stopping).
//
// CPU_state-shaped buffer we hand to ref_difftest_regcpy. Layout MUST match
// nemu/src/isa/riscv32/include/isa-def.h::riscv32_CPU_state when REF is built
// with CONFIG_RVE=y (which our riscv32-nemu-share_defconfig guarantees):
//   uint32_t gpr[16];
//   uint32_t pc;
//   uint32_t mstatus, mtvec, mepc, mcause;   // CSR tail
// Total = (16 + 1 + 4) * 4 = 84 bytes.
struct NpcRegState {
  uint32_t gpr[16];
  uint32_t pc;
  uint32_t mstatus;
  uint32_t mtvec;
  uint32_t mepc;
  uint32_t mcause;
};

typedef void (*ref_memcpy_fn)(uint32_t addr, void *buf, size_t n, bool dir);
typedef void (*ref_regcpy_fn)(void *buf, bool dir);
typedef void (*ref_exec_fn)(uint64_t n);
typedef void (*ref_init_fn)(int port);

static void           *diff_handle = nullptr;
static bool            diff_enabled = false;
static ref_memcpy_fn   ref_memcpy = nullptr;
static ref_regcpy_fn   ref_regcpy = nullptr;
static ref_exec_fn     ref_exec   = nullptr;
static ref_init_fn     ref_init   = nullptr;
// Latched at clk=0 phase, consumed after clk=1 phase. Records whether the
// instruction that is ABOUT TO retire touches an MMIO address. We sample
// pre-edge because right after the posedge the combinational LSU signals
// have already swung to the NEXT (post-PC-update) instruction.
static bool            diff_was_mmio_this_cycle = false;

enum { DIFF_TO_DUT = 0, DIFF_TO_REF = 1 };

// Pull RTL state straight out of the Verilator-generated root. Path names
// here mirror what verilator emits for `cpu.u_rf.rf` etc. — they will drift
// if the RTL module hierarchy changes, in which case grep them out of
// build/obj_dir/Vcpu___024root.h again.
static inline uint32_t rtl_pc()                  { return top->rootp->cpu__DOT__pc; }
static inline uint32_t rtl_gpr(int i)            { return top->rootp->cpu__DOT__u_rf__DOT__rf[i]; }
static inline uint32_t rtl_lsu_addr() {
#ifdef VCD_TRACE
  return 0;  // not used in trace mode (difftest disabled)
#else
  return top->rootp->cpu__DOT__u_exu__DOT__alu_result;
#endif
}
static inline bool     rtl_lsu_active() {
#ifdef VCD_TRACE
  // --trace inlines the cell-input wires away; in trace mode we don't run
  // difftest, so we can safely return false (no MMIO skip needed).
  return false;
#else
  return top->rootp->cpu__DOT____Vcellinp__u_lsu__mem_re
      || top->rootp->cpu__DOT____Vcellinp__u_lsu__mem_we;
#endif
}

static void snapshot_dut(NpcRegState *s) {
  for (int i = 0; i < 16; ++i) s->gpr[i] = rtl_gpr(i);
  s->pc      = rtl_pc();
  s->mstatus = 0;            // CSR not modelled in NPC RV32E for now
  s->mtvec   = 0;
  s->mepc    = 0;
  s->mcause  = 0;
}

// Difftest's MMIO predicate. Word-aligns first because alu_result is a raw
// byte address (e.g. 0x10000003 for an LCR store), then reuses in_mmio().
static inline bool addr_is_mmio(uint32_t a) {
  return in_mmio(a & ~3u);
}

static void diff_init_so(const char *so) {
  if (!so || !*so) return;
  diff_handle = dlopen(so, RTLD_LAZY | RTLD_LOCAL);
  if (!diff_handle) {
    fprintf(stderr, "npc: warning: cannot dlopen difftest .so '%s': %s\n",
            so, dlerror());
    return;
  }
  ref_memcpy = (ref_memcpy_fn) dlsym(diff_handle, "difftest_memcpy");
  ref_regcpy = (ref_regcpy_fn) dlsym(diff_handle, "difftest_regcpy");
  ref_exec   = (ref_exec_fn)   dlsym(diff_handle, "difftest_exec");
  ref_init   = (ref_init_fn)   dlsym(diff_handle, "difftest_init");
  if (!ref_memcpy || !ref_regcpy || !ref_exec || !ref_init) {
    fprintf(stderr, "npc: warning: difftest .so missing required symbols\n");
    dlclose(diff_handle);
    diff_handle = nullptr;
    return;
  }
  ref_init(0);
  diff_enabled = true;
  fprintf(stderr, "npc: difftest enabled (REF=%s)\n", so);
}

// Ship the program image into REF and align REF's CPU state with NPC's
// reset state. Called once after RTL reset has been released.
static void diff_sync_from_dut(size_t img_size) {
  if (!diff_enabled) return;
  // Reset state of NPC: gpr=0, pc=0x80000000 (matches NEMU init_isa).
  // Copy the loaded program over to REF so its first cpu_exec(1) fetches the
  // same bytes NPC's IFU does.
  if (img_size > 0) {
    ref_memcpy(PMEM_BASE, pmem, img_size, DIFF_TO_REF);
  }
  NpcRegState st;
  snapshot_dut(&st);
  ref_regcpy(&st, DIFF_TO_REF);
}

// Compare REF and DUT after one retired instruction. If they diverge, print
// a verbose diff and arrange a clean shutdown via trap_hit (so main() prints
// "DIFFTEST: failed" and the wrapper script picks it up).
static void diff_step_and_check() {
  if (!diff_enabled) return;

  // Skip MMIO instructions: REF's MMIO model (none, because DEVICE=n in
  // share_defconfig) would diverge from NPC's UART/RTC. Copy DUT->REF and
  // do NOT advance REF for this instruction. The semantic is identical to
  // NEMU's difftest_skip_ref() — see nemu/src/cpu/difftest/dut.c.
  // We must read the LSU signals BEFORE clocking again, so this function is
  // called immediately after the clk=1 eval of the retiring instruction.
  // At that point the combinational LSU signals reflect the NEXT instr (PC
  // already advanced), so we cache the predicate state pre-edge.
  bool was_mmio = diff_was_mmio_this_cycle;
  diff_was_mmio_this_cycle = false;

  if (was_mmio) {
    NpcRegState st;
    snapshot_dut(&st);
    ref_regcpy(&st, DIFF_TO_REF);
    return;
  }

  ref_exec(1);

  NpcRegState ref_st;
  ref_regcpy(&ref_st, DIFF_TO_DUT);

  bool bad = false;
  uint32_t dut_pc = rtl_pc();
  if (ref_st.pc != dut_pc) {
    fprintf(stderr, "DIFFTEST: pc mismatch @cycle %llu: ref=0x%08x dut=0x%08x\n",
            (unsigned long long)cycle_cnt, ref_st.pc, dut_pc);
    bad = true;
  }
  for (int i = 0; i < 16; ++i) {
    uint32_t r = ref_st.gpr[i];
    uint32_t d = rtl_gpr(i);
    if (r != d) {
      fprintf(stderr, "DIFFTEST: x%-2d mismatch @cycle %llu: ref=0x%08x dut=0x%08x\n",
              i, (unsigned long long)cycle_cnt, r, d);
      bad = true;
    }
  }
  if (bad) {
    fprintf(stderr, "DIFFTEST: failed (first mismatch at cycle %llu, dut pc=0x%08x)\n",
            (unsigned long long)cycle_cnt, dut_pc);
    // Force loop exit. We reuse trap_hit so the "DIFFTEST: failed" print at
    // the end of main() also lands.
    trap_hit  = true;
    trap_code = -2;
  }
}

// --- main loop ---------------------------------------------------------------
// Drive one clock cycle. When difftest is enabled we sample the LSU's MMIO
// predicate at clk=0 phase (after combinational has settled, before the
// register update), then trigger the posedge. The caller is responsible for
// invoking diff_step_and_check() after this returns, unless `rst` is high.
#ifdef VCD_TRACE
static VerilatedVcdC *tfp = nullptr;
static uint64_t       vcd_time = 0;
#endif

static void clock_pulse() {
  top->clk = 0;
  top->eval();
#ifdef VCD_TRACE
  if (tfp) { tfp->dump(vcd_time); vcd_time++; }
#endif
  if (diff_enabled && !top->rst) {
    uint32_t a = rtl_lsu_addr();
    diff_was_mmio_this_cycle = rtl_lsu_active() && addr_is_mmio(a);
  }
  top->clk = 1;
  top->eval();
#ifdef VCD_TRACE
  if (tfp) { tfp->dump(vcd_time); vcd_time++; }
#endif
  cycle_cnt++;
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);

  const char *image_path = nullptr;
  const char *diff_so    = nullptr;
  const char *trace_path = nullptr;
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    if (!strncmp(a, "--image=", 8))         image_path = a + 8;
    else if (!strncmp(a, "--diff=", 7))     diff_so    = a + 7;
    else if (!strncmp(a, "--trace=", 8))    trace_path = a + 8;
    else if (!strncmp(a, "--max-cycles=", 13))
      max_cycles = strtoull(a + 13, nullptr, 0);
    else if (a[0] != '-')                   image_path = a;
  }
  (void)trace_path;
#ifdef VCD_TRACE
  if (trace_path) {
    Verilated::traceEverOn(true);
    tfp = new VerilatedVcdC;
  }
#endif

  pmem = (uint8_t *)calloc(PMEM_SIZE, 1);
  if (!pmem) { fprintf(stderr, "npc: pmem alloc failed\n"); return 2; }

  size_t image_size = 0;
  if (image_path) {
    image_size = load_image(image_path);
    if (!image_size) { free(pmem); return 2; }
    fprintf(stderr, "npc: loaded %zu bytes from %s\n", image_size, image_path);
  } else {
    fprintf(stderr, "npc: no image provided (--image=...); running empty memory\n");
  }

  diff_init_so(diff_so);

  top = new Vcpu;
#ifdef VCD_TRACE
  if (tfp) { top->trace(tfp, 99); tfp->open(trace_path); }
#endif

  // reset for 5 cycles. clock_pulse() skips its MMIO sample while rst==1,
  // so no spurious skips get latched during these cycles.
  top->rst = 1;
  for (int i = 0; i < 5; ++i) clock_pulse();
  top->rst = 0;
  // One combinational settle after dropping rst so the snapshot we hand REF
  // reflects the post-reset register file (rf[*]=0, pc=PC_RESET_VEC).
  top->eval();
  diff_sync_from_dut(image_size);

  while (!Verilated::gotFinish() && !trap_hit && cycle_cnt < max_cycles) {
    clock_pulse();
    diff_step_and_check();
  }

  int rc;
  if (trap_hit) {
    if (trap_code == -2) {
      // diff_step_and_check already printed the per-reg diff lines above.
      printf("DIFFTEST: failed (mismatch at cycle %llu)\n",
             (unsigned long long)cycle_cnt);
      rc = 1;
    } else if (trap_code == 0) {
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

#ifdef VCD_TRACE
  if (tfp) { tfp->close(); delete tfp; tfp = nullptr; }
#endif
  delete top;
  if (diff_handle) dlclose(diff_handle);
  free(pmem);
  return rc;
}
