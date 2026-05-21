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

  // B5a: NPC 现在是 5 段流水线 (ysyx_22040000.v). 没有多周期 FSM 的 state
  // 信号了; 改用 ebreak_park sticky bit: WB 段一旦看到 ebreak, ebreak_park
  // 拉高并锁住整条流水线. host 检测 ebreak_park=1 即可稳定捕获 ebreak.
  // 同时把 mem_wb_inst 当作 inst_r 的对应物用于 sanity check (= 0x00100073).
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
  const char *pc_trace_env = getenv("PC_TRACE_EVERY");
  uint64_t pc_trace_every = pc_trace_env ? strtoull(pc_trace_env, nullptr, 0) : 0;
  const char *jalr_dump_env = getenv("JALR_DUMP");
  bool jalr_dump = jalr_dump_env && *jalr_dump_env == '1';
  int jalr_dumps_left = 20;
  while (!Verilated::gotFinish() && cycle_cnt < max_cycles) {
    clock_pulse();
    if (r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__any_fault) {
      ++fault_events;
    }
    if (jalr_dump && jalr_dumps_left > 0) {
      uint32_t id_ex_is_jalr = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__id_ex_is_jalr;
      uint32_t id_ex_valid   = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__id_ex_valid;
      if (id_ex_is_jalr && id_ex_valid) {
        uint32_t id_ex_pc   = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__id_ex_pc;
        uint32_t id_ex_rs1  = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__id_ex_rs1_val;
        uint32_t id_ex_imm  = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__id_ex_imm;
        uint32_t id_ex_rd   = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__id_ex_rd;
        uint32_t ifid_pc    = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__if_id_pc;
        uint32_t ifid_inst  = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__if_id_inst;
        uint32_t ra_now     = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__u_rf__DOT__rf[1];
        fprintf(stderr, "[JALR_EX] cyc=%llu id_ex_pc=%08x rs1_val=%08x imm=%08x rd=%u ifid_pc=%08x ifid_inst=%08x rf_ra=%08x\n",
                (unsigned long long)cycle_cnt, id_ex_pc, id_ex_rs1, id_ex_imm, id_ex_rd, ifid_pc, ifid_inst, ra_now);
        --jalr_dumps_left;
      }
    }
    if (pc_trace_every && (cycle_cnt % pc_trace_every) == 0) {
      uint32_t pc       = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__pc;
      uint32_t wb_pc    = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__mem_wb_pc;
      uint32_t wb_valid = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__mem_wb_valid;
      uint32_t pf       = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__pipe_freeze;
      uint32_t hz       = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__id_hazard;
      uint32_t irv      = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__ifu_cpu_resp_valid;
      uint32_t exv      = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__ex_mem_valid;
      uint32_t exre     = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__ex_mem_mem_re;
      uint32_t exwe     = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__ex_mem_mem_we;
      uint32_t lrv      = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__io_lsu_respValid;
      uint64_t ica = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__u_icache__DOT__cnt_access;
      uint64_t ich = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__u_icache__DOT__cnt_hit;
      uint64_t icm = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__u_icache__DOT__cnt_miss;
      uint32_t icst = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__u_icache__DOT__state;
      uint32_t brv  = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT___bridge_io_ifu_respValid;
      uint32_t bsi  = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__bridge__DOT__stateI;
      uint32_t bsd  = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__bridge__DOT__stateD;
      fprintf(stderr, "[PC] cyc=%llu pc=%08x wb_v=%u wb_pc=%08x | pf=%u hz=%u irv=%u | exv=%u exre=%u exwe=%u lrv=%u | icA=%llu icH=%llu icM=%llu icST=%u brv=%u bsI=%u bsD=%u\n",
              (unsigned long long)cycle_cnt, pc, wb_valid, wb_pc, pf, hz, irv, exv, exre, exwe, lrv,
              (unsigned long long)ica, (unsigned long long)ich, (unsigned long long)icm, icst, brv, bsi, bsd);
    }
    if (r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__ebreak_park) {
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

  // B4a/B4b/B4c: 退出前打印 icache 性能计数器 (verilator 通过 public_flat_rd
  // 把它们抬到 root scope 的扁平符号表里). cnt_victim 理论上 == cnt_miss,
  // 是 LRU 选 victim 的 sanity check.
  // B4c 加打印一行 cycle 数 + cycles-per-access, 方便对比不同块大小配置.
  uint64_t ic_access = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__u_icache__DOT__cnt_access;
  uint64_t ic_hit    = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__u_icache__DOT__cnt_hit;
  uint64_t ic_miss   = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__u_icache__DOT__cnt_miss;
  uint64_t ic_victim = r->ysyxSoCFull__DOT__asic__DOT__cpu__DOT__cpu__DOT__u_icache__DOT__cnt_victim;
  double hit_rate = (ic_access == 0) ? 0.0 : (double)ic_hit * 100.0 / (double)ic_access;
  fprintf(stderr, "npc-soc: icache access=%llu hit=%llu miss=%llu hit_rate=%.2f%% victim=%llu\n",
          (unsigned long long)ic_access,
          (unsigned long long)ic_hit,
          (unsigned long long)ic_miss,
          hit_rate,
          (unsigned long long)ic_victim);
  fprintf(stderr, "npc-soc: total_cycles=%llu access/miss/hit ratio: %.3f cyc/access\n",
          (unsigned long long)cycle_cnt,
          (ic_access == 0) ? 0.0 : (double)cycle_cnt / (double)ic_access);

  delete top;
  return 0;
}
