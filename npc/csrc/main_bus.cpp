// =============================================================================
// NPC bus harness (B1): drive ysyx_22040000_bus.v with a host-side memory model
// that speaks the FULL SimpleBus handshake (reqValid/reqReady + respValid/
// respReady) and injects RANDOM delays into both phases.
//
// This is INDEPENDENT from main.cpp (D4/D5 DPI harness, top=cpu) and from
// main_soc.cpp (D6 ysyxSoC harness, top=ysyxSoCFull). It exists to validate
// the full-handshake bus FSM in isolation, with delay distributions that the
// real SoC bridge cannot expose.
//
// Memory model:
//   * Single 128 MiB region at 0x80000000 (matches the cpu-tests image layout
//     used by sim).
//   * Each port (IFU read, LSU read/write) is modeled as a 2-stage pipeline:
//       phase A: reqValid -> reqReady. Slave is "busy" for req_delay cycles,
//                then accepts the request. After that the request is
//                logically in-flight.
//       phase B: respValid -> respReady. Slave then needs resp_delay cycles
//                before respValid goes high. respValid stays high until
//                respReady is also high (handshake). After the handshake the
//                slave returns to idle.
//   * req_delay and resp_delay are drawn from a uniform distribution
//     [REQ_DELAY_MIN .. REQ_DELAY_MAX] and [RESP_DELAY_MIN .. RESP_DELAY_MAX]
//     respectively. The seed is fixed (--seed) so failures reproduce.
//
// Trap detection: the harness reads io_dbg_state and io_dbg_inst exposed by
// the RTL. When state == S_EX (== 2) and inst == ebreak (0x00100073), the
// CPU is parked on ebreak; we read a0 from the regfile and emit GOOD/BAD
// TRAP, identical to main.cpp.
// =============================================================================

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <verilated.h>
#include "Vysyx_22040000_bus.h"
#include "Vysyx_22040000_bus___024root.h"

#define PMEM_BASE 0x80000000u
#define PMEM_SIZE (128u * 1024u * 1024u)
#define PMEM_END  (PMEM_BASE + PMEM_SIZE)

// FSM state encoding mirrors ysyx_22040000_bus.v.
static constexpr uint8_t S_IF_REQ  = 0;
static constexpr uint8_t S_IF_WAIT = 1;
static constexpr uint8_t S_EX      = 2;
static constexpr uint8_t S_LS_REQ  = 3;
static constexpr uint8_t S_LS_WAIT = 4;
static constexpr uint8_t S_WB      = 5;

// ---- Host memory ------------------------------------------------------------
static uint8_t *pmem = nullptr;
static Vysyx_22040000_bus *top = nullptr;
static uint64_t cycle_cnt = 0;
static uint64_t max_cycles = 50000000ull;

static inline bool in_pmem(uint32_t a) {
  return a >= PMEM_BASE && a < PMEM_END;
}
static inline uint8_t *gh(uint32_t a) { return pmem + (a - PMEM_BASE); }

static uint32_t mem_read32(uint32_t addr) {
  uint32_t a = addr & ~3u;
  if (!in_pmem(a)) return 0;
  uint32_t w;
  memcpy(&w, gh(a), 4);
  return w;
}

static void mem_write32(uint32_t addr, uint32_t data, uint8_t mask) {
  uint32_t a = addr & ~3u;
  if (!in_pmem(a)) return;
  uint8_t *p = gh(a);
  for (int i = 0; i < 4; ++i) {
    if (mask & (1u << i)) p[i] = (uint8_t)((data >> (8 * i)) & 0xff);
  }
}

// ---- Random delay generator -------------------------------------------------
static std::mt19937 rng;
static int req_delay_min = 1, req_delay_max = 5;
static int resp_delay_min = 0, resp_delay_max = 8;

static int draw_req_delay() {
  std::uniform_int_distribution<int> d(req_delay_min, req_delay_max);
  return d(rng);
}
static int draw_resp_delay() {
  std::uniform_int_distribution<int> d(resp_delay_min, resp_delay_max);
  return d(rng);
}

// ---- Per-port slave FSM -----------------------------------------------------
// Each port is independent and tracks its own delay counters.
//
//   P_IDLE   : ready=0. When master asserts reqValid, we start counting down
//              req_delay cycles. When the counter hits 0, ready=1 for one
//              cycle (so the handshake completes), capture {addr, wen, ...},
//              and move to P_PROCESS.
//   P_PROCESS: ready=0, respValid=0. Count down resp_delay. When 0, perform
//              the read or write side-effect, latch rdata, move to P_RESPOND.
//   P_RESPOND: respValid=1 (rdata is on the wire). When master asserts
//              respReady (and we keep respValid=1), the handshake fires;
//              return to P_IDLE.
//
// Implementation note: the harness DRIVES the slave-side wires BEFORE
// `top->eval()` of the clk=0 phase, observes the master's req/resp signals
// AFTER eval, then updates internal counters before clk=1 phase.
struct Port {
  enum Phase { P_IDLE, P_PROCESS, P_RESPOND } phase = P_IDLE;
  int      req_cnt   = 0;          // cycles left until reqReady=1
  int      resp_cnt  = 0;          // cycles left until respValid=1
  uint32_t latched_addr  = 0;
  uint32_t latched_wdata = 0;
  uint8_t  latched_wmask = 0;
  bool     latched_wen   = false;
  uint32_t cached_rdata  = 0;

  // Outputs the master sees this cycle. Set in update_outputs(), consumed by
  // verilator on the next eval.
  bool reqReady   = false;
  bool respValid  = false;
  uint32_t rdata  = 0;

  // Compute what we drive to the master this cycle, based on phase only.
  void update_outputs() {
    reqReady  = false;
    respValid = false;
    switch (phase) {
      case P_IDLE:
        reqReady = (req_cnt == 0);
        break;
      case P_PROCESS:
        break;
      case P_RESPOND:
        respValid = true;
        rdata     = cached_rdata;
        break;
    }
  }

  // Step the slave state machine, given what the master is currently asserting.
  // Called once per simulated cycle (after both clk phases).
  void tick(bool reqValid, uint32_t addr, bool wen, uint32_t wdata,
            uint8_t wmask, bool respReady) {
    switch (phase) {
      case P_IDLE:
        if (req_cnt > 0) {
          // Hold ready=0 until the random req-delay counter expires. Only
          // decrement when the master is actually asserting reqValid: if no
          // request is pending there is nothing to delay.
          if (reqValid) req_cnt--;
        } else if (reqValid) {
          // req_cnt==0 -> reqReady was high this cycle, master had reqValid
          // high -> handshake fires.
          latched_addr  = addr;
          latched_wen   = wen;
          latched_wdata = wdata;
          latched_wmask = wmask;
          phase    = P_PROCESS;
          resp_cnt = draw_resp_delay();
          // Pre-draw the next req_delay for the request that will arrive
          // after we eventually return to P_IDLE.
          req_cnt  = draw_req_delay();
        }
        // else: req_cnt==0 and no request -- stay idle with reqReady=1.
        break;
      case P_PROCESS:
        if (resp_cnt > 0) {
          resp_cnt--;
        } else {
          // Perform the access now (one cycle before respValid goes high).
          // Writes ignore rdata on this bus, but we still latch zero so the
          // wire is deterministic.
          if (latched_wen) {
            mem_write32(latched_addr, latched_wdata, latched_wmask);
            cached_rdata = 0;
          } else {
            cached_rdata = mem_read32(latched_addr);
          }
          phase = P_RESPOND;
        }
        break;
      case P_RESPOND:
        if (respReady) phase = P_IDLE;
        break;
    }
  }
};

static Port ifu_port;
static Port lsu_port;

// ---- Image loader -----------------------------------------------------------
static size_t load_image(const char *path) {
  FILE *fp = fopen(path, "rb");
  if (!fp) { fprintf(stderr, "npc-bus: cannot open '%s'\n", path); return 0; }
  fseek(fp, 0, SEEK_END);
  size_t sz = (size_t)ftell(fp);
  fseek(fp, 0, SEEK_SET);
  if (sz > PMEM_SIZE) {
    fprintf(stderr, "npc-bus: image too large (%zu > %u)\n", sz, PMEM_SIZE);
    fclose(fp); return 0;
  }
  size_t rd = fread(pmem, 1, sz, fp);
  fclose(fp);
  return rd;
}

// ---- Clock driver -----------------------------------------------------------
// One cycle = clk falling-edge eval (so combinational outputs from the master
// are visible to us), then update slave-driven wires + clk rising-edge eval.
// The Port::tick() bookkeeping happens once at the end of the cycle, based on
// the master signals we observed post-eval.
static void apply_slave_outputs() {
  ifu_port.update_outputs();
  lsu_port.update_outputs();
  top->io_ifu_reqReady  = ifu_port.reqReady;
  top->io_ifu_respValid = ifu_port.respValid;
  top->io_ifu_rdata     = ifu_port.rdata;
  top->io_lsu_reqReady  = lsu_port.reqReady;
  top->io_lsu_respValid = lsu_port.respValid;
  top->io_lsu_rdata     = lsu_port.rdata;
}

static void clock_pulse() {
  // ---- low phase --------------------------------------------------------
  top->clock = 0;
  apply_slave_outputs();
  top->eval();
  // Capture master signals AFTER eval (so combinational outputs reflect the
  // current state). We use these for the slave tick after this clk edge.
  bool ifu_reqV     = top->io_ifu_reqValid;
  bool ifu_respR    = top->io_ifu_respReady;
  uint32_t ifu_addr = top->io_ifu_addr;
  bool lsu_reqV     = top->io_lsu_reqValid;
  bool lsu_respR    = top->io_lsu_respReady;
  uint32_t lsu_addr = top->io_lsu_addr;
  bool lsu_wen      = top->io_lsu_wen;
  uint32_t lsu_wdata= top->io_lsu_wdata;
  uint8_t  lsu_wmask= (uint8_t)(top->io_lsu_wmask & 0xf);

  // ---- high phase -------------------------------------------------------
  top->clock = 1;
  top->eval();

  // ---- end-of-cycle slave bookkeeping -----------------------------------
  ifu_port.tick(ifu_reqV, ifu_addr & ~3u, /*wen=*/false, 0, 0, ifu_respR);
  lsu_port.tick(lsu_reqV, lsu_addr & ~3u, lsu_wen, lsu_wdata, lsu_wmask, lsu_respR);
  cycle_cnt++;
}

// ---- Trap detection ---------------------------------------------------------
// We poll the RTL debug ports after each cycle. ebreak parks the FSM in S_EX
// (state==2) with inst_r==0x00100073. a0 is x10 in the regfile.
static bool trap_hit  = false;
static int  trap_code = -1;

static inline uint32_t rtl_state() { return top->rootp->ysyx_22040000_bus__DOT__state; }
static inline uint32_t rtl_inst()  { return top->rootp->ysyx_22040000_bus__DOT__inst_r; }
static inline uint32_t rtl_a0()    { return top->rootp->ysyx_22040000_bus__DOT__u_rf__DOT__rf[10]; }

static void check_trap() {
  if (rtl_state() == S_EX && rtl_inst() == 0x00100073u) {
    trap_hit  = true;
    trap_code = (int)rtl_a0();
  }
}

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);

  const char *image_path = nullptr;
  uint32_t seed = 0xdeadbeef;
  for (int i = 1; i < argc; ++i) {
    const char *a = argv[i];
    if (!strncmp(a, "--image=", 8))         image_path = a + 8;
    else if (!strncmp(a, "--seed=", 7))     seed = (uint32_t)strtoul(a + 7, nullptr, 0);
    else if (!strncmp(a, "--max-cycles=", 13))
      max_cycles = strtoull(a + 13, nullptr, 0);
    else if (!strncmp(a, "--req-min=", 10))  req_delay_min  = atoi(a + 10);
    else if (!strncmp(a, "--req-max=", 10))  req_delay_max  = atoi(a + 10);
    else if (!strncmp(a, "--resp-min=", 11)) resp_delay_min = atoi(a + 11);
    else if (!strncmp(a, "--resp-max=", 11)) resp_delay_max = atoi(a + 11);
    else if (a[0] != '-')                    image_path     = a;
  }

  if (req_delay_max < req_delay_min)   req_delay_max  = req_delay_min;
  if (resp_delay_max < resp_delay_min) resp_delay_max = resp_delay_min;

  rng.seed(seed);
  fprintf(stderr, "npc-bus: seed=0x%x req=[%d..%d] resp=[%d..%d]\n",
          seed, req_delay_min, req_delay_max, resp_delay_min, resp_delay_max);

  pmem = (uint8_t *)calloc(PMEM_SIZE, 1);
  if (!pmem) { fprintf(stderr, "npc-bus: pmem alloc failed\n"); return 2; }

  if (image_path) {
    size_t n = load_image(image_path);
    if (!n) { free(pmem); return 2; }
    fprintf(stderr, "npc-bus: loaded %zu bytes from %s\n", n, image_path);
  } else {
    fprintf(stderr, "npc-bus: no --image= given (running empty memory)\n");
  }

  top = new Vysyx_22040000_bus;
  // Initial slave state: both ports idle, reqReady picks the first req_delay.
  ifu_port.req_cnt = draw_req_delay();
  lsu_port.req_cnt = draw_req_delay();

  // Reset for a generous burst (10 cycles) so all internal regs settle.
  top->reset = 1;
  for (int i = 0; i < 10; ++i) clock_pulse();
  top->reset = 0;

  while (!Verilated::gotFinish() && !trap_hit && cycle_cnt < max_cycles) {
    clock_pulse();
    check_trap();
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
    printf("npc-bus: max cycles (%llu) reached without ebreak\n",
           (unsigned long long)max_cycles);
    rc = 1;
  }
  fprintf(stderr, "npc-bus: cycles=%llu\n", (unsigned long long)cycle_cnt);

  delete top;
  free(pmem);
  return rc;
}
