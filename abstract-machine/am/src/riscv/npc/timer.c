// =============================================================================
// AM timer for the NPC target (D6c).
// =============================================================================
// Instead of reading the host-side RTC MMIO that the D5 harness exposes at
// 0xa0000048, we now read the `mcycle` CSR (0xb00). This is what runs on
// real silicon, so the implementation matches what we will eventually tape
// out. The DPI harness implements mcycle as a free-running counter
// incremented every clock cycle (see `npc/vsrc/csr.v`).
//
// `MCYCLE_PER_US` converts mcycle counts to microseconds. In simulation,
// one cycle is NOT a fixed wall-clock duration -- a verilator host typically
// runs the SoC at ~100k--1M cycles per second of wall time, which is many
// orders of magnitude slower than a real ~100 MHz CPU. We empirically choose
// a divisor that makes `am-tests`'s real-time-clock test print roughly once
// per wall-clock second (see d/6.md "运行时钟测试").
//
// The chosen value (1 cycle == 1 us) makes the simulated test count by
// cycles directly. For a 100 MHz target you would set this to 100 instead.
// =============================================================================

#include <am.h>
#include <npc.h>
#include "../riscv.h"

// Number of mcycle counts per simulated microsecond. Tuned so rtc-test prints
// at roughly real-time pace in the verilator harness (see d/6.md step
// "运行时钟测试").
// Empirically: this verilator+SoC harness runs at ~4 million CPU cycles per
// wall-clock second on a typical host (most cycles are LSU waits on SPI
// flash / SDRAM). MCYCLE_PER_US=4 makes rtc-test print at roughly one
// second per wall-clock second on the ysyxSoC simulator. The D4/D5 DPI
// harness runs ~100x faster, so on that harness sim seconds will fly by --
// that is fine because dummy/cpu-tests do not depend on rtc pacing.
#ifndef MCYCLE_PER_US
#define MCYCLE_PER_US 4u
#endif

void __am_timer_init(void) {
}

static inline uint32_t read_mcycle(void) {
  uint32_t v;
  // CSR 0xb00 == mcycle (low 32 bits). The wrapper CPU (`ysyx_22040000.v`)
  // and the DPI harness top (`cpu.v`) both implement it as a free-running
  // counter that wraps mod 2^32.
  asm volatile("csrr %0, mcycle" : "=r"(v));
  return v;
}

void __am_timer_uptime(AM_TIMER_UPTIME_T *uptime) {
  // mcycle is a 64-bit counter on RV64 but only 32 bits on RV32E. We track
  // wrap-around in software: every time the low half goes backwards, bump a
  // host-side high half. This is good enough for am-tests which only reads
  // monotonically increasing values; the simulator never runs long enough
  // for a 64-bit wrap to matter.
  static uint32_t last_lo = 0;
  static uint32_t hi      = 0;
  uint32_t lo = read_mcycle();
  if (lo < last_lo) hi++;
  last_lo = lo;
  uint64_t cycles = ((uint64_t)hi << 32) | lo;
  uptime->us = cycles / MCYCLE_PER_US;
}

void __am_timer_rtc(AM_TIMER_RTC_T *rtc) {
  rtc->second = 0;
  rtc->minute = 0;
  rtc->hour   = 0;
  rtc->day    = 0;
  rtc->month  = 0;
  rtc->year   = 1900;
}
