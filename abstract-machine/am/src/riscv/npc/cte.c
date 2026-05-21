// C5a: CTE for the NPC target.
//
// Trap path:
//   * yield() issues `li a5, -1; ecall` under ilp32e.
//   * NPC IDU asserts is_ecall on the ecall instruction.
//   * CSR file latches mcause <= a5 (== -1 for yield, syscall number otherwise)
//     and mepc <= pc.
//   * WBU redirects PC to mtvec, which `cte_init` programs with __am_asm_trap.
//   * trap.S saves the full context, calls __am_irq_handle, restores, mret.
//
// kcontext(): build a minimal Context in the user-supplied kstack so the next
// __am_asm_trap restore path lands inside `entry` with sp = kstack.end and
// a0 = arg. The user handler returns this Context to swap stacks.

#include <am.h>
#include <riscv/riscv.h>
#include <klib.h>

static Context* (*user_handler)(Event, Context*) = NULL;

Context* __am_irq_handle(Context *c) {
  if (user_handler) {
    Event ev = {0};
    // Mirror NEMU's `mcause = a5` semantic: -1 means YIELD, anything else is
    // a real syscall number (forwarded as EVENT_SYSCALL).
    switch (c->mcause) {
      case (uintptr_t)-1: ev.event = EVENT_YIELD; break;
      default:            ev.event = EVENT_SYSCALL; break;
    }

    // Advance past the ecall instruction so mret resumes at ecall+4. Failing
    // to do this would re-trap on the same ecall forever.
    c->mepc += 4;

    c = user_handler(ev, c);
    assert(c != NULL);
  }

  return c;
}

extern void __am_asm_trap(void);

bool cte_init(Context*(*handler)(Event, Context*)) {
  // Program the exception entry address into mtvec. WBU reads mtvec on every
  // trap to determine the next PC.
  asm volatile("csrw mtvec, %0" : : "r"(__am_asm_trap));

  // Register the user-supplied dispatcher (e.g. yield-os' schedule()).
  user_handler = handler;

  return true;
}

Context *kcontext(Area kstack, void (*entry)(void *), void *arg) {
  // Place the Context at the very top of kstack so all kstack bytes below it
  // are free for the kernel thread's working stack. trap.S restores sp from
  // gpr[2] last, so we point gpr[2] at kstack.end (the byte just past the
  // Context, which IS the new stack top under RV's downward-growing stack).
  Context *ctx = (Context *)((uintptr_t)kstack.end - sizeof(Context));

  // Zero everything we don't explicitly set. Avoids landing into random
  // garbage if the user handler ever reads e.g. gpr[5] or pdir.
  for (int i = 0; i < NR_REGS; i++) ctx->gpr[i] = 0;
  ctx->mcause  = 0;
  // mstatus = 0x1800 keeps the previous-privilege bits in M-mode and is
  // what NEMU's REF expects on a fresh context (see ics-pa/4.1.md DiffTest
  // section).
  ctx->mstatus = 0x1800;
  ctx->mepc    = (uintptr_t)entry;
  ctx->pdir    = NULL;

  // Entry-function calling convention: arg is the first argument -> a0/x10.
  // The new thread will also need a valid sp once execution begins. We set
  // sp = kstack.end so the kernel thread has the full kstack minus the
  // context for its own stack growth.
  ctx->gpr[10] = (uintptr_t)arg;            // a0
  ctx->gpr[2]  = (uintptr_t)kstack.end;     // sp

  return ctx;
}

void yield() {
#ifdef __riscv_e
  asm volatile("li a5, -1; ecall");
#else
  asm volatile("li a7, -1; ecall");
#endif
}

bool ienabled() {
  return false;
}

void iset(bool enable) {
}
