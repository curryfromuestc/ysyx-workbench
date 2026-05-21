#include <am.h>
#include <riscv/riscv.h>
#include <klib.h>

static Context* (*user_handler)(Event, Context*) = NULL;

Context* __am_irq_handle(Context *c) {
  if (user_handler) {
    Event ev = {0};
    // NEMU stores the ecall's a7 into mcause (see riscv32/inst.c). yield()
    // sets a7 = -1 to mark a YIELD; any other value is a real syscall number.
    switch (c->mcause) {
      case (uintptr_t)-1: ev.event = EVENT_YIELD; break;
      default:            ev.event = EVENT_SYSCALL; break;
    }

    // ecall saves the PC of the ecall instruction itself into mepc; advance
    // past it so mret returns to the next instruction.
    c->mepc += 4;

    c = user_handler(ev, c);
    assert(c != NULL);
  }

  return c;
}

extern void __am_asm_trap(void);

bool cte_init(Context*(*handler)(Event, Context*)) {
  // initialize exception entry
  asm volatile("csrw mtvec, %0" : : "r"(__am_asm_trap));

  // register event handler
  user_handler = handler;

  return true;
}

Context *kcontext(Area kstack, void (*entry)(void *), void *arg) {
  return NULL;
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
