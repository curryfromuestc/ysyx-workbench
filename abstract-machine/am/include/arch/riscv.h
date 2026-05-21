#ifndef ARCH_H__
#define ARCH_H__

#ifdef __riscv_e
#define NR_REGS 16
#else
#define NR_REGS 32
#endif

struct Context {
  // Members must match the order trap.S pushes onto the stack:
  //   sp + 0                 -> gpr[0..NR_REGS-1]
  //   sp + NR_REGS*XLEN      -> mcause
  //   sp + (NR_REGS+1)*XLEN  -> mstatus
  //   sp + (NR_REGS+2)*XLEN  -> mepc
  // pdir is for PA4 address-space switching; it lives beyond the trap-saved
  // window and is populated by kcontext(), not by trap.S.
  uintptr_t gpr[NR_REGS];
  uintptr_t mcause;
  uintptr_t mstatus;
  uintptr_t mepc;
  void *pdir;
};

// libos/syscall.c uses (a7, a0, a1, a2, a0) for (type, arg0, arg1, arg2, ret)
// — or (a5, a0, a1, a2, a0) under RVE where a7/a5 swap roles for the syscall
// number. The Context indexes below must mirror that, otherwise do_syscall
// reads garbage (e.g. 0x5a5a5a5a stack-init bytes) for the arg slots.
#ifdef __riscv_e
#define GPR1 gpr[15] // a5 — syscall number
#else
#define GPR1 gpr[17] // a7 — syscall number
#endif

#define GPR2 gpr[10] // a0 — arg 0
#define GPR3 gpr[11] // a1 — arg 1
#define GPR4 gpr[12] // a2 — arg 2
#define GPRx gpr[10] // a0 — return value

#endif
