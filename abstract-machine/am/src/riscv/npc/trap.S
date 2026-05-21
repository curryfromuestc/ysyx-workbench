// NPC CTE trap entry (C5a). Mirrors the structure of the NEMU trap.S but:
//   - Compiles cleanly under -march=rv32e_zicsr -mabi=ilp32e (NR_REGS=16).
//   - Saves the OLD sp into the gpr[2] slot so kcontext/schedule can swap
//     stacks safely. trap.S in the NEMU port leaves gpr[2] uninitialised
//     which only happens to work because nothing reads it back.
//
// Stack layout (sp at trap entry == new sp after addi):
//   sp + 0           gpr[0]   (x0, padding; we don't store anything here)
//   sp + XLEN        gpr[1]   (ra)
//   sp + 2*XLEN      gpr[2]   (sp,  populated by hand below)
//   sp + 3*XLEN ...  gpr[3..NR_REGS-1]
//   sp + NR_REGS*XLEN          mcause
//   sp + (NR_REGS+1)*XLEN      mstatus
//   sp + (NR_REGS+2)*XLEN      mepc
//
// We do NOT touch x10 (a0) before the call: __am_irq_handle's argument is the
// Context*. We compute it via `mv a0, sp` right before the call.

#define concat_temp(x, y) x ## y
#define concat(x, y) concat_temp(x, y)
#define MAP(c, f) c(f)

#if __riscv_xlen == 32
#define LOAD  lw
#define STORE sw
#define XLEN  4
#else
#define LOAD  ld
#define STORE sd
#define XLEN  8
#endif

// x2 (sp) is handled separately so it picks up the OLD sp instead of the
// new (post-addi) sp.
#define REGS_LO16(f) \
      f( 1)       f( 3) f( 4) f( 5) f( 6) f( 7) f( 8) f( 9) \
f(10) f(11) f(12) f(13) f(14) f(15)
#ifndef __riscv_e
#define REGS_HI16(f) \
                                    f(16) f(17) f(18) f(19) \
f(20) f(21) f(22) f(23) f(24) f(25) f(26) f(27) f(28) f(29) \
f(30) f(31)
#define NR_REGS 32
#else
#define REGS_HI16(f)
#define NR_REGS 16
#endif

#define REGS(f) REGS_LO16(f) REGS_HI16(f)

#define PUSH(n) STORE concat(x, n), (n * XLEN)(sp);
#define POP(n)  LOAD  concat(x, n), (n * XLEN)(sp);

// Three trailing slots for mcause/mstatus/mepc; no pdir (PA4 only).
#define CONTEXT_SIZE  ((NR_REGS + 3) * XLEN)
#define OFFSET_SP     ( 2 * XLEN)
#define OFFSET_CAUSE  ((NR_REGS + 0) * XLEN)
#define OFFSET_STATUS ((NR_REGS + 1) * XLEN)
#define OFFSET_EPC    ((NR_REGS + 2) * XLEN)

.align 3
.globl __am_asm_trap
__am_asm_trap:
  addi sp, sp, -CONTEXT_SIZE

  MAP(REGS, PUSH)

  // Save the OLD sp (= new sp + CONTEXT_SIZE) into gpr[2].
  addi t0, sp, CONTEXT_SIZE
  STORE t0, OFFSET_SP(sp)

  csrr t0, mcause
  csrr t1, mstatus
  csrr t2, mepc

  STORE t0, OFFSET_CAUSE(sp)
  STORE t1, OFFSET_STATUS(sp)
  STORE t2, OFFSET_EPC(sp)

  mv a0, sp
  call __am_irq_handle

  // __am_irq_handle returns a Context* (possibly different stack). Switch sp
  // to point at the returned context, then pop in reverse order.
  mv sp, a0

  LOAD t1, OFFSET_STATUS(sp)
  LOAD t2, OFFSET_EPC(sp)
  csrw mstatus, t1
  csrw mepc, t2

  MAP(REGS, POP)
  // Restore sp last so the POPs above still see the trap frame.
  LOAD x2, OFFSET_SP(sp)

  mret
