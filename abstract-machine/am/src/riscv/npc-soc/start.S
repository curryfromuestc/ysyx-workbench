/* =========================================================================
 * B2c boot sequence for NPC-on-ysyxSoC.
 * =========================================================================
 *
 * CPU reset PC is 0x30000000 (Flash). The byte at that address must be the
 * first instruction of `_start` (placed by linker via *(entry)).
 *
 * Boot sequence:
 *   1. Set sp = _stack_pointer (high end of SDRAM).
 *      Stack must be valid BEFORE the .data copy / .bss zero loops use
 *      any local storage. We use only registers in those loops, but having
 *      sp pointing at SDRAM also means a future debugger can stash things
 *      safely.
 *
 *      Note: setting sp must use `lui+addi` (la pseudo) because the SP
 *      symbol is in the SDRAM range (0x82000000) and cannot reach via PC-
 *      relative addressing from 0x30000000.
 *
 *   2. memcpy .data: copy [_data_lma, _data_lma + _data_size) over to
 *      [_data_vma, _data_vma + _data_size). The Flash side is read via
 *      32-bit aligned lw -- the linker .ld file guarantees _data_lma and
 *      _data_vma are 4-byte aligned. _data_size is also rounded up to 4.
 *
 *   3. zero .bss: write 0 to [_bss_start, _bss_end).
 *
 *   4. clear s0 (mirror NEMU start.S; frame pointer convention).
 *
 *   5. jal _trm_init -- never returns (_trm_init eventually halts via
 *      ebreak with a0 = main return code).
 *
 * Registers used:
 *   t0..t3 are caller-saved; we trample them freely.
 *   sp is initialised early so any future trap can save a context there.
 *
 * Why not use libc memcpy here? klib's memcpy lives in .text (in flash) so
 * we could legally call it, BUT it might pull in libgcc helpers that have
 * not been linked yet (PC-relative jumps to flash addresses just work, but
 * having start.S self-contained reduces blast radius if klib changes).
 * Loop is 6 instructions; not worth a function call.
 * =========================================================================
 */

.section entry, "ax"
.globl _start
.type _start, @function

_start:
  /* --- Step 1: set stack pointer ----------------------------------------
   * `la sp, _stack_pointer` expands to `lui + addi`, which gives a 32-bit
   * absolute immediate (good for any SDRAM address).
   */
  la    sp, _stack_pointer

  /* --- Step 2: copy .data (.data LMA in flash) --> (.data VMA in SDRAM)
   * RV32E only has registers x0..x15. We use:
   *   t0 (x5) = src cursor  (Flash LMA)
   *   t1 (x6) = dst cursor  (SDRAM VMA)
   *   t2 (x7) = end-of-dst  (VMA + size)
   *   a5 (x15) = scratch (loaded word) -- highest-numbered E-legal reg.
   * a5 is a temporary caller-saved register; we are pre-main so no caller
   * cares about its value.
   * Skip the loop entirely when size==0 to avoid one unaligned tail read.
   */
  la    t0, _data_lma
  la    t1, _data_vma
  la    t2, _data_size
  add   t2, t1, t2
  beq   t1, t2, .Ldata_done
.Ldata_loop:
  lw    a5, 0(t0)
  sw    a5, 0(t1)
  addi  t0, t0, 4
  addi  t1, t1, 4
  bne   t1, t2, .Ldata_loop
.Ldata_done:

  /* --- Step 3: zero .bss ------------------------------------------------
   * t0 = cursor (.bss start)
   * t1 = end    (.bss end)
   * Linker pads both to 4-byte alignment.
   */
  la    t0, _bss_start
  la    t1, _bss_end
  beq   t0, t1, .Lbss_done
.Lbss_loop:
  sw    zero, 0(t0)
  addi  t0, t0, 4
  bne   t0, t1, .Lbss_loop
.Lbss_done:

  /* --- Step 4: clear frame pointer (RV ABI cosmetic) -------------------- */
  mv    s0, zero

  /* --- Step 5: jump into AM TRM ----------------------------------------- */
  call  _trm_init

  /* _trm_init should not return; spin on safety if it does. */
.Lhang:
  j     .Lhang

.size _start, . - _start
