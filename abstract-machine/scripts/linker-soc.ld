/* =========================================================================
 * B2c link script: NPC-on-ysyxSoC -- XIP from Flash, RAM in SDRAM.
 * =========================================================================
 *
 * Memory map (see B2b report and ysyxSoCFull.v APBFanout decode):
 *
 *   flash   : 0x30000000-0x30ffffff  16 MiB, R/O  -- XIP for .text/.rodata
 *                                     CPU reset PC is here.
 *   sdram   : 0x80000000-0x81ffffff  32 MiB, R/W -- backing for .data/.bss
 *                                     /stack/heap. (The SoC top routes the
 *                                     entire 0x80000000-0x9fffffff window
 *                                     to sdramChisel.) We carve only the
 *                                     first 32 MiB because that is what
 *                                     the controller's address window
 *                                     exposes; the rest mirrors.
 *
 * Linkage rule:
 *   - .text  -> flash@VMA, flash@LMA            (run-in-place)
 *   - .rodata-> flash@VMA, flash@LMA            (run-in-place, with .text)
 *   - .data  -> sdram@VMA, flash@LMA            (copied at boot by _start)
 *   - .bss   -> sdram@VMA, no LMA               (zeroed at boot by _start)
 *
 * Symbols exposed to start.S:
 *   _flash_start    LMA of .text (= 0x30000000)
 *   _data_lma       LMA of .data section
 *   _data_vma       VMA of .data section
 *   _data_size      bytes in .data
 *   _bss_start      VMA of .bss start
 *   _bss_end        VMA of .bss end
 *   _stack_pointer  initial stack top (high end of stack reservation)
 *   _heap_start     start of heap (after stack)
 *
 * IMPORTANT: minirv-gcc / riscv32-unknown-elf-gcc assumes 4-byte alignment
 * on most accesses (mstrict-align in COMMON_CFLAGS). All section boundaries
 * therefore get an explicit `. = ALIGN(4);`.
 * =========================================================================
 */

ENTRY(_start)

MEMORY {
  flash (rx)  : ORIGIN = 0x30000000, LENGTH = 16M
  sdram (rwx) : ORIGIN = 0x80000000, LENGTH = 32M
}

PHDRS {
  text PT_LOAD FLAGS(5);   /* RX */
  data PT_LOAD FLAGS(6);   /* RW */
  bss  PT_LOAD FLAGS(6);   /* RW (zero-fill, but make LOAD so OBJCOPY lays
                              out the file image correctly -- we never read
                              .bss from flash, but a LOAD phdr keeps the
                              memory image contiguous in tools). */
}

SECTIONS {
  /* ---------------------- FLASH side (XIP) ----------------------------- */
  . = ORIGIN(flash);
  _flash_start = .;

  .text : ALIGN(4) {
    *(entry)         /* _start lives in `entry` section (start.S) */
    *(.text*)
  } > flash :text

  .rodata : ALIGN(4) {
    *(.rodata*)
    *(.srodata*)
  } > flash :text

  . = ALIGN(4);
  _etext = .;
  etext = .;

  /* ---------------------- SDRAM side (R/W) ----------------------------- */
  /*
   * Place .data with VMA in SDRAM and LMA in Flash. The `>sdram AT>flash`
   * idiom tells the linker to:
   *   - resolve symbols inside .data against the SDRAM VMA (so the running
   *     code reads/writes the bytes from SDRAM)
   *   - emit the initial bytes into the flash image (so OBJCOPY -O binary
   *     puts them inside the .bin we burn into flash[0..])
   *
   * _start then copies _data_size bytes from _data_lma to _data_vma during
   * boot before calling main().
   */
  .data : ALIGN(4) {
    _data_vma = .;
    *(.data*)
    *(.sdata*)
    . = ALIGN(4);
    _data_end = .;
  } > sdram AT> flash :data

  _data_lma  = LOADADDR(.data);
  _data_size = SIZEOF(.data);

  .bss : ALIGN(4) {
    _bss_start = .;
    *(.bss*)
    *(.sbss*)
    *(.scommon)
    *(COMMON)
    . = ALIGN(4);
    _bss_end = .;
  } > sdram :bss

  _edata = _data_end;

  /* ---------------------- stack + heap (SDRAM) ------------------------- */
  /*
   * Layout in SDRAM, low -> high:
   *   _bss_end .. _heap_start         alignment slack
   *   _heap_start .. _stack_bottom    heap (grows up)
   *   _stack_bottom .. _stack_pointer stack (grows down)
   *
   * We reserve 128 KiB stack at the high end of SDRAM (well above any
   * microbench need; even queen depth-8 recursion fits in ~2 KiB).
   *
   * Heap is everything between _heap_start and _stack_bottom -- microbench
   * `mlim` for the `test` setting maxes out at ~256 KiB, but we leave the
   * full SDRAM tail (about 31 MiB) addressable.
   */
  . = ALIGN(16);
  _heap_start = .;

  _sdram_end    = ORIGIN(sdram) + LENGTH(sdram);
  _stack_pointer = _sdram_end;                /* high end -- sp initial    */
  _stack_bottom  = _sdram_end - 0x20000;      /* 128 KiB stack reservation */

  end = .;
  _end = .;
}
