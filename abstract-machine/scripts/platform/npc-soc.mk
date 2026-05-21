# =========================================================================
# AM platform: npc-soc (B2c).
# =========================================================================
# This platform targets the verilated ysyxSoC harness (`make sim-soc`).
# Differences from the original `npc` platform (DPI-C harness):
#   - Uses linker-soc.ld which splits flash (XIP for .text/.rodata) and
#     SDRAM (R/W for .data/.bss/stack/heap).
#   - Uses a custom start.S that copies .data from flash to SDRAM and zeroes
#     .bss before calling _trm_init.
#   - Boot bin is loaded into the SoC's SPI flash, not into PSRAM.
#
# We still share trm.c / cte.c / trap.S / timer.c / input.c / ioe.c with the
# DPI-C `npc` platform, because the runtime layer (UART putchar, mcycle
# uptime, ecall trap path) is identical -- only the link-time memory map
# and the boot-time data copy differ.
#
# Build rules (image, insert-arg, run) mirror npc.mk but the resulting .bin
# is consumed by `make -C $(NPC_HOME) sim-soc FLASH=...`. The IMAGE_REL.bin
# already contains the bootloader (the entry section in start.S sits at
# 0x30000000 in the file image), so we hand it straight to the SoC --
# no vendor gen.sh wrapping required.
# =========================================================================

AM_SRCS := riscv/npc-soc/start.S \
           riscv/npc/trm.c \
           riscv/npc/ioe.c \
           riscv/npc/timer.c \
           riscv/npc/input.c \
           riscv/npc/cte.c \
           riscv/npc/trap.S \
           platform/dummy/vme.c \
           platform/dummy/mpe.c

CFLAGS    += -fdata-sections -ffunction-sections
CFLAGS    += -I$(AM_HOME)/am/src/platform/npc/include

# Host riscv64-linux-gnu cross toolchain may need the rv32e stub shim
# (same as npc.mk). NPC_HOME falls back to the sibling tree.
NPC_HOME ?= $(abspath $(AM_HOME)/../npc)
CFLAGS    += -isystem $(NPC_HOME)/tools/sysinc
ASFLAGS   += -isystem $(NPC_HOME)/tools/sysinc

LDSCRIPTS += $(AM_HOME)/scripts/linker-soc.ld
LDFLAGS   += --gc-sections -e _start

# trm.c references `_pmem_start` for klib's heap.end (PMEM_END = _pmem_start +
# 128 MiB). The linker-soc.ld script does not export this symbol because
# our heap lives between _heap_start (SDRAM low) and _stack_bottom (SDRAM
# high - 128 KiB stack). Provide a defsym so the unresolved reference in
# trm.c resolves to the SDRAM base, then we will clamp PMEM_END in trm.c
# via a separate symbol if needed. With SDRAM = 32 MiB, _pmem_start +
# 128 MiB overshoots SDRAM, but the heap top is actually bounded by
# `_stack_bottom` (a separate linker symbol). klib's Area heap = RANGE(
# _heap_start, PMEM_END) would let malloc grow into the stack -- but
# microbench's `test` setting only needs ~256 KiB heap, way below the
# 32 MiB - 128 KiB safe window, so this is benign for B2c.
LDFLAGS   += --defsym=_pmem_start=0x80000000

# Use the same MAINARGS pattern as the other platforms: a fixed placeholder
# string is compiled into the image (`mainargs[]` in trm.c), and the
# `insert-arg` rule patches the actual mainargs into the .bin AFTER ld+objcopy
# completes. The placeholder lives in .rodata (flash) so the patcher needs to
# find the right offset; it scans for the unique string at runtime.
MAINARGS_MAX_LEN = 64
MAINARGS_PLACEHOLDER = the_insert-arg_rule_in_Makefile_will_insert_mainargs_here
CFLAGS += -DMAINARGS_MAX_LEN=$(MAINARGS_MAX_LEN) -DMAINARGS_PLACEHOLDER=$(MAINARGS_PLACEHOLDER)

insert-arg: image
	@python $(AM_HOME)/tools/insert-arg.py $(IMAGE).bin $(MAINARGS_MAX_LEN) $(MAINARGS_PLACEHOLDER) "$(mainargs)"

image: image-dep
	@$(OBJDUMP) -d $(IMAGE).elf > $(IMAGE).txt
	@echo + OBJCOPY "->" $(IMAGE_REL).bin
	@$(OBJCOPY) -S --set-section-flags .bss=alloc,contents -O binary $(IMAGE).elf $(IMAGE).bin

# `make ARCH=riscv32e-npc-soc run` -> hand the binary straight to npc-soc.
# Callers wanting a non-default cycle budget can pass MAX_CYCLES on the
# am-kernel command line: `make ARCH=... run MAX_CYCLES=5000000000`.
NPC_SOC_MAX_CYCLES ?= 3000000000
run: insert-arg
	@$(MAKE) -s -C $(NPC_HOME) sim-soc FLASH=$(IMAGE).bin MAX_CYCLES=$(NPC_SOC_MAX_CYCLES)

.PHONY: insert-arg
