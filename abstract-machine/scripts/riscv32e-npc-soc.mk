include $(AM_HOME)/scripts/isa/riscv.mk
include $(AM_HOME)/scripts/platform/npc-soc.mk
COMMON_CFLAGS += -march=rv32e_zicsr -mabi=ilp32e  # overwrite
LDFLAGS       += -melf32lriscv                    # overwrite

# Software libgcc -- the host RISC-V toolchain ships libgcc.a built against
# rv32imc/ilp32, not against rv32e/ilp32e, so we can't link the system one.
# These files (also used by the `riscv32e-npc` platform) implement just
# enough of the helpers (mul/div/shift) for our microbenchmarks.
AM_SRCS += riscv/npc/libgcc/div.S \
           riscv/npc/libgcc/muldi3.S \
           riscv/npc/libgcc/multi3.c \
           riscv/npc/libgcc/ashldi3.c \
           riscv/npc/libgcc/unused.c
