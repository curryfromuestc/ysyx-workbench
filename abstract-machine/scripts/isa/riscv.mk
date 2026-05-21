CROSS_COMPILE := riscv32-unknown-elf-
COMMON_CFLAGS := -fno-pic -march=rv32im -mabi=ilp32 -mcmodel=medany -mstrict-align
CFLAGS        += $(COMMON_CFLAGS) -static
ASFLAGS       += $(COMMON_CFLAGS) -O0
LDFLAGS       += -melf32lriscv

# overwrite ARCH_H defined in $(AM_HOME)/Makefile
ARCH_H := arch/riscv.h
