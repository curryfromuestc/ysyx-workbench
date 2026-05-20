CC      ?= gcc
CFLAGS  ?= -Wall -Wextra -O2 -std=c11

RVCC    ?= riscv32-unknown-elf-gcc
RVOD    ?= riscv32-unknown-elf-objdump
RVFLAGS ?= -O0 -std=c11 -ffreestanding -fno-builtin

c_basics: c_basics.c
	$(CC) $(CFLAGS) -o $@ $<

hello.i: hello.c
	$(RVCC) $(RVFLAGS) -E $< -o $@

hello.s: hello.c
	$(RVCC) $(RVFLAGS) -S $< -o $@

hello.o: hello.c
	$(RVCC) $(RVFLAGS) -c $< -o $@

# 裸机: 无 crt0, 直接以 main 作入口符号.
hello.elf: hello.o
	$(RVCC) $(RVFLAGS) -nostartfiles -nostdlib -Wl,-e,main $< -o $@

hello.dis: hello.elf
	$(RVOD) -d hello.elf > $@

# E4b: 链接器脚本 + crt0 启动文件 -> 提供 _start 入口的完整裸机 ELF.
crt0.o: crt0.S
	$(RVCC) $(RVFLAGS) -c $< -o $@

hello_bare.elf: hello.o crt0.o linker.ld
	$(RVCC) $(RVFLAGS) -nostartfiles -nostdlib -T linker.ld \
		crt0.o hello.o -o $@

hello_bare.dis: hello_bare.elf
	$(RVOD) -d hello_bare.elf > $@

.PHONY: pipeline bare clean
pipeline: hello.i hello.s hello.o hello.elf hello.dis
bare: hello_bare.elf hello_bare.dis

clean:
	rm -f c_basics hello.i hello.s hello.o hello.elf hello.dis \
		crt0.o hello_bare.elf hello_bare.dis
