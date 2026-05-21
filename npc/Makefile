# NPC: minirv single-cycle CPU, verilated.
#
# Targets:
#   sim       verilate + compile the D4/D5 DPI harness (top=cpu)
#             -> build/npc; run with ARGS=... (or default img)
#   sim-soc   verilate + compile the D6 ysyxSoC harness (top=ysyxSoCFull)
#             -> build/npc-soc; run with SOC_ARGS=... (defaults to a few k cycles)
#   clean     wipe build/

TOPDIR    := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
BUILD_DIR := $(TOPDIR)/build

# Provide NEMU_HOME for ../Makefile's git_commit hook (it computes
# YSYX_HOME = $(NEMU_HOME)/..). NEMU_HOME may come from the environment;
# default to the sibling nemu directory in the workbench tree.
NEMU_HOME ?= $(abspath $(TOPDIR)/../nemu)
export NEMU_HOME

# ---- D4/D5 (DPI-C) harness --------------------------------------------------
OBJ_DIR   := $(BUILD_DIR)/obj_dir
BIN       := $(BUILD_DIR)/npc

# Only the cpu-side RTL (no SoC top, no SoC wrapper).
VSRCS     := $(TOPDIR)/vsrc/cpu.v \
             $(TOPDIR)/vsrc/csr.v \
             $(TOPDIR)/vsrc/exu.v \
             $(TOPDIR)/vsrc/idu.v \
             $(TOPDIR)/vsrc/ifu.v \
             $(TOPDIR)/vsrc/lsu.v \
             $(TOPDIR)/vsrc/regfile.v \
             $(TOPDIR)/vsrc/wbu.v
CSRCS     := $(TOPDIR)/csrc/main.cpp
TOP_MOD   := cpu

VFLAGS    := --cc --exe --build --top-module $(TOP_MOD) \
             -Wno-fatal -Wno-WIDTH -Wno-UNUSED -Wno-UNOPTFLAT \
             -Mdir $(OBJ_DIR) -o $(BIN) \
             -CFLAGS "-O2 -std=c++17" -LDFLAGS "-ldl"

# ARGS may be passed in by the difftest runner: ARGS="--diff=... --image=..."
ARGS ?=

# Default image used when ARGS is empty. Built from a tiny self-test program
# so that `make sim` produces something runnable.
DEFAULT_IMG := $(BUILD_DIR)/cpu-tests.bin

all: sim

# Workaround: the installed verilator embeds PYTHON3=/usr/bin/python3.10 in
# verilated.mk; pick an available python3 at build time and pass it through to
# the sub-make that verilator triggers.
PYTHON3 := $(shell command -v python3.12 || command -v python3.11 || command -v python3 || echo /usr/bin/python3)

$(BIN): $(VSRCS) $(CSRCS)
	$(call git_commit, "sim RTL") # DO NOT REMOVE THIS LINE!!!
	@mkdir -p $(BUILD_DIR)
	verilator $(VFLAGS) -MAKEFLAGS "PYTHON3=$(PYTHON3)" $(VSRCS) $(CSRCS)

$(DEFAULT_IMG): $(TOPDIR)/tools/mkdummy.sh
	@mkdir -p $(BUILD_DIR)
	bash $(TOPDIR)/tools/mkdummy.sh $@

sim: $(BIN) $(DEFAULT_IMG)
	@if [ -n "$(ARGS)" ]; then \
	  echo "==> $(BIN) $(ARGS)"; $(BIN) $(ARGS); \
	else \
	  echo "==> $(BIN) --image=$(DEFAULT_IMG)"; $(BIN) --image=$(DEFAULT_IMG); \
	fi

# ---- C3 sim with VCD wave dump (for gtkwave) --------------------------------
# Same harness as `sim`, but with --trace + -DVCD_TRACE so the main loop
# emits build/wave.vcd. Default image still cpu-tests.bin; pass FILE= to
# override the dump path.
TRACE_BIN  := $(BUILD_DIR)/npc-trace
TRACE_OBJ  := $(BUILD_DIR)/obj_dir_trace
WAVE       ?= $(BUILD_DIR)/wave.vcd

$(TRACE_BIN): $(VSRCS) $(CSRCS)
	@mkdir -p $(BUILD_DIR)
	verilator $(VFLAGS) --trace -DVCD_TRACE \
	  -CFLAGS "-O2 -std=c++17 -DVCD_TRACE" \
	  -Mdir $(TRACE_OBJ) -o $(TRACE_BIN) \
	  -MAKEFLAGS "PYTHON3=$(PYTHON3)" $(VSRCS) $(CSRCS)

sim-trace: $(TRACE_BIN) $(DEFAULT_IMG)
	@echo "==> $(TRACE_BIN) --image=$(DEFAULT_IMG) --trace=$(WAVE)"
	$(TRACE_BIN) --image=$(DEFAULT_IMG) --trace=$(WAVE)
	@echo "==> wave.vcd written: $(WAVE)"
	@ls -lh $(WAVE)

# ---- D6 ysyxSoC harness -----------------------------------------------------
SOC_DIR        := $(TOPDIR)/vsrc-soc
SOC_OBJ_DIR    := $(BUILD_DIR)/obj_dir_soc
SOC_BIN        := $(BUILD_DIR)/npc-soc
YSYXSOC_DIR    := $(abspath $(TOPDIR)/../ysyxSoC)

# CPU side: same modules used by `sim`, plus the SoC-compatible top wrapper.
SOC_CPU_VSRCS  := $(TOPDIR)/vsrc/csr.v \
                  $(TOPDIR)/vsrc/exu.v \
                  $(TOPDIR)/vsrc/idu.v \
                  $(TOPDIR)/vsrc/regfile.v \
                  $(TOPDIR)/vsrc/wbu.v \
                  $(TOPDIR)/vsrc/ysyx_22040000.v

# ysyxSoCFull (patched copy: module name `ysyx_00000000` -> `ysyx_22040000`).
SOC_TOP_V      := $(SOC_DIR)/ysyxSoCFull.v

# All peripheral .v files shipped with ysyxSoC.
SOC_PERIP_V    := $(shell find $(YSYXSOC_DIR)/perip -name '*.v' -type f)

SOC_VSRCS      := $(SOC_CPU_VSRCS) $(SOC_TOP_V) $(SOC_PERIP_V)
SOC_CSRCS      := $(TOPDIR)/csrc/main_soc.cpp
SOC_TOP_MOD    := ysyxSoCFull

# verilator needs to know where to find `uart_defines.v` / `spi_defines.v`
# which are `include`d by their siblings.
SOC_INCDIRS    := +incdir+$(YSYXSOC_DIR)/perip/uart16550/rtl \
                  +incdir+$(YSYXSOC_DIR)/perip/spi/rtl

# Notes on flags:
#   --timescale "1ns/1ns"  : required because peripheral .v files use `timescale
#   --no-timing            : disable SV timing constructs (delays etc.)
#   -Wno-* (extensive)     : the vendor RTL trips many lint rules we cannot fix.
SOC_VFLAGS     := --cc --exe --build --top-module $(SOC_TOP_MOD) \
                  --timescale "1ns/1ns" --no-timing \
                  -Wno-fatal -Wno-WIDTH -Wno-UNUSED -Wno-UNOPTFLAT \
                  -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-PINMISSING \
                  -Wno-IMPLICIT -Wno-CASEX -Wno-WIDTHCONCAT \
                  -Wno-MULTIDRIVEN -Wno-INITIALDLY -Wno-MULTITOP \
                  -Wno-COMBDLY -Wno-STMTDLY -Wno-SELRANGE \
                  -Wno-ASCRANGE -Wno-EOFNEWLINE -Wno-BLKANDNBLK \
                  $(SOC_INCDIRS) \
                  -Mdir $(SOC_OBJ_DIR) -o $(SOC_BIN) \
                  -CFLAGS "-O1 -std=c++17" -LDFLAGS "-ldl"

SOC_ARGS ?=

# Convenience: `make sim-soc FLASH=path/to/foo.bin [MAX_CYCLES=...]` forwards
# the args to npc-soc as `--flash=... --max-cycles=...`. SOC_ARGS still
# works for callers that want full control of the CLI.
ifneq ($(FLASH),)
SOC_ARGS += --flash=$(FLASH)
endif
ifneq ($(MAX_CYCLES),)
SOC_ARGS += --max-cycles=$(MAX_CYCLES)
endif

$(SOC_BIN): $(SOC_VSRCS) $(SOC_CSRCS)
	$(call git_commit, "sim SoC RTL") # DO NOT REMOVE THIS LINE!!!
	@mkdir -p $(BUILD_DIR)
	verilator $(SOC_VFLAGS) -MAKEFLAGS "PYTHON3=$(PYTHON3)" $(SOC_VSRCS) $(SOC_CSRCS)

sim-soc: $(SOC_BIN)
	@echo "==> $(SOC_BIN) $(SOC_ARGS)"; $(SOC_BIN) $(SOC_ARGS)

# ---- B1: full-handshake SimpleBus harness -----------------------------------
# Independent from `sim` (DPI-C, top=cpu) and `sim-soc` (ysyxSoC, top=
# ysyxSoCFull). Uses a new RTL top `ysyx_22040000_bus` whose IFU/LSU master
# interfaces speak the FULL handshake (reqValid/reqReady + respValid/
# respReady). The host harness in csrc/main_bus.cpp implements a slave-side
# memory model that injects random delays in both phases.
BUS_OBJ_DIR := $(BUILD_DIR)/obj_dir_bus
BUS_BIN     := $(BUILD_DIR)/npc-bus
BUS_VSRCS   := $(TOPDIR)/vsrc/csr.v \
               $(TOPDIR)/vsrc/exu.v \
               $(TOPDIR)/vsrc/idu.v \
               $(TOPDIR)/vsrc/regfile.v \
               $(TOPDIR)/vsrc/wbu.v \
               $(TOPDIR)/vsrc/ysyx_22040000_bus.v
BUS_CSRCS   := $(TOPDIR)/csrc/main_bus.cpp
BUS_TOP_MOD := ysyx_22040000_bus

BUS_VFLAGS  := --cc --exe --build --top-module $(BUS_TOP_MOD) \
               -Wno-fatal -Wno-WIDTH -Wno-UNUSED -Wno-UNOPTFLAT \
               -Mdir $(BUS_OBJ_DIR) -o $(BUS_BIN) \
               -CFLAGS "-O2 -std=c++17" -LDFLAGS "-ldl"

BUS_ARGS ?=

$(BUS_BIN): $(BUS_VSRCS) $(BUS_CSRCS)
	@mkdir -p $(BUILD_DIR)
	verilator $(BUS_VFLAGS) -MAKEFLAGS "PYTHON3=$(PYTHON3)" $(BUS_VSRCS) $(BUS_CSRCS)

sim-bus: $(BUS_BIN) $(DEFAULT_IMG)
	@if [ -n "$(BUS_ARGS)" ]; then \
	  echo "==> $(BUS_BIN) $(BUS_ARGS)"; $(BUS_BIN) $(BUS_ARGS); \
	else \
	  echo "==> $(BUS_BIN) --image=$(DEFAULT_IMG)"; $(BUS_BIN) --image=$(DEFAULT_IMG); \
	fi

# Sweep a fixed set of cpu-tests under several random seeds + delay profiles.
# Listed binaries are the riscv32e-npc (full RV32E) flavour built by the
# am-kernels cpu-tests target; they all end with `ebreak; a0=0` on success.
BUS_TESTS_DIR := $(abspath $(TOPDIR)/../am-kernels/tests/cpu-tests/build)
BUS_TESTS     := dummy add bubble-sort recursion fib
BUS_PROFILES  := "--req-min=1 --req-max=1 --resp-min=0 --resp-max=0" \
                 "--req-min=1 --req-max=3 --resp-min=0 --resp-max=3" \
                 "--req-min=2 --req-max=8 --resp-min=2 --resp-max=10"

bus-tests: $(BUS_BIN)
	@set -e; pass=0; fail=0; \
	for t in $(BUS_TESTS); do \
	  img=$(BUS_TESTS_DIR)/$$t-riscv32e-npc.bin; \
	  if [ ! -f $$img ]; then echo "  SKIP $$t (missing $$img)"; continue; fi; \
	  for seed in 0xdead1 0xbeef2 0xcafe3; do \
	    for prof in $(BUS_PROFILES); do \
	      printf "  %-14s seed=%s prof=%s ... " "$$t" "$$seed" "$$prof"; \
	      out=$$($(BUS_BIN) --image=$$img --seed=$$seed $$prof --max-cycles=2000000 2>/dev/null); \
	      if echo "$$out" | grep -q "HIT GOOD TRAP"; then \
	        echo "PASS"; pass=$$((pass+1)); \
	      else \
	        echo "FAIL ($$out)"; fail=$$((fail+1)); \
	      fi; \
	    done; \
	  done; \
	done; \
	echo "==> bus-tests done: pass=$$pass fail=$$fail"; \
	test $$fail -eq 0

clean:
	rm -rf $(BUILD_DIR)

# ---- D6b: yosys synthesis (lint + check -assert, no tech mapping) -----------
# Goal: prove the NPC RTL is synth-clean (no inferred latch, no multiple
# drivers). We synthesise two views:
#   1. The SoC-facing top `ysyx_22040000` (this is the *real* tape-out target).
#   2. The DPI-C harness top `cpu`, parsed with -DSYNTHESIS so the DPI calls
#      and the cross-module `u_rf.rf[10]` reference are stripped at parse
#      time and replaced with plain pmem-shaped ports.
#
# yosys is invoked with -q so stdout only carries warnings/errors. The full
# (and very chatty) yosys log lands in $(BUILD_DIR)/syn-soc.log and
# $(BUILD_DIR)/syn-cpu.log for debugging. `check -assert` inside the .ys
# scripts hard-fails on any latch / unconnected / driver-conflict issue so
# the exit code is reliable on its own.
SYN_LOG_SOC  := $(BUILD_DIR)/syn-soc.log
SYN_LOG_CPU  := $(BUILD_DIR)/syn-cpu.log
SYN_LOG_BUS  := $(BUILD_DIR)/syn-bus.log
# Sources actually parsed by the three yosys scripts (cpu top + soc-wrapper
# top + bus-handshake top).
SYN_RTL_SRCS := $(VSRCS) $(TOPDIR)/vsrc/ysyx_22040000.v $(TOPDIR)/vsrc/ysyx_22040000_bus.v

yosys-synth: $(SYN_RTL_SRCS) $(TOPDIR)/syn/synth_soc_top.ys $(TOPDIR)/syn/synth_cpu_top.ys $(TOPDIR)/syn/synth_bus_top.ys
	@mkdir -p $(BUILD_DIR)
	@echo "==> yosys: synthesise ysyx_22040000 (SoC-facing CPU top)"
	cd $(TOPDIR) && yosys -q -l $(SYN_LOG_SOC) -s syn/synth_soc_top.ys
	@echo "==> yosys: synthesise cpu (D4/D5 harness top, -DSYNTHESIS)"
	cd $(TOPDIR) && yosys -q -l $(SYN_LOG_CPU) -s syn/synth_cpu_top.ys
	@echo "==> yosys: synthesise ysyx_22040000_bus (B1 full-handshake top)"
	cd $(TOPDIR) && yosys -q -l $(SYN_LOG_BUS) -s syn/synth_bus_top.ys
	@echo "==> yosys synthesis OK: clean lint, all check -assert passed"
	@echo "    full log: $(SYN_LOG_SOC), $(SYN_LOG_CPU), $(SYN_LOG_BUS)"

.PHONY: all sim sim-soc sim-trace sim-bus bus-tests clean yosys-synth

include $(TOPDIR)/../Makefile
