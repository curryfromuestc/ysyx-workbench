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

$(SOC_BIN): $(SOC_VSRCS) $(SOC_CSRCS)
	$(call git_commit, "sim SoC RTL") # DO NOT REMOVE THIS LINE!!!
	@mkdir -p $(BUILD_DIR)
	verilator $(SOC_VFLAGS) -MAKEFLAGS "PYTHON3=$(PYTHON3)" $(SOC_VSRCS) $(SOC_CSRCS)

sim-soc: $(SOC_BIN)
	@echo "==> $(SOC_BIN) $(SOC_ARGS)"; $(SOC_BIN) $(SOC_ARGS)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all sim sim-soc clean

include $(TOPDIR)/../Makefile
