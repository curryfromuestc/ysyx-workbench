# NPC: minirv single-cycle CPU, verilated.
#
# Targets:
#   sim     verilate + compile -> build/npc; run with ARGS=... (or default img)
#   clean   wipe build/

TOPDIR    := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
BUILD_DIR := $(TOPDIR)/build

# Provide NEMU_HOME for ../Makefile's git_commit hook (it computes
# YSYX_HOME = $(NEMU_HOME)/..). NEMU_HOME may come from the environment;
# default to the sibling nemu directory in the workbench tree.
NEMU_HOME ?= $(abspath $(TOPDIR)/../nemu)
export NEMU_HOME
OBJ_DIR   := $(BUILD_DIR)/obj_dir
BIN       := $(BUILD_DIR)/npc

VSRCS     := $(wildcard $(TOPDIR)/vsrc/*.v)
CSRCS     := $(wildcard $(TOPDIR)/csrc/*.cpp)
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

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all sim clean

include $(TOPDIR)/../Makefile
