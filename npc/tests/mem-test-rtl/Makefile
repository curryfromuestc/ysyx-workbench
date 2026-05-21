# B2b: stand-alone harness for the PSRAM / SDRAM die-level models.
#
# These tests pair each die model (ysyxSoC/perip/psram/psram.v and
# ysyxSoC/perip/sdram/sdram.v) directly with the controller it was written
# for (EF_PSRAM_CTRL_wb and sdram_axi_core respectively) and run a 4 KB
# write/read pattern sweep over APB. They are entirely independent from
# npc/sim-soc -- the patched ysyxSoCFull.v shipped with this tree does not
# instantiate either die, so this is the canonical way to verify the models.
#
# We use verilator (rather than iverilog) because EF_PSRAM_CTRL.v uses
# SystemVerilog-only bit-select on an expression that iverilog does not
# accept, and we are forbidden from modifying the controller source.
# `--binary --timing` lets us drive the testbench entirely from initial
# begin / always #5 blocks without writing a separate C++ harness. Add
# `--trace` and `-DDUMP` if you need a VCD; it is off by default to keep
# regression runs fast (the SDRAM run otherwise produces ~60 MByte of VCD
# for a single 4 KB sweep).

TOPDIR    := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
WB_DIR    := $(abspath $(TOPDIR)/../../..)
PERIP_DIR := $(WB_DIR)/ysyxSoC/perip

COMMON_INC := $(TOPDIR)/apb_master.vh

PSRAM_SRCS := $(PERIP_DIR)/psram/psram_top_apb.v \
              $(PERIP_DIR)/psram/efabless/EF_PSRAM_CTRL_wb.v \
              $(PERIP_DIR)/psram/efabless/EF_PSRAM_CTRL.v \
              $(PERIP_DIR)/psram/psram.v \
              $(TOPDIR)/tb_psram.v

# B2c: 1 MiB stress variant of the same harness. Re-uses the same RTL --
# only the testbench top differs (size 4 KiB -> 1 MiB).
PSRAM_1M_SRCS := $(PERIP_DIR)/psram/psram_top_apb.v \
                 $(PERIP_DIR)/psram/efabless/EF_PSRAM_CTRL_wb.v \
                 $(PERIP_DIR)/psram/efabless/EF_PSRAM_CTRL.v \
                 $(PERIP_DIR)/psram/psram.v \
                 $(TOPDIR)/tb_psram_1m.v

SDRAM_SRCS := $(PERIP_DIR)/sdram/sdram_top_apb.v \
              $(PERIP_DIR)/sdram/core_sdram_axi4/sdram_axi_core.v \
              $(PERIP_DIR)/sdram/sdram.v \
              $(TOPDIR)/tb_sdram.v

BUILD_PSRAM    := $(TOPDIR)/build/psram
BUILD_PSRAM_1M := $(TOPDIR)/build/psram_1m
BUILD_SDRAM    := $(TOPDIR)/build/sdram

VFLAGS := --binary --timing -j 1 \
          --timescale "1ns/1ps" \
          -CFLAGS "-O1 -std=c++20" \
          +incdir+$(TOPDIR) \
          -Wno-fatal -Wno-WIDTH -Wno-UNUSED -Wno-UNOPTFLAT \
          -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-PINMISSING \
          -Wno-IMPLICIT -Wno-CASEX -Wno-WIDTHCONCAT \
          -Wno-MULTIDRIVEN -Wno-INITIALDLY -Wno-MULTITOP \
          -Wno-COMBDLY -Wno-STMTDLY -Wno-SELRANGE \
          -Wno-ASCRANGE -Wno-EOFNEWLINE -Wno-BLKANDNBLK \
          -Wno-PROCASSWIRE -Wno-CONSTRAINTIGN

PYTHON3 := $(shell command -v python3.12 || command -v python3.11 || command -v python3 || echo /usr/bin/python3)

.PHONY: all psram psram-1m sdram clean

# Default leaves the 1 MiB stress test out so `make all` stays fast for the
# B2b smoke regression. Run it explicitly with `make psram-1m`.
all: psram sdram

$(BUILD_PSRAM)/Vtb_psram: $(PSRAM_SRCS) $(COMMON_INC)
	@mkdir -p $(BUILD_PSRAM)
	verilator $(VFLAGS) --top-module tb_psram \
	  -Mdir $(BUILD_PSRAM) -o Vtb_psram \
	  -MAKEFLAGS "PYTHON3=$(PYTHON3)" \
	  $(PSRAM_SRCS)

$(BUILD_SDRAM)/Vtb_sdram: $(SDRAM_SRCS) $(COMMON_INC)
	@mkdir -p $(BUILD_SDRAM)
	verilator $(VFLAGS) --top-module tb_sdram \
	  -Mdir $(BUILD_SDRAM) -o Vtb_sdram \
	  -MAKEFLAGS "PYTHON3=$(PYTHON3)" \
	  $(SDRAM_SRCS)

$(BUILD_PSRAM_1M)/Vtb_psram_1m: $(PSRAM_1M_SRCS) $(COMMON_INC)
	@mkdir -p $(BUILD_PSRAM_1M)
	verilator $(VFLAGS) --top-module tb_psram_1m \
	  -Mdir $(BUILD_PSRAM_1M) -o Vtb_psram_1m \
	  -MAKEFLAGS "PYTHON3=$(PYTHON3)" \
	  $(PSRAM_1M_SRCS)

psram: $(BUILD_PSRAM)/Vtb_psram
	@echo "==> running PSRAM mem-test"
	$(BUILD_PSRAM)/Vtb_psram

psram-1m: $(BUILD_PSRAM_1M)/Vtb_psram_1m
	@echo "==> running PSRAM mem-test 1MiB stress"
	$(BUILD_PSRAM_1M)/Vtb_psram_1m

sdram: $(BUILD_SDRAM)/Vtb_sdram
	@echo "==> running SDRAM mem-test"
	$(BUILD_SDRAM)/Vtb_sdram

clean:
	rm -rf $(TOPDIR)/build
