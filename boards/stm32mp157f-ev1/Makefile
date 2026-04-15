# Licensed under the Apache License, Version 2.0 or the MIT License.
# SPDX-License-Identifier: Apache-2.0 OR MIT
# Copyright Tock Contributors 2025.

# Makefile for building the tock kernel for the STM32MP157F-EV1 board
# (Cortex-M4 core in engineering mode)

# Capture this file's directory before `include` changes MAKEFILE_LIST.
BOARD_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

include ../Makefile.common

OPENOCD=openocd
TOCKLOADER ?= tockloader
DD ?= dd

KERNEL_FLASH_ADDRESS := 0x10000000
APPS_FLASH_ADDRESS := 0x10020000

APP_BUILD_DIR := /Users/tom/tock-org/tock/boards/stm32mp157f-ev1/app_builds
APPS_TAB_FILES := $(sort $(wildcard $(APP_BUILD_DIR)/*.tab))
LOCAL_APPS_BOARD := stm32mp157f-ev1-local-apps

APPS_BUNDLE_BIN := $(APP_BUILD_DIR)/apps_bundle.bin
KERNEL_AND_APPS_BIN := $(APP_BUILD_DIR)/kernel_and_apps.bin

# OpenOCD configuration for STM32MP1 engineering mode.
# Engineering mode: Cortex-A7 is not running, M4 loaded directly via debugger.
# AP_NUM 2 selects the Cortex-M4 access port on the dual-core STM32MP1.
OPENOCD_CFG = \
	-f interface/stlink-dap.cfg \
	-c "transport select dapdirect_swd" \
	-c "set WORKAREASIZE 0x8000" \
	-c "set CHIPNAME STM32MP157CAAx" \
	-c "set ENABLE_LOW_POWER 1" \
	-c "set STOP_WATCHDOG 1" \
	-c "set CLOCK_FREQ 8000" \
	-c "set AP_NUM 2" \
	-f target/stm32mp15x.cfg \
	-c "reset_config srst_only" \
	-c "STM32MP157CAAx.cpu0 configure -defer-examine" \
	-c "STM32MP157CAAx.cpu1 configure -defer-examine"

# After reset halt, the default target is cpu0 (Cortex-A7). Switch to the CM4
# target and configure a work area for load/verify. We use the IPC shared memory
# region (0x10040000) which is outside our linker regions.
OPENOCD_LOAD = \
	-c "init" \
	-c "reset halt" \
	-c "targets STM32MP157CAAx.cm4" \
	-c "STM32MP157CAAx.cm4 configure -work-area-phys 0x10040000 -work-area-size 0x8000 -work-area-backup 0"

# Default target for installing the kernel.
.PHONY: install
install: flash

.PHONY: flash-debug
flash-debug: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/debug/$(PLATFORM).elf
	$(OPENOCD) $(OPENOCD_CFG) $(OPENOCD_LOAD) \
	-c "load_image $<" \
	-c "verify_image $<" \
	-c "source $(BOARD_DIR)openocd_cm4_resume.tcl"

.PHONY: flash
flash: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).elf
	$(OPENOCD) $(OPENOCD_CFG) $(OPENOCD_LOAD) \
	-c "load_image $<" \
	-c "verify_image $<" \
	-c "source $(BOARD_DIR)openocd_cm4_resume.tcl"

.PHONY: flash-kernel-and-apps
flash-kernel-and-apps: $(KERNEL_AND_APPS_BIN)
	$(OPENOCD) $(OPENOCD_CFG) $(OPENOCD_LOAD) \
	-c "load_image $< $(KERNEL_FLASH_ADDRESS)" \
	-c "verify_image $< $(KERNEL_FLASH_ADDRESS)" \
	-c "source $(BOARD_DIR)openocd_cm4_resume.tcl"

$(APPS_BUNDLE_BIN): $(APPS_TAB_FILES)
	@if [ -z "$(strip $(APPS_TAB_FILES))" ]; then \
		echo "No app TAB files found in $(APP_BUILD_DIR)."; \
		echo "Build apps first so $(APP_BUILD_DIR) contains *.tab files."; \
		exit 1; \
	fi
	@set -euo pipefail; \
	trap '$(TOCKLOADER) local-board unset >/dev/null 2>&1 || true' EXIT; \
	rm -f "$@"; \
	$(TOCKLOADER) local-board set $(LOCAL_APPS_BOARD) \
		--binary-path "$@" \
		--arch cortex-m4 \
		--app-address $(APPS_FLASH_ADDRESS) \
		--flash-address $(KERNEL_FLASH_ADDRESS); \
	$(TOCKLOADER) install --local-board --board $(LOCAL_APPS_BOARD) --bundle-apps $(APPS_TAB_FILES)

$(KERNEL_AND_APPS_BIN): $(TARGET_PATH)/release/$(PLATFORM).bin $(APPS_BUNDLE_BIN)
	$(Q)cp "$(APPS_BUNDLE_BIN)" "$@"
	$(Q)$(DD) if="$<" of="$@" conv=notrunc status=none
