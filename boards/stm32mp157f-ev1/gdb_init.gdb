# GDB initialization script for STM32MP157 Cortex-M4 (Engineering Mode)
#
# Usage:
#   1. Start OpenOCD in a separate terminal (same flags as `make flash`):
#        openocd -f interface/stlink-dap.cfg \
#          -c "transport select dapdirect_swd" \
#          -c "set WORKAREASIZE 0x8000" \
#          -c "set CHIPNAME STM32MP157CAAx" \
#          -c "set ENABLE_LOW_POWER 1" \
#          -c "set STOP_WATCHDOG 1" \
#          -c "set CLOCK_FREQ 8000" \
#          -c "set AP_NUM 2" \
#          -f target/stm32mp15x.cfg \
#          -c "reset_config srst_only" \
#          -c "STM32MP157CAAx.cpu0 configure -defer-examine" \
#          -c "STM32MP157CAAx.cpu1 configure -defer-examine"
#
#   2. From the Tock repository root, run GDB (cwd must be the repo so the
#      hardcoded ELF path resolves):
#        arm-none-eabi-gdb -x boards/stm32mp157f-ev1/gdb_init.gdb
#
# NOTE: The Cortex-A7 "timeout waiting for DSCR" errors on the OpenOCD
# console are expected in Engineering Mode -- the A7 cores are not
# accessible. They do not affect CM4 debugging.
#
# What this does:
#   After reset, the CM4 PC is at 0x00000008 (BootROM default). The Tock
#   kernel is linked for MCU SRAM at 0x10000000. This script loads the ELF,
#   then manually sets VTOR/MSP/PC so the CM4 begins executing from the
#   Tock vector table in SRAM -- bypassing the need for a bootloader.

file ../../target/thumbv7em-none-eabi/debug/stm32mp157f-ev1.elf

# Increase timeout -- OpenOCD is slow due to A7 error retries.
set remotetimeout 30

# Connect to the CM4 GDB port (see OpenOCD startup: cm4 is often 3334 when both
# A7 cores are deferred; always confirm in the log).
target extended-remote :3334

# Route monitor commands to the Cortex-M4 target (required before reset/load).
monitor targets STM32MP157CAAx.cm4

# Configure work area for the CM4 (outside our linker regions)
monitor STM32MP157CAAx.cm4 configure -work-area-phys 0x10040000 -work-area-size 0x8000 -work-area-backup 0

# Reset and halt the CM4. The A7 "DSCR timeout" errors on the OpenOCD
# console are expected and harmless in Engineering Mode.
monitor reset halt

# Load the Tock ELF into MCU SRAM at 0x10000000
load

# --- Manual vector table bootstrap ---
# On Cortex-M, the vector table starts with:
#   Word 0 (offset 0x00): Initial Stack Pointer (MSP)
#   Word 1 (offset 0x04): Reset Handler (entry point)
#
# Use OpenOCD to write VTOR since the PPB region (0xE0000000+) may not
# be accessible via GDB's memory model.

# Set VTOR (Vector Table Offset Register) to our vector table in SRAM.
monitor mww 0xE000ED08 0x10000000

# Read MSP and PC from the loaded vector table in SRAM.
# Now that we're on the CM4 GDB port, GDB memory access works.
# Use C-style casts; `{unsigned int}addr` breaks some GDB builds ("unexpected token").
set $msp = *(unsigned int *)0x10000000
set $pc = *(unsigned int *)0x10000004

# Verify with explicit reads
printf "=== Vector table at 0x10000000 ===\n"
printf "  Word 0 (MSP):   0x%08x\n", *(unsigned int *)0x10000000
printf "  Word 1 (Reset):  0x%08x\n", *(unsigned int *)0x10000004
printf "  Word 2 (NMI):    0x%08x\n", *(unsigned int *)0x10000008
printf "  Word 3 (HardF):  0x%08x\n", *(unsigned int *)0x1000000C
printf "=== Registers after setup ===\n"
printf "  VTOR  = 0x%08x\n", *(unsigned int *)0xE000ED08
printf "  MSP   = 0x%08x\n", $msp
printf "  PC    = 0x%08x\n", $pc
printf "=== Linker symbols ===\n"
printf "  _szero      = 0x%08x\n", &_szero
printf "  _ezero      = 0x%08x\n", &_ezero
printf "  _srelocate  = 0x%08x\n", &_srelocate
printf "  _erelocate  = 0x%08x\n", &_erelocate
printf "  _etext      = 0x%08x\n", &_etext
printf "  _sstack     = 0x%08x\n", &_sstack
printf "  _estack     = 0x%08x\n", &_estack
printf "=== Fault status (CFSR) ===\n"
printf "  CFSR  = 0x%08x\n", *(unsigned int *)0xE000ED28
printf "  HFSR  = 0x%08x\n", *(unsigned int *)0xE000ED2C
printf "================================\n"

# Do not `break initialize_ram_jump_to_main`: GDB often places that breakpoint
# on the first `cmp` inside the naked BSS-clear loop (e.g. 0x1000013e), so every
# `continue` stops once per word zeroed — looks like an infinite reset loop.
# Break in Rust code after RAM init instead:
break main
