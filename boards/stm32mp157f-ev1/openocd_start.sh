#!/usr/bin/env bash
# Start OpenOCD for STM32MP157 Cortex-M4 Engineering Mode debugging.
# Usage: ./boards/stm32mp157f-ev1/openocd_start.sh
#
# GDB ports (check OpenOCD log: "Listening on port ... for gdb connections").
# With both A7 cores deferred, stm32mp15x often exposes cpu0 on 3333 and cm4 on
# 3334 (cpu1 may have no GDB server). Connect GDB to the cm4 port.

exec openocd \
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
