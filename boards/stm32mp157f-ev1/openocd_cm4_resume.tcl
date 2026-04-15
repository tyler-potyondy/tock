# Licensed under the Apache License, Version 2.0 or the MIT License.
# SPDX-License-Identifier: Apache-2.0 OR MIT
# Copyright Tock Contributors 2026.
#
# OpenOCD helper: start the CM4 like a hardware reset after the kernel ELF
# was programmed into MCU SRAM.
#
# Prerequisites (caller must do this first):
#   - OpenOCD is connected, `init` has run
#   - Current target is STM32MP157CAAx.cm4 (see Makefile `targets ...`)
#   - The kernel image is already loaded into MCU SRAM (use `load_image` on the ELF)
#
# Cortex-M reset loads MSP from word 0 and PC from word 1 of the vector table.
# Simply `resume 0x10000000` is wrong: that address holds the initial MSP value,
# not Thumb code. We set VTOR, MSP, and PC from the table then resume.

mww 0xE000ED08 0x10000000
set _tock_vt [read_memory 0x10000000 32 2]
set _msp [lindex $_tock_vt 0]
set _pc [lindex $_tock_vt 1]
echo [format "stm32mp157f-ev1: VTOR=0x10000000 MSP=0x%08x PC=0x%08x" $_msp $_pc]
reg msp $_msp
reg pc $_pc
resume
shutdown
