/* Licensed under the Apache License, Version 2.0 or the MIT License. */
/* SPDX-License-Identifier: Apache-2.0 OR MIT                         */
/* Copyright Tock Contributors 2026.                                  */

/* Memory layout for the STM32MP157x (Cortex-M4)
 * Based on MCU SRAM mapping:
 * m_text (Code/ROM equivalent) starts at 0x10000000
 * m_data (RAM equivalent) starts at 0x10020000
 *
 * We divide the available MCU SRAM into ROM (Flash/Code) and RAM.
 */

MEMORY
{
  /* ROM: Maps to 'm_text' in ST script. 
   * Length is 128KB (0x20000) */
  rom (rx)  : ORIGIN = 0x10000000, LENGTH = 0x00020000

  /* PROG: This is the space for Userland apps. 
   * Since MP1 usually loads apps to RAM, we can carve this 
   * out of the upper part of the code RAM or use a different bank. */
  prog (rx) : ORIGIN = 0x10020000, LENGTH = 0x00010000

  /* RAM: Maps to 'm_data' in ST script.
   * Origin adjusted to follow prog if needed, or set to standard data bank.
   * Total data RAM in ST script is 128KB (0x20000). */
  ram (rwx) : ORIGIN = 0x10030000, LENGTH = 0x00010000
}

/* The STM32MP1 SRAM typically doesn't have "pages" in the Flash sense,
 * but Tock uses this for alignment. 1K or 2K is standard. */
PAGE_SIZE = 1K;
