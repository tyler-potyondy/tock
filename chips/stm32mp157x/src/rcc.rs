// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, ReadWrite};
use kernel::utilities::StaticRef;

//=================================================================================================
// (NOTE) Register translation and bitfield macros generated from stm32mp157fxx_ca7.h in cmsis using
// LLM. cmsis file source:
// https://github.com/STMicroelectronics/STM32CubeMP1/blob/525d2499658d817a9e669eb17e66390906954895/Drivers/CMSIS/Device/ST/STM32MP1xx/Include/stm32mp157fxx_ca7.h)
//=================================================================================================

/// Reset and clock control
#[repr(C)]
struct RccRegisters {
    /// RCC TrustZone Control Register.
    tzcr: ReadWrite<u32, TZCR::Register>,
    /// Reserved.
    _reserved0: [u32; 2],
    /// RCC Oscillator Clock Enable Set Register.
    ocensetr: ReadWrite<u32, OCENSETR::Register>,
    /// RCC Oscillator Enable Control Clear Register.
    ocenclrr: ReadWrite<u32, OCENCLRR::Register>,
    /// Reserved.
    _reserved1: u32,
    /// RCC HSI Configuration Register.
    hsicfgr: ReadWrite<u32, HSICFGR::Register>,
    /// RCC CSI Configuration Register.
    csicfgr: ReadWrite<u32, CSICFGR::Register>,
    /// RCC MPU Clock Selection Register.
    mpckselr: ReadWrite<u32, MPCKSELR::Register>,
    /// RCC AXI Sub-System Clock Selection Register.
    assckselr: ReadWrite<u32, ASSCKSELR::Register>,
    /// RCC PLL1 and PLL2 Reference Clock Selection Register.
    rck12selr: ReadWrite<u32, RCK12SELR::Register>,
    /// RCC MPU Clock Divider Register.
    mpckdivr: ReadWrite<u32, MPCKDIVR::Register>,
    /// RCC AXI Clock Divider Register.
    axidivr: ReadWrite<u32, AXIDIVR::Register>,
    /// Reserved.
    _reserved2: [u32; 2],
    /// RCC APB4 Clock Divider Register.
    apb4divr: ReadWrite<u32, APB4DIVR::Register>,
    /// RCC APB5 Clock Divider Register.
    apb5divr: ReadWrite<u32, APB5DIVR::Register>,
    /// RCC RTC Clock Division Register.
    rtcdivr: ReadWrite<u32, RTCDIVR::Register>,
    /// RCC MCU Sub-System Clock Selection Register.
    mssckselr: ReadWrite<u32, MSSCKSELR::Register>,
    /// Reserved.
    _reserved3: [u32; 13],
    /// RCC PLL1 Control Register.
    pll1cr: ReadWrite<u32, PLL1CR::Register>,
    /// RCC PLL1 Configuration Register 1.
    pll1cfgr1: ReadWrite<u32, PLL1CFGR1::Register>,
    /// RCC PLL1 Configuration Register 2.
    pll1cfgr2: ReadWrite<u32, PLL1CFGR2::Register>,
    /// RCC PLL1 Fractional Register.
    pll1fracr: ReadWrite<u32, PLL1FRACR::Register>,
    /// RCC PLL1 Clock Spreading Generator Register.
    pll1csgr: ReadWrite<u32, PLL1CSGR::Register>,
    /// RCC PLL2 Control Register.
    pll2cr: ReadWrite<u32, PLL2CR::Register>,
    /// RCC PLL2 Configuration Register 1.
    pll2cfgr1: ReadWrite<u32, PLL2CFGR1::Register>,
    /// RCC PLL2 Configuration Register 2.
    pll2cfgr2: ReadWrite<u32, PLL2CFGR2::Register>,
    /// RCC PLL2 Fractional Register.
    pll2fracr: ReadWrite<u32, PLL2FRACR::Register>,
    /// RCC PLL2 Clock Spreading Generator Register.
    pll2csgr: ReadWrite<u32, PLL2CSGR::Register>,
    /// Reserved.
    _reserved4: [u32; 6],
    /// RCC I2C4 and I2C6 Kernel Clock Selection Register.
    i2c46ckselr: ReadWrite<u32>,
    /// RCC SPI6 Kernel Clock Selection Register.
    spi6ckselr: ReadWrite<u32>,
    /// RCC USART1 Kernel Clock Selection Register.
    uart1ckselr: ReadWrite<u32>,
    /// RCC RNG1 Kernel Clock Selection Register.
    rng1ckselr: ReadWrite<u32>,
    /// RCC Common Peripheral Clock Selection Register.
    cperckselr: ReadWrite<u32>,
    /// RCC STGEN Clock Selection Register.
    stgenckselr: ReadWrite<u32>,
    /// RCC DDR Interface Control Register.
    ddritfcr: ReadWrite<u32>,
    /// Reserved.
    _reserved5: u32,
    /// Reserved.
    _reserved6: [u32; 8],
    /// RCC Hold Boot Control Register.
    mp_bootcr: ReadWrite<u32>,
    /// RCC Stop Request Set Register.
    mp_sreqsetr: ReadWrite<u32>,
    /// RCC Stop Request Clear Register.
    mp_sreqclrr: ReadWrite<u32>,
    /// RCC Global Control Register.
    mp_gcr: ReadWrite<u32>,
    /// RCC Application Reset Control Register.
    mp_aprstcr: ReadWrite<u32>,
    /// RCC Application Reset Status Register.
    mp_aprstsr: ReadWrite<u32>,
    /// Reserved.
    _reserved7: [u32; 10],
    /// RCC Backup Domain Control Register.
    bdcr: ReadWrite<u32>,
    /// RCC Reset Duration and LSI Control Register.
    rdlsicr: ReadWrite<u32>,
    /// Reserved.
    _reserved8: [u32; 14],
    /// RCC APB4 Peripheral Reset Set Register.
    apb4rstsetr: ReadWrite<u32>,
    /// RCC APB4 Peripheral Reset Clear Register.
    apb4rstclrr: ReadWrite<u32>,
    /// RCC APB5 Peripheral Reset Set Register.
    apb5rstsetr: ReadWrite<u32>,
    /// RCC APB5 Peripheral Reset Clear Register.
    apb5rstclrr: ReadWrite<u32>,
    /// RCC AHB5 Peripheral Reset Set Register.
    ahb5rstsetr: ReadWrite<u32>,
    /// RCC AHB5 Peripheral Reset Clear Register.
    ahb5rstclrr: ReadWrite<u32>,
    /// RCC AHB6 Peripheral Reset Set Register.
    ahb6rstsetr: ReadWrite<u32>,
    /// RCC AHB6 Peripheral Reset Clear Register.
    ahb6rstclrr: ReadWrite<u32>,
    /// RCC TZAHB6 Peripheral Reset Set Register.
    tzahb6rstsetr: ReadWrite<u32>,
    /// RCC TZAHB6 Peripheral Reset Clear Register.
    tzahb6rstclrr: ReadWrite<u32>,
    /// Reserved.
    _reserved9: [u32; 22],
    /// RCC APB4 Peripheral Enable for MPU Set Register.
    mp_apb4ensetr: ReadWrite<u32>,
    /// RCC APB4 Peripheral Enable for MPU Clear Register.
    mp_apb4enclrr: ReadWrite<u32>,
    /// RCC APB5 Peripheral Enable for MPU Set Register.
    mp_apb5ensetr: ReadWrite<u32>,
    /// RCC APB5 Peripheral Enable for MPU Clear Register.
    mp_apb5enclrr: ReadWrite<u32>,
    /// RCC AHB5 Peripheral Enable for MPU Set Register.
    mp_ahb5ensetr: ReadWrite<u32>,
    /// RCC AHB5 Peripheral Enable for MPU Clear Register.
    mp_ahb5enclrr: ReadWrite<u32>,
    /// RCC AHB6 Peripheral Enable for MPU Set Register.
    mp_ahb6ensetr: ReadWrite<u32>,
    /// RCC AHB6 Peripheral Enable for MPU Clear Register.
    mp_ahb6enclrr: ReadWrite<u32>,
    /// Reserved.
    _reserved10: [u32; 24],
    /// RCC APB4 Peripheral Enable for MCU Set Register.
    mc_apb4ensetr: ReadWrite<u32>,
    /// RCC APB4 Peripheral Enable for MCU Clear Register.
    mc_apb4enclrr: ReadWrite<u32>,
    /// RCC APB5 Peripheral Enable for MCU Set Register.
    mc_apb5ensetr: ReadWrite<u32>,
    /// RCC APB5 Peripheral Enable for MCU Clear Register.
    mc_apb5enclrr: ReadWrite<u32>,
    /// RCC AHB5 Peripheral Enable for MCU Set Register.
    mc_ahb5ensetr: ReadWrite<u32>,
    /// RCC AHB5 Peripheral Enable for MCU Clear Register.
    mc_ahb5enclrr: ReadWrite<u32>,
    /// RCC AHB6 Peripheral Enable for MCU Set Register.
    mc_ahb6ensetr: ReadWrite<u32>,
    /// RCC AHB6 Peripheral Enable for MCU Clear Register.
    mc_ahb6enclrr: ReadWrite<u32>,
    /// Reserved.
    _reserved11: [u32; 24],
    /// RCC APB4 Sleep Clock Enable for MPU Set Register.
    mp_apb4lpensetr: ReadWrite<u32>,
    /// RCC APB4 Sleep Clock Enable for MPU Clear Register.
    mp_apb4lpenclrr: ReadWrite<u32>,
    /// RCC APB5 Sleep Clock Enable for MPU Set Register.
    mp_apb5lpensetr: ReadWrite<u32>,
    /// RCC APB5 Sleep Clock Enable for MPU Clear Register.
    mp_apb5lpenclrr: ReadWrite<u32>,
    /// RCC AHB5 Sleep Clock Enable for MPU Set Register.
    mp_ahb5lpensetr: ReadWrite<u32>,
    /// RCC AHB5 Sleep Clock Enable for MPU Clear Register.
    mp_ahb5lpenclrr: ReadWrite<u32>,
    /// RCC AHB6 Sleep Clock Enable for MPU Set Register.
    mp_ahb6lpensetr: ReadWrite<u32>,
    /// RCC AHB6 Sleep Clock Enable for MPU Clear Register.
    mp_ahb6lpenclrr: ReadWrite<u32>,
    /// Reserved.
    _reserved12: [u32; 24],
    /// RCC APB4 Sleep Clock Enable for MCU Set Register.
    mc_apb4lpensetr: ReadWrite<u32>,
    /// RCC APB4 Sleep Clock Enable for MCU Clear Register.
    mc_apb4lpenclrr: ReadWrite<u32>,
    /// RCC APB5 Sleep Clock Enable for MCU Set Register.
    mc_apb5lpensetr: ReadWrite<u32>,
    /// RCC APB5 Sleep Clock Enable for MCU Clear Register.
    mc_apb5lpenclrr: ReadWrite<u32>,
    /// RCC AHB5 Sleep Clock Enable for MCU Set Register.
    mc_ahb5lpensetr: ReadWrite<u32>,
    /// RCC AHB5 Sleep Clock Enable for MCU Clear Register.
    mc_ahb5lpenclrr: ReadWrite<u32>,
    /// RCC AHB6 Sleep Clock Enable for MCU Set Register.
    mc_ahb6lpensetr: ReadWrite<u32>,
    /// RCC AHB6 Sleep Clock Enable for MCU Clear Register.
    mc_ahb6lpenclrr: ReadWrite<u32>,
    /// Reserved.
    _reserved13: [u32; 24],
    /// RCC BootROM Reset Status Clear Register.
    br_rstsclrr: ReadWrite<u32>,
    /// RCC Global Reset Control Set Register.
    mp_grstcsetr: ReadWrite<u32>,
    /// RCC MPU Reset Status Clear Register.
    mp_rstsclrr: ReadWrite<u32>,
    /// RCC IWDG Clock Freeze Set Register.
    mp_iwdgfzsetr: ReadWrite<u32>,
    /// RCC IWDG Clock Freeze Clear Register.
    mp_iwdgfzclrr: ReadWrite<u32>,
    /// RCC Clock Source Interrupt Enable Register.
    mp_cier: ReadWrite<u32>,
    /// RCC Clock Source Interrupt Flag Register.
    mp_cifr: ReadWrite<u32>,
    /// RCC PWR_LP Delay Control Register.
    pwrlpdlycr: ReadWrite<u32>,
    /// RCC MPU Reset Status Set Register.
    mp_rstssetr: ReadWrite<u32>,
    /// Reserved.
    _reserved14: [u32; 247],
    /// RCC MCO1 Configuration Register.
    mco1cfgr: ReadWrite<u32>,
    /// RCC MCO2 Configuration Register.
    mco2cfgr: ReadWrite<u32>,
    /// RCC Oscillator Clock Ready Register.
    ocrdyr: ReadWrite<u32>,
    /// RCC Debug Configuration Register.
    dbgcfgr: ReadWrite<u32>,
    /// Reserved.
    _reserved15: [u32; 4],
    /// RCC PLL3 Reference Clock Selection Register.
    rck3selr: ReadWrite<u32>,
    /// RCC PLL4 Reference Clock Selection Register.
    rck4selr: ReadWrite<u32>,
    /// RCC TIM Group 1 Prescaler Register.
    timg1prer: ReadWrite<u32>,
    /// RCC TIM Group 2 Prescaler Register.
    timg2prer: ReadWrite<u32>,
    /// RCC MCU Clock Prescaler Register.
    mcudivr: ReadWrite<u32>,
    /// RCC APB1 Clock Prescaler Register.
    apb1divr: ReadWrite<u32>,
    /// RCC APB2 Clock Prescaler Register.
    apb2divr: ReadWrite<u32>,
    /// RCC APB3 Clock Prescaler Register.
    apb3divr: ReadWrite<u32>,
    /// Reserved.
    _reserved16: [u32; 16],
    /// RCC PLL3 Control Register.
    pll3cr: ReadWrite<u32>,
    /// RCC PLL3 Configuration Register 1.
    pll3cfgr1: ReadWrite<u32>,
    /// RCC PLL3 Configuration Register 2.
    pll3cfgr2: ReadWrite<u32>,
    /// RCC PLL3 Fractional Register.
    pll3fracr: ReadWrite<u32>,
    /// RCC PLL3 Clock Spreading Generator Register.
    pll3csgr: ReadWrite<u32>,
    /// RCC PLL4 Control Register.
    pll4cr: ReadWrite<u32>,
    /// RCC PLL4 Configuration Register 1.
    pll4cfgr1: ReadWrite<u32>,
    /// RCC PLL4 Configuration Register 2.
    pll4cfgr2: ReadWrite<u32>,
    /// RCC PLL4 Fractional Register.
    pll4fracr: ReadWrite<u32>,
    /// RCC PLL4 Clock Spreading Generator Register.
    pll4csgr: ReadWrite<u32>,
    /// Reserved.
    _reserved17: [u32; 6],
    /// RCC I2C1 and I2C2 Kernel Clock Selection Register.
    i2c12ckselr: ReadWrite<u32>,
    /// RCC I2C3 and I2C5 Kernel Clock Selection Register.
    i2c35ckselr: ReadWrite<u32>,
    /// RCC SAI1 Kernel Clock Selection Register.
    sai1ckselr: ReadWrite<u32>,
    /// RCC SAI2 Kernel Clock Selection Register.
    sai2ckselr: ReadWrite<u32>,
    /// RCC SAI3 Kernel Clock Selection Register.
    sai3ckselr: ReadWrite<u32>,
    /// RCC SAI4 Kernel Clock Selection Register.
    sai4ckselr: ReadWrite<u32>,
    /// RCC SPI/I2S1 Kernel Clock Selection Register.
    spi2s1ckselr: ReadWrite<u32>,
    /// RCC SPI/I2S2 and SPI/I2S3 Kernel Clock Selection Register.
    spi2s23ckselr: ReadWrite<u32>,
    /// RCC SPI4 and SPI5 Kernel Clock Selection Register.
    spi45ckselr: ReadWrite<u32>,
    /// RCC USART6 Kernel Clock Selection Register.
    uart6ckselr: ReadWrite<u32>,
    /// RCC UART2 and UART4 Kernel Clock Selection Register.
    uart24ckselr: ReadWrite<u32>,
    /// RCC UART3 and UART5 Kernel Clock Selection Register.
    uart35ckselr: ReadWrite<u32>,
    /// RCC UART7 and UART8 Kernel Clock Selection Register.
    uart78ckselr: ReadWrite<u32>,
    /// RCC SDMMC1 and SDMMC2 Kernel Clock Selection Register.
    sdmmc12ckselr: ReadWrite<u32>,
    /// RCC SDMMC3 Kernel Clock Selection Register.
    sdmmc3ckselr: ReadWrite<u32>,
    /// RCC Ethernet Kernel Clock Selection Register.
    ethckselr: ReadWrite<u32>,
    /// RCC QUADSPI Kernel Clock Selection Register.
    qspickselr: ReadWrite<u32>,
    /// RCC FMC Kernel Clock Selection Register.
    fmcckselr: ReadWrite<u32>,
    /// Reserved.
    _reserved18: u32,
    /// RCC FDCAN Kernel Clock Selection Register.
    fdcanckselr: ReadWrite<u32>,
    /// Reserved.
    _reserved19: u32,
    /// RCC SPDIF Kernel Clock Selection Register.
    spdifckselr: ReadWrite<u32>,
    /// RCC CEC Kernel Clock Selection Register.
    cecckselr: ReadWrite<u32>,
    /// RCC USB Kernel Clock Selection Register.
    usbckselr: ReadWrite<u32>,
    /// RCC RNG2 Kernel Clock Selection Register.
    rng2ckselr: ReadWrite<u32>,
    /// RCC DSI Kernel Clock Selection Register.
    dsickselr: ReadWrite<u32>,
    /// RCC ADC Kernel Clock Selection Register.
    adcckselr: ReadWrite<u32>,
    /// RCC LPTIM4 and LPTIM5 Kernel Clock Selection Register.
    lptim45ckselr: ReadWrite<u32>,
    /// RCC LPTIM2 and LPTIM3 Kernel Clock Selection Register.
    lptim23ckselr: ReadWrite<u32>,
    /// RCC LPTIM1 Kernel Clock Selection Register.
    lptim1ckselr: ReadWrite<u32>,
    /// Reserved.
    _reserved20: [u32; 18],
    /// RCC APB1 Peripheral Reset Set Register.
    apb1rstsetr: ReadWrite<u32>,
    /// RCC APB1 Peripheral Reset Clear Register.
    apb1rstclrr: ReadWrite<u32>,
    /// RCC APB2 Peripheral Reset Set Register.
    apb2rstsetr: ReadWrite<u32>,
    /// RCC APB2 Peripheral Reset Clear Register.
    apb2rstclrr: ReadWrite<u32>,
    /// RCC APB3 Peripheral Reset Set Register.
    apb3rstsetr: ReadWrite<u32>,
    /// RCC APB3 Peripheral Reset Clear Register.
    apb3rstclrr: ReadWrite<u32>,
    /// RCC AHB2 Peripheral Reset Set Register.
    ahb2rstsetr: ReadWrite<u32>,
    /// RCC AHB2 Peripheral Reset Clear Register.
    ahb2rstclrr: ReadWrite<u32>,
    /// RCC AHB3 Peripheral Reset Set Register.
    ahb3rstsetr: ReadWrite<u32>,
    /// RCC AHB3 Peripheral Reset Clear Register.
    ahb3rstclrr: ReadWrite<u32>,
    /// RCC AHB4 Peripheral Reset Set Register.
    ahb4rstsetr: ReadWrite<u32>,
    /// RCC AHB4 Peripheral Reset Clear Register.
    ahb4rstclrr: ReadWrite<u32>,
    /// Reserved.
    _reserved21: [u32; 20],
    /// RCC APB1 Peripheral Enable for MPU Set Register.
    mp_apb1ensetr: ReadWrite<u32>,
    /// RCC APB1 Peripheral Enable for MPU Clear Register.
    mp_apb1enclrr: ReadWrite<u32>,
    /// RCC APB2 Peripheral Enable for MPU Set Register.
    mp_apb2ensetr: ReadWrite<u32>,
    /// RCC APB2 Peripheral Enable for MPU Clear Register.
    mp_apb2enclrr: ReadWrite<u32>,
    /// RCC APB3 Peripheral Enable for MPU Set Register.
    mp_apb3ensetr: ReadWrite<u32>,
    /// RCC APB3 Peripheral Enable for MPU Clear Register.
    mp_apb3enclrr: ReadWrite<u32>,
    /// RCC AHB2 Peripheral Enable for MPU Set Register.
    mp_ahb2ensetr: ReadWrite<u32>,
    /// RCC AHB2 Peripheral Enable for MPU Clear Register.
    mp_ahb2enclrr: ReadWrite<u32>,
    /// RCC AHB3 Peripheral Enable for MPU Set Register.
    mp_ahb3ensetr: ReadWrite<u32>,
    /// RCC AHB3 Peripheral Enable for MPU Clear Register.
    mp_ahb3enclrr: ReadWrite<u32>,
    /// RCC AHB4 Peripheral Enable for MPU Set Register.
    mp_ahb4ensetr: ReadWrite<u32>,
    /// RCC AHB4 Peripheral Enable for MPU Clear Register.
    mp_ahb4enclrr: ReadWrite<u32>,
    /// Reserved.
    _reserved22: [u32; 2],
    /// RCC MLAHB Peripheral Enable for MPU Set Register.
    mp_mlahbensetr: ReadWrite<u32>,
    /// RCC MLAHB Peripheral Enable for MPU Clear Register.
    mp_mlahbenclrr: ReadWrite<u32>,
    /// Reserved.
    _reserved23: [u32; 16],
    /// RCC APB1 Peripheral Enable for MCU Set Register.
    mc_apb1ensetr: ReadWrite<u32, MC_APB1ENSET::Register>,
    /// RCC APB1 Peripheral Enable for MCU Clear Register.
    mc_apb1enclrr: ReadWrite<u32, MC_APB1ENCLR::Register>,
    /// RCC APB2 Peripheral Enable for MCU Set Register.
    mc_apb2ensetr: ReadWrite<u32>,
    /// RCC APB2 Peripheral Enable for MCU Clear Register.
    mc_apb2enclrr: ReadWrite<u32>,
    /// RCC APB3 Peripheral Enable for MCU Set Register.
    mc_apb3ensetr: ReadWrite<u32>,
    /// RCC APB3 Peripheral Enable for MCU Clear Register.
    mc_apb3enclrr: ReadWrite<u32>,
    /// RCC AHB2 Peripheral Enable for MCU Set Register.
    mc_ahb2ensetr: ReadWrite<u32>,
    /// RCC AHB2 Peripheral Enable for MCU Clear Register.
    mc_ahb2enclrr: ReadWrite<u32>,
    /// RCC AHB3 Peripheral Enable for MCU Set Register.
    mc_ahb3ensetr: ReadWrite<u32>,
    /// RCC AHB3 Peripheral Enable for MCU Clear Register.
    mc_ahb3enclrr: ReadWrite<u32>,
    /// RCC AHB4 Peripheral Enable for MCU Set Register.
    mc_ahb4ensetr: ReadWrite<u32, MC_AHB4ENSET::Register>,
    /// RCC AHB4 Peripheral Enable for MCU Clear Register.
    mc_ahb4enclrr: ReadWrite<u32, MC_AHB4ENCLR::Register>,
    /// RCC AXIM Peripheral Enable for MCU Set Register.
    mc_aximensetr: ReadWrite<u32>,
    /// RCC AXIM Peripheral Enable for MCU Clear Register.
    mc_aximenclrr: ReadWrite<u32>,
    /// RCC MLAHB Peripheral Enable for MCU Set Register.
    mc_mlahbensetr: ReadWrite<u32>,
    /// RCC MLAHB Peripheral Enable for MCU Clear Register.
    mc_mlahbenclrr: ReadWrite<u32>,
    /// Reserved.
    _reserved24: [u32; 16],
    /// RCC APB1 Sleep Clock Enable for MPU Set Register.
    mp_apb1lpensetr: ReadWrite<u32>,
    /// RCC APB1 Sleep Clock Enable for MPU Clear Register.
    mp_apb1lpenclrr: ReadWrite<u32>,
    /// RCC APB2 Sleep Clock Enable for MPU Set Register.
    mp_apb2lpensetr: ReadWrite<u32>,
    /// RCC APB2 Sleep Clock Enable for MPU Clear Register.
    mp_apb2lpenclrr: ReadWrite<u32>,
    /// RCC APB3 Sleep Clock Enable for MPU Set Register.
    mp_apb3lpensetr: ReadWrite<u32>,
    /// RCC APB3 Sleep Clock Enable for MPU Clear Register.
    mp_apb3lpenclrr: ReadWrite<u32>,
    /// RCC AHB2 Sleep Clock Enable for MPU Set Register.
    mp_ahb2lpensetr: ReadWrite<u32>,
    /// RCC AHB2 Sleep Clock Enable for MPU Clear Register.
    mp_ahb2lpenclrr: ReadWrite<u32>,
    /// RCC AHB3 Sleep Clock Enable for MPU Set Register.
    mp_ahb3lpensetr: ReadWrite<u32>,
    /// RCC AHB3 Sleep Clock Enable for MPU Clear Register.
    mp_ahb3lpenclrr: ReadWrite<u32>,
    /// RCC AHB4 Sleep Clock Enable for MPU Set Register.
    mp_ahb4lpensetr: ReadWrite<u32>,
    /// RCC AHB4 Sleep Clock Enable for MPU Clear Register.
    mp_ahb4lpenclrr: ReadWrite<u32>,
    /// RCC AXIM Sleep Clock Enable for MPU Set Register.
    mp_aximlpensetr: ReadWrite<u32>,
    /// RCC AXIM Sleep Clock Enable for MPU Clear Register.
    mp_aximlpenclrr: ReadWrite<u32>,
    /// RCC MLAHB Sleep Clock Enable for MPU Set Register.
    mp_mlahblpensetr: ReadWrite<u32>,
    /// RCC MLAHB Sleep Clock Enable for MPU Clear Register.
    mp_mlahblpenclrr: ReadWrite<u32>,
    /// Reserved.
    _reserved25: [u32; 16],
    /// RCC APB1 Sleep Clock Enable for MCU Set Register.
    mc_apb1lpensetr: ReadWrite<u32>,
    /// RCC APB1 Sleep Clock Enable for MCU Clear Register.
    mc_apb1lpenclrr: ReadWrite<u32>,
    /// RCC APB2 Sleep Clock Enable for MCU Set Register.
    mc_apb2lpensetr: ReadWrite<u32>,
    /// RCC APB2 Sleep Clock Enable for MCU Clear Register.
    mc_apb2lpenclrr: ReadWrite<u32>,
    /// RCC APB3 Sleep Clock Enable for MCU Set Register.
    mc_apb3lpensetr: ReadWrite<u32>,
    /// RCC APB3 Sleep Clock Enable for MCU Clear Register.
    mc_apb3lpenclrr: ReadWrite<u32>,
    /// RCC AHB2 Sleep Clock Enable for MCU Set Register.
    mc_ahb2lpensetr: ReadWrite<u32>,
    /// RCC AHB2 Sleep Clock Enable for MCU Clear Register.
    mc_ahb2lpenclrr: ReadWrite<u32>,
    /// RCC AHB3 Sleep Clock Enable for MCU Set Register.
    mc_ahb3lpensetr: ReadWrite<u32>,
    /// RCC AHB3 Sleep Clock Enable for MCU Clear Register.
    mc_ahb3lpenclrr: ReadWrite<u32>,
    /// RCC AHB4 Sleep Clock Enable for MCU Set Register.
    mc_ahb4lpensetr: ReadWrite<u32>,
    /// RCC AHB4 Sleep Clock Enable for MCU Clear Register.
    mc_ahb4lpenclrr: ReadWrite<u32>,
    /// RCC AXIM Sleep Clock Enable for MCU Set Register.
    mc_aximlpensetr: ReadWrite<u32>,
    /// RCC AXIM Sleep Clock Enable for MCU Clear Register.
    mc_aximlpenclrr: ReadWrite<u32>,
    /// RCC MLAHB Sleep Clock Enable for MCU Set Register.
    mc_mlahblpensetr: ReadWrite<u32>,
    /// RCC MLAHB Sleep Clock Enable for MCU Clear Register.
    mc_mlahblpenclrr: ReadWrite<u32>,
    /// Reserved.
    _reserved26: [u32; 16],
    /// RCC MCU Reset Status Clear Register.
    mc_rstsclrr: ReadWrite<u32>,
    /// Reserved.
    _reserved27: [u32; 4],
    /// RCC Clock Source Interrupt Enable Register.
    mc_cier: ReadWrite<u32>,
    /// RCC Clock Source Interrupt Flag Register.
    mc_cifr: ReadWrite<u32>,
    /// Reserved.
    _reserved28: [u32; 246],
    /// RCC Version Register.
    verr: ReadWrite<u32>,
    /// RCC ID Register.
    ipidr: ReadWrite<u32>,
    /// RCC Size ID Register.
    sidr: ReadWrite<u32>,
}

register_bitfields![u32,
    TZCR [
        /// RCC TrustZone (secure) Enable.
        TZEN OFFSET(0) NUMBITS(1) [],
        /// Protection of mcuss_ck clock generation (secure) Enable.
        MCKPROT OFFSET(1) NUMBITS(1) []
    ],
    OCENSETR [
        /// Set HSION bit - Enabling HSI clock.
        HSION OFFSET(0) NUMBITS(1) [],
        /// Set HSIKERON bit - Enabling HSIKERON clock.
        HSIKERON OFFSET(1) NUMBITS(1) [],
        /// Set CSION bit - Enabling CSION clock.
        CSION OFFSET(4) NUMBITS(1) [],
        /// Set CSIKERON bit - Enabling CSIKERON clock.
        CSIKERON OFFSET(5) NUMBITS(1) [],
        /// Set DIGBYP bit - Enabling DIGBYP clock.
        DIGBYP OFFSET(7) NUMBITS(1) [],
        /// Set HSEON bit - Enabling HSEON clock.
        HSEON OFFSET(8) NUMBITS(1) [],
        /// Set HSEKERON bit - Enabling HSEKERON clock.
        HSEKERON OFFSET(9) NUMBITS(1) [],
        /// Set HSEBYP bit - Enabling HSEBYP clock.
        HSEBYP OFFSET(10) NUMBITS(1) [],
        /// Set HSECSSON bit - Enabling HSECSSON clock.
        HSECSSON OFFSET(11) NUMBITS(1) []
    ],
    OCENCLRR [
        /// Clears HSION bit - Disabling HSI clock.
        HSION OFFSET(0) NUMBITS(1) [],
        /// Clears HSIKERON bit - Disabling HSIKERON clock.
        HSIKERON OFFSET(1) NUMBITS(1) [],
        /// Clears CSION bit - Disabling CSION clock.
        CSION OFFSET(4) NUMBITS(1) [],
        /// Clears CSIKERON bit - Disabling CSIKERON clock.
        CSIKERON OFFSET(5) NUMBITS(1) [],
        /// Clears DIGBYP bit - Disabling DIGBYP clock.
        DIGBYP OFFSET(7) NUMBITS(1) [],
        /// Clears HSEON bit - Disabling HSEON clock.
        HSEON OFFSET(8) NUMBITS(1) [],
        /// Clears HSEKERON bit - Disabling HSEKERON clock.
        HSEKERON OFFSET(9) NUMBITS(1) [],
        /// Clears HSEBYP bit - Disabling HSEBYP clock.
        HSEBYP OFFSET(10) NUMBITS(1) []
    ],
    HSICFGR [
        /// HSI clock divider.
        HSIDIV OFFSET(0) NUMBITS(2) [],
        /// HSI clock trimming.
        HSITRIM OFFSET(8) NUMBITS(7) [],
        /// HSI clock calibration.
        HSICAL OFFSET(16) NUMBITS(12) []
    ],
    CSICFGR [
        /// CSI clock trimming.
        CSITRIM OFFSET(8) NUMBITS(5) [],
        /// CSI clock calibration.
        CSICAL OFFSET(16) NUMBITS(8) []
    ],
    MPCKSELR [
        /// MPU clock source selection.
        MPUSRC OFFSET(0) NUMBITS(2) [],
        /// MPU clock source selection update ready.
        MPUSRCRDY OFFSET(31) NUMBITS(1) []
    ],
    ASSCKSELR [
        /// AXI clock source selection.
        AXISSRC OFFSET(0) NUMBITS(3) [],
        /// AXI clock source selection update ready.
        AXISSRCRDY OFFSET(31) NUMBITS(1) []
    ],
    RCK12SELR [
        /// PLL1 and PLL2 reference clock source selection.
        PLL12SRC OFFSET(0) NUMBITS(2) [],
        /// PLL1 and PLL2 source selection update ready.
        PLL12SRCRDY OFFSET(31) NUMBITS(1) []
    ],
    MPCKDIVR [
        /// MPU clock divider.
        MPUDIV OFFSET(0) NUMBITS(3) [],
        /// MPU clock divider update ready.
        MPUDIVRDY OFFSET(31) NUMBITS(1) []
    ],
    AXIDIVR [
        /// AXI clock divider.
        AXIDIV OFFSET(0) NUMBITS(3) [],
        /// AXI clock divider update ready.
        AXIDIVRDY OFFSET(31) NUMBITS(1) []
    ],
    APB4DIVR [
        /// APB4 clock divider.
        APB4DIV OFFSET(0) NUMBITS(3) [],
        /// APB4 clock divider update ready.
        APB4DIVRDY OFFSET(31) NUMBITS(1) []
    ],
    APB5DIVR [
        /// APB5 clock divider.
        APB5DIV OFFSET(0) NUMBITS(3) [],
        /// APB5 clock divider update ready.
        APB5DIVRDY OFFSET(31) NUMBITS(1) []
    ],
    RTCDIVR [
        /// RTC clock divider.
        RTCDIV OFFSET(0) NUMBITS(6) []
    ],
    MSSCKSELR [
        /// MCU subsystem clock source selection.
        MCUSSRC OFFSET(0) NUMBITS(2) [],
        /// MCU subsystem clock source selection update ready.
        MCUSSRCRDY OFFSET(31) NUMBITS(1) []
    ],
    PLL1CR [
        /// PLL1 enable.
        PLLON OFFSET(0) NUMBITS(1) [],
        /// PLL1 ready.
        PLL1RDY OFFSET(1) NUMBITS(1) [],
        /// Spread-spectrum control.
        SSCG_CTRL OFFSET(2) NUMBITS(1) [],
        /// PLL1 DIVP output enable.
        DIVPEN OFFSET(4) NUMBITS(1) [],
        /// PLL1 DIVQ output enable.
        DIVQEN OFFSET(5) NUMBITS(1) [],
        /// PLL1 DIVR output enable.
        DIVREN OFFSET(6) NUMBITS(1) []
    ],
    PLL1CFGR1 [
        /// PLL1 multiplication factor.
        DIVN OFFSET(0) NUMBITS(9) [],
        /// PLL1 input divider.
        DIVM1 OFFSET(16) NUMBITS(6) []
    ],
    PLL1CFGR2 [
        /// PLL1 DIVP divider.
        DIVP OFFSET(0) NUMBITS(7) [],
        /// PLL1 DIVQ divider.
        DIVQ OFFSET(8) NUMBITS(7) [],
        /// PLL1 DIVR divider.
        DIVR OFFSET(16) NUMBITS(7) []
    ],
    PLL1FRACR [
        /// PLL1 fractional value.
        FRACV OFFSET(3) NUMBITS(13) [],
        /// PLL1 fractional latch enable.
        FRACLE OFFSET(16) NUMBITS(1) []
    ],
    PLL1CSGR [
        /// PLL1 modulation period.
        MOD_PER OFFSET(0) NUMBITS(13) [],
        /// Disable triangle downspread.
        TPDFN_DIS OFFSET(13) NUMBITS(1) [],
        /// Disable random spread.
        RPDFN_DIS OFFSET(14) NUMBITS(1) [],
        /// Spread-spectrum mode.
        SSCG_MODE OFFSET(15) NUMBITS(1) [],
        /// Increment step.
        INC_STEP OFFSET(16) NUMBITS(15) []
    ],
    PLL2CR [
        /// PLL2 enable.
        PLLON OFFSET(0) NUMBITS(1) [],
        /// PLL2 ready.
        PLL2RDY OFFSET(1) NUMBITS(1) [],
        /// Spread-spectrum control.
        SSCG_CTRL OFFSET(2) NUMBITS(1) [],
        /// PLL2 DIVP output enable.
        DIVPEN OFFSET(4) NUMBITS(1) [],
        /// PLL2 DIVQ output enable.
        DIVQEN OFFSET(5) NUMBITS(1) [],
        /// PLL2 DIVR output enable.
        DIVREN OFFSET(6) NUMBITS(1) []
    ],
    PLL2CFGR1 [
        /// PLL2 multiplication factor.
        DIVN OFFSET(0) NUMBITS(9) [],
        /// PLL2 input divider.
        DIVM2 OFFSET(16) NUMBITS(6) []
    ],
    PLL2CFGR2 [
        /// PLL2 DIVP divider.
        DIVP OFFSET(0) NUMBITS(7) [],
        /// PLL2 DIVQ divider.
        DIVQ OFFSET(8) NUMBITS(7) [],
        /// PLL2 DIVR divider.
        DIVR OFFSET(16) NUMBITS(7) []
    ],
    PLL2FRACR [
        /// PLL2 fractional value.
        FRACV OFFSET(3) NUMBITS(13) [],
        /// PLL2 fractional latch enable.
        FRACLE OFFSET(16) NUMBITS(1) []
    ],
    PLL2CSGR [
        /// PLL2 modulation period.
        MOD_PER OFFSET(0) NUMBITS(13) [],
        /// Disable triangle downspread.
        TPDFN_DIS OFFSET(13) NUMBITS(1) [],
        /// Disable random spread.
        RPDFN_DIS OFFSET(14) NUMBITS(1) [],
        /// Spread-spectrum mode.
        SSCG_MODE OFFSET(15) NUMBITS(1) [],
        /// Increment step.
        INC_STEP OFFSET(16) NUMBITS(15) []
    ],
    MC_AHB4ENSET [
        /// GPIOA peripheral enable.
        GPIOAEN OFFSET(0) NUMBITS(1) [],
        /// GPIOB peripheral enable.
        GPIOBEN OFFSET(1) NUMBITS(1) [],
        /// GPIOC peripheral enable.
        GPIOCEN OFFSET(2) NUMBITS(1) [],
        /// GPIOD peripheral enable.
        GPIODEN OFFSET(3) NUMBITS(1) [],
        /// GPIOE peripheral enable.
        GPIOEEN OFFSET(4) NUMBITS(1) [],
        /// GPIOF peripheral enable.
        GPIOFEN OFFSET(5) NUMBITS(1) [],
        /// GPIOG peripheral enable.
        GPIOGEN OFFSET(6) NUMBITS(1) [],
        /// GPIOH peripheral enable.
        GPIOHEN OFFSET(7) NUMBITS(1) [],
        /// GPIOI peripheral enable.
        GPIOIEN OFFSET(8) NUMBITS(1) [],
        /// GPIOJ peripheral enable.
        GPIOJEN OFFSET(9) NUMBITS(1) [],
        /// GPIOK peripheral enable.
        GPIOKEN OFFSET(10) NUMBITS(1) [],
    ],
    MC_AHB4ENCLR [
        /// GPIOA peripheral enable.
        GPIOAEN OFFSET(0) NUMBITS(1) [],
        /// GPIOB peripheral enable.
        GPIOBEN OFFSET(1) NUMBITS(1) [],
        /// GPIOC peripheral enable.
        GPIOCEN OFFSET(2) NUMBITS(1) [],
        /// GPIOD peripheral enable.
        GPIODEN OFFSET(3) NUMBITS(1) [],
        /// GPIOE peripheral enable.
        GPIOEEN OFFSET(4) NUMBITS(1) [],
        /// GPIOF peripheral enable.
        GPIOFEN OFFSET(5) NUMBITS(1) [],
        /// GPIOG peripheral enable.
        GPIOGEN OFFSET(6) NUMBITS(1) [],
        /// GPIOH peripheral enable.
        GPIOHEN OFFSET(7) NUMBITS(1) [],
        /// GPIOI peripheral enable.
        GPIOIEN OFFSET(8) NUMBITS(1) [],
        /// GPIOJ peripheral enable.
        GPIOJEN OFFSET(9) NUMBITS(1) [],
        /// GPIOK peripheral enable.
        GPIOKEN OFFSET(10) NUMBITS(1) [],
    ],

    MC_APB1ENSET [
        /// TIM2 peripheral enable.
        TIM2EN OFFSET(0) NUMBITS(1) [],
        /// TIM3 peripheral enable.
        TIM3EN OFFSET(1) NUMBITS(1) [],
        /// TIM4 peripheral enable.
        TIM4EN OFFSET(2) NUMBITS(1) [],
        /// TIM5 peripheral enable.
        TIM5EN OFFSET(3) NUMBITS(1) [],
        /// TIM6 peripheral enable.
        TIM6EN OFFSET(4) NUMBITS(1) [],
        /// TIM7 peripheral enable.
        TIM7EN OFFSET(5) NUMBITS(1) [],
        /// TIM12 peripheral enable.
        TIM12EN OFFSET(6) NUMBITS(1) [],
        /// TIM13 peripheral enable.
        TIM13EN OFFSET(7) NUMBITS(1) [],
        // TODO incomplete
    ],
    MC_APB1ENCLR [
        /// TIM2 peripheral enable.
        TIM2EN OFFSET(0) NUMBITS(1) [],
        /// TIM3 peripheral enable.
        TIM3EN OFFSET(1) NUMBITS(1) [],
        /// TIM4 peripheral enable.
        TIM4EN OFFSET(2) NUMBITS(1) [],
        /// TIM5 peripheral enable.
        TIM5EN OFFSET(3) NUMBITS(1) [],
        /// TIM6 peripheral enable.
        TIM6EN OFFSET(4) NUMBITS(1) [],
        /// TIM7 peripheral enable.
        TIM7EN OFFSET(5) NUMBITS(1) [],
        // todo incomplete
    ],
    // Legacy STM32F-style RCC definitions removed.
];

const RCC_BASE: StaticRef<RccRegisters> =
    unsafe { StaticRef::new(0x50000000 as *const RccRegisters) };

pub enum RtcClockSource {
    LSE,
    CSS,
}

pub struct Rcc {
    registers: StaticRef<RccRegisters>,
}

impl Rcc {
    const MC_APB1_UART4EN: u32 = 1 << 16;
    const MC_APB5_USART1EN: u32 = 1 << 4;
    pub fn new() -> Self {
        let rcc = Self {
            registers: RCC_BASE,
        };
        rcc.init();
        rcc
    }

    // Some clocks need to be initialized before use
    fn init(&self) {
        // (todo) Initialize the clocks.
    }

    fn init_pll_clock(&self) {
        unimplemented!("init_pll_clock");
    }

    // Get the current system clock source
    pub(crate) fn get_sys_clock_source(&self) -> SysClockSource {
        match self.registers.mssckselr.read(MSSCKSELR::MCUSSRC) {
            0b00 => SysClockSource::HSI,
            0b01 => SysClockSource::HSE,
            _ => SysClockSource::PLL,
        }
    }

    // Set the system clock source
    // The source must be enabled
    // NOTE: The flash latency also needs to be configured when changing the system clock frequency
    pub(crate) fn set_sys_clock_source(&self, source: SysClockSource) {
        let _ = source;
        unimplemented!("set_sys_clock_source");
    }

    pub(crate) fn is_hsi_clock_system_clock(&self) -> bool {
        unimplemented!("is_hsi_clock_system_clock");
    }

    pub(crate) fn is_hse_clock_system_clock(&self) -> bool {
        unimplemented!("is_hse_clock_system_clock");
    }

    /* HSI clock */
    // The HSI clock must not be configured as the system clock, either directly or indirectly.
    pub(crate) fn disable_hsi_clock(&self) {
        self.registers.ocenclrr.modify(OCENCLRR::HSION::SET);
    }

    pub(crate) fn enable_hsi_clock(&self) {
        self.registers.ocensetr.modify(OCENSETR::HSION::SET);
    }

    pub(crate) fn is_enabled_hsi_clock(&self) -> bool {
        self.registers.ocensetr.is_set(OCENSETR::HSION)
    }

    // Indicates whether the HSI oscillator is stable
    pub(crate) fn is_ready_hsi_clock(&self) -> bool {
        unimplemented!("is_ready_hsi_clock");
    }

    /* HSE clock */
    pub(crate) fn disable_hse_clock(&self) {
        unimplemented!("disable_hse_clock");
    }

    pub(crate) fn enable_hse_clock_bypass(&self) {
        unimplemented!("enable_hse_clock_bypass");
    }

    pub(crate) fn enable_hse_clock(&self) {
        unimplemented!("enable_hse_clock");
    }

    pub(crate) fn is_enabled_hse_clock(&self) -> bool {
        unimplemented!("is_enabled_hse_clock");
    }

    // Indicates whether the HSE oscillator is stable
    pub(crate) fn is_ready_hse_clock(&self) -> bool {
        unimplemented!("is_ready_hse_clock");
    }

    /* Main PLL clock*/

    // The main PLL clock must not be configured as the system clock.
    pub(crate) fn disable_pll_clock(&self) {
        unimplemented!("disable_pll_clock");
    }

    pub(crate) fn enable_pll_clock(&self) {
        unimplemented!("enable_pll_clock");
    }

    pub(crate) fn is_enabled_pll_clock(&self) -> bool {
        unimplemented!("is_enabled_pll_clock");
    }

    // The PLL clock is locked when its signal is stable
    pub(crate) fn is_locked_pll_clock(&self) -> bool {
        unimplemented!("is_locked_pll_clock");
    }

    pub(crate) fn get_pll_clocks_source(&self) -> PllSource {
        unimplemented!("get_pll_clocks_source");
    }

    // This method must be called only when all PLL clocks are disabled
    pub(crate) fn set_pll_clocks_source(&self, source: PllSource) {
        let _ = source;
        unimplemented!("set_pll_clocks_source");
    }

    pub(crate) fn get_pll_clocks_m_divider(&self) -> PLLM {
        unimplemented!("get_pll_clocks_m_divider");
    }

    // This method must be called only when all PLL clocks are disabled
    pub(crate) fn set_pll_clocks_m_divider(&self, m: PLLM) {
        let _ = m;
        unimplemented!("set_pll_clocks_m_divider");
    }

    pub(crate) fn get_pll_clock_n_multiplier(&self) -> usize {
        unimplemented!("get_pll_clock_n_multiplier");
    }

    // This method must be called only if the main PLL clock is disabled
    pub(crate) fn set_pll_clock_n_multiplier(&self, n: usize) {
        let _ = n;
        unimplemented!("set_pll_clock_n_multiplier");
    }

    pub(crate) fn get_pll_clock_p_divider(&self) -> PLLP {
        unimplemented!("get_pll_clock_p_divider");
    }

    // This method must be called only if the main PLL clock is disabled
    pub(crate) fn set_pll_clock_p_divider(&self, p: PLLP) {
        let _ = p;
        unimplemented!("set_pll_clock_p_divider");
    }

    pub(crate) fn _get_pll_clock_q_divider(&self) -> PLLQ {
        unimplemented!("_get_pll_clock_q_divider");
    }

    // This method must be called only if the main PLL clock is disabled
    pub(crate) fn set_pll_clock_q_divider(&self, q: PLLQ) {
        let _ = q;
        unimplemented!("set_pll_clock_q_divider");
    }

    /* AHB prescaler */

    pub(crate) fn set_ahb_prescaler(&self, ahb_prescaler: AHBPrescaler) {
        let _ = ahb_prescaler;
        unimplemented!("set_ahb_prescaler");
    }

    pub(crate) fn get_ahb_prescaler(&self) -> AHBPrescaler {
        match self.registers.mcudivr.get() & 0b1111 {
            0b1000 => AHBPrescaler::DivideBy2,
            0b1001 => AHBPrescaler::DivideBy4,
            0b1010 => AHBPrescaler::DivideBy8,
            0b1011 => AHBPrescaler::DivideBy16,
            0b1100 => AHBPrescaler::DivideBy64,
            0b1101 => AHBPrescaler::DivideBy128,
            0b1110 => AHBPrescaler::DivideBy256,
            0b1111 => AHBPrescaler::DivideBy512,
            _ => AHBPrescaler::DivideBy1,
        }
    }

    /* APB1 prescaler */

    pub(crate) fn set_apb1_prescaler(&self, apb1_prescaler: APBPrescaler) {
        let _ = apb1_prescaler;
        unimplemented!("set_apb1_prescaler");
    }

    pub(crate) fn get_apb1_prescaler(&self) -> APBPrescaler {
        match self.registers.apb1divr.get() & 0b111 {
            0b100 => APBPrescaler::DivideBy2,
            0b101 => APBPrescaler::DivideBy4,
            0b110 => APBPrescaler::DivideBy8,
            0b111 => APBPrescaler::DivideBy16,
            _ => APBPrescaler::DivideBy1,
        }
    }

    /* APB2 prescaler */

    pub(crate) fn set_apb2_prescaler(&self, apb2_prescaler: APBPrescaler) {
        let _ = apb2_prescaler;
        unimplemented!("set_apb2_prescaler");
    }

    pub(crate) fn get_apb2_prescaler(&self) -> APBPrescaler {
        unimplemented!("get_apb2_prescaler");
    }

    pub(crate) fn set_mco1_clock_source(&self, source: MCO1Source) {
        let _ = source;
        unimplemented!("set_mco1_clock_source");
    }

    pub(crate) fn get_mco1_clock_source(&self) -> MCO1Source {
        unimplemented!("get_mco1_clock_source");
    }

    pub(crate) fn set_mco1_clock_divider(&self, divider: MCO1Divider) {
        let _ = divider;
        unimplemented!("set_mco1_clock_divider");
    }

    pub(crate) fn get_mco1_clock_divider(&self) -> MCO1Divider {
        unimplemented!("get_mco1_clock_divider");
    }

    pub(crate) fn configure_rng_clock(&self) {
        unimplemented!("configure_rng_clock");
    }

    // I2C1 clock

    pub(crate) fn is_enabled_i2c1_clock(&self) -> bool {
        unimplemented!("is_enabled_i2c1_clock");
    }

    pub(crate) fn enable_i2c1_clock(&self) {
        unimplemented!("enable_i2c1_clock");
    }

    pub(crate) fn disable_i2c1_clock(&self) {
        unimplemented!("disable_i2c1_clock");
    }

    // SPI3 clock

    pub(crate) fn is_enabled_spi3_clock(&self) -> bool {
        unimplemented!("is_enabled_spi3_clock");
    }

    pub(crate) fn enable_spi3_clock(&self) {
        unimplemented!("enable_spi3_clock");
    }

    pub(crate) fn disable_spi3_clock(&self) {
        unimplemented!("disable_spi3_clock");
    }

    // TIM2 clock
    pub(crate) fn is_enabled_tim_pre(&self) -> bool {
        (self.registers.timg1prer.get() & 0x1) != 0
    }

    pub(crate) fn is_enabled_tim2_clock(&self) -> bool {
        self.registers.mc_apb1ensetr.is_set(MC_APB1ENSET::TIM2EN)
    }

    pub(crate) fn enable_tim2_clock(&self) {
        self.registers
            .mc_apb1ensetr
            .modify(MC_APB1ENSET::TIM2EN::SET);
    }

    pub(crate) fn disable_tim2_clock(&self) {
        self.registers
            .mc_apb1enclrr
            .modify(MC_APB1ENCLR::TIM2EN::CLEAR);
    }

    // SYSCFG clock

    pub(crate) fn is_enabled_syscfg_clock(&self) -> bool {
        unimplemented!("is_enabled_syscfg_clock");
    }

    pub(crate) fn enable_syscfg_clock(&self) {
        unimplemented!("enable_syscfg_clock");
    }

    pub(crate) fn disable_syscfg_clock(&self) {
        unimplemented!("disable_syscfg_clock");
    }

    // DMA1 clock

    pub(crate) fn is_enabled_dma1_clock(&self) -> bool {
        unimplemented!("is_enabled_dma1_clock");
    }

    pub(crate) fn enable_dma1_clock(&self) {
        unimplemented!("enable_dma1_clock");
    }

    pub(crate) fn disable_dma1_clock(&self) {
        unimplemented!("disable_dma1_clock");
    }

    // DMA2 clock
    pub(crate) fn is_enabled_dma2_clock(&self) -> bool {
        unimplemented!("is_enabled_dma2_clock");
    }

    pub(crate) fn enable_dma2_clock(&self) {
        unimplemented!("enable_dma2_clock");
    }

    pub(crate) fn disable_dma2_clock(&self) {
        unimplemented!("disable_dma2_clock");
    }

    // GPIOH clock

    pub(crate) fn is_enabled_gpioh_clock(&self) -> bool {
        unimplemented!("is_enabled_gpioh_clock");
    }

    pub(crate) fn enable_gpioh_clock(&self) {
        unimplemented!("enable_gpioh_clock");
    }

    pub(crate) fn disable_gpioh_clock(&self) {
        unimplemented!("disable_gpioh_clock");
    }

    // GPIOG clock

    pub(crate) fn is_enabled_gpiog_clock(&self) -> bool {
        self.registers.mc_ahb4ensetr.is_set(MC_AHB4ENSET::GPIOGEN)
    }

    pub(crate) fn enable_gpiog_clock(&self) {
        self.registers
            .mc_ahb4ensetr
            .modify(MC_AHB4ENSET::GPIOGEN::SET);
    }

    pub(crate) fn disable_gpiog_clock(&self) {
        self.registers
            .mc_ahb4enclrr
            .modify(MC_AHB4ENCLR::GPIOGEN::SET);
    }

    // GPIOF clock

    pub(crate) fn is_enabled_gpiof_clock(&self) -> bool {
        unimplemented!("is_enabled_gpiof_clock");
    }

    pub(crate) fn enable_gpiof_clock(&self) {
        unimplemented!("enable_gpiof_clock");
    }

    pub(crate) fn disable_gpiof_clock(&self) {
        unimplemented!("disable_gpiof_clock");
    }

    // GPIOE clock

    pub(crate) fn is_enabled_gpioe_clock(&self) -> bool {
        unimplemented!("is_enabled_gpioe_clock");
    }

    pub(crate) fn enable_gpioe_clock(&self) {
        unimplemented!("enable_gpioe_clock");
    }

    pub(crate) fn disable_gpioe_clock(&self) {
        unimplemented!("disable_gpioe_clock");
    }

    // GPIOD clock

    pub(crate) fn is_enabled_gpiod_clock(&self) -> bool {
        self.registers.mc_ahb4ensetr.is_set(MC_AHB4ENSET::GPIODEN)
    }

    pub(crate) fn enable_gpiod_clock(&self) {
        self.registers
            .mc_ahb4ensetr
            .modify(MC_AHB4ENSET::GPIODEN::SET);
    }

    pub(crate) fn disable_gpiod_clock(&self) {
        self.registers
            .mc_ahb4enclrr
            .modify(MC_AHB4ENCLR::GPIODEN::SET);
    }

    // GPIOC clock

    pub(crate) fn is_enabled_gpioc_clock(&self) -> bool {
        unimplemented!("is_enabled_gpioc_clock");
    }

    pub(crate) fn enable_gpioc_clock(&self) {
        unimplemented!("enable_gpioc_clock");
    }

    pub(crate) fn disable_gpioc_clock(&self) {
        unimplemented!("disable_gpioc_clock");
    }

    // GPIOB clock

    pub(crate) fn is_enabled_gpiob_clock(&self) -> bool {
        self.registers.mc_ahb4ensetr.is_set(MC_AHB4ENSET::GPIOBEN)
    }

    pub(crate) fn enable_gpiob_clock(&self) {
        self.registers
            .mc_ahb4ensetr
            .modify(MC_AHB4ENSET::GPIOBEN::SET);
    }

    pub(crate) fn disable_gpiob_clock(&self) {
        self.registers
            .mc_ahb4enclrr
            .modify(MC_AHB4ENCLR::GPIOBEN::SET);
    }

    // GPIOA clock

    pub(crate) fn is_enabled_gpioa_clock(&self) -> bool {
        self.registers.mc_ahb4ensetr.is_set(MC_AHB4ENSET::GPIOAEN)
    }

    pub(crate) fn enable_gpioa_clock(&self) {
        self.registers
            .mc_ahb4ensetr
            .modify(MC_AHB4ENSET::GPIOAEN::SET);
    }

    pub(crate) fn disable_gpioa_clock(&self) {
        self.registers
            .mc_ahb4enclrr
            .modify(MC_AHB4ENCLR::GPIOAEN::SET);
    }

    // FMC

    pub(crate) fn is_enabled_fmc_clock(&self) -> bool {
        unimplemented!("is_enabled_fmc_clock");
    }

    pub(crate) fn enable_fmc_clock(&self) {
        unimplemented!("enable_fmc_clock");
    }

    pub(crate) fn disable_fmc_clock(&self) {
        unimplemented!("disable_fmc_clock");
    }

    // USART1 clock
    pub(crate) fn is_enabled_usart1_clock(&self) -> bool {
        (self.registers.mc_apb5ensetr.get() & Self::MC_APB5_USART1EN) != 0
    }

    pub(crate) fn enable_usart1_clock(&self) {
        self.registers.mc_apb5ensetr.set(Self::MC_APB5_USART1EN);
    }

    pub(crate) fn disable_usart1_clock(&self) {
        self.registers.mc_apb5enclrr.set(Self::MC_APB5_USART1EN);
    }

    // USART2 clock

    pub(crate) fn is_enabled_usart2_clock(&self) -> bool {
        unimplemented!("is_enabled_usart2_clock");
    }

    pub(crate) fn enable_usart2_clock(&self) {
        unimplemented!("enable_usart2_clock");
    }

    pub(crate) fn disable_usart2_clock(&self) {
        unimplemented!("disable_usart2_clock");
    }

    // USART3 clock

    pub(crate) fn is_enabled_usart3_clock(&self) -> bool {
        unimplemented!("is_enabled_usart3_clock");
    }

    pub(crate) fn enable_usart3_clock(&self) {
        unimplemented!("enable_usart3_clock");
    }

    pub(crate) fn disable_usart3_clock(&self) {
        unimplemented!("disable_usart3_clock");
    }

    // UART4 clock
    pub(crate) fn is_enabled_uart4_clock(&self) -> bool {
        (self.registers.mc_apb1ensetr.get() & Self::MC_APB1_UART4EN) != 0
    }

    pub(crate) fn enable_uart4_clock(&self) {
        self.registers.mc_apb1ensetr.set(Self::MC_APB1_UART4EN);
    }

    pub(crate) fn disable_uart4_clock(&self) {
        self.registers.mc_apb1enclrr.set(Self::MC_APB1_UART4EN);
    }

    // ADC1 clock

    pub(crate) fn is_enabled_adc1_clock(&self) -> bool {
        unimplemented!("is_enabled_adc1_clock");
    }

    pub(crate) fn enable_adc1_clock(&self) {
        unimplemented!("enable_adc1_clock");
    }

    pub(crate) fn disable_adc1_clock(&self) {
        unimplemented!("disable_adc1_clock");
    }

    // DAC clock

    pub(crate) fn is_enabled_dac_clock(&self) -> bool {
        unimplemented!("is_enabled_dac_clock");
    }

    pub(crate) fn enable_dac_clock(&self) {
        unimplemented!("enable_dac_clock");
    }

    pub(crate) fn disable_dac_clock(&self) {
        unimplemented!("disable_dac_clock");
    }

    // RNG clock

    pub(crate) fn is_enabled_rng_clock(&self) -> bool {
        unimplemented!("is_enabled_rng_clock");
    }

    pub(crate) fn enable_rng_clock(&self) {
        unimplemented!("enable_rng_clock");
    }

    pub(crate) fn disable_rng_clock(&self) {
        unimplemented!("disable_rng_clock");
    }

    // OTGFS clock

    pub(crate) fn is_enabled_otgfs_clock(&self) -> bool {
        unimplemented!("is_enabled_otgfs_clock");
    }

    pub(crate) fn enable_otgfs_clock(&self) {
        unimplemented!("enable_otgfs_clock");
    }

    pub(crate) fn disable_otgfs_clock(&self) {
        unimplemented!("disable_otgfs_clock");
    }

    // CAN1 clock

    pub(crate) fn is_enabled_can1_clock(&self) -> bool {
        unimplemented!("is_enabled_can1_clock");
    }

    pub(crate) fn enable_can1_clock(&self) {
        unimplemented!("enable_can1_clock");
    }

    pub(crate) fn disable_can1_clock(&self) {
        unimplemented!("disable_can1_clock");
    }

    // RTC clock
    pub(crate) fn source_into_u32(source: RtcClockSource) -> u32 {
        let _ = source;
        unimplemented!("source_into_u32");
    }

    pub(crate) fn enable_lsi_clock(&self) {
        unimplemented!("enable_lsi_clock");
    }

    pub(crate) fn is_enabled_pwr_clock(&self) -> bool {
        unimplemented!("is_enabled_pwr_clock");
    }

    pub(crate) fn enable_pwr_clock(&self) {
        unimplemented!("enable_pwr_clock");
    }

    pub(crate) fn disable_pwr_clock(&self) {
        unimplemented!("disable_pwr_clock");
    }

    pub(crate) fn is_enabled_rtc_clock(&self) -> bool {
        unimplemented!("is_enabled_rtc_clock");
    }

    pub(crate) fn enable_rtc_clock(&self, source: RtcClockSource) {
        unimplemented!("enable_rtc_clock");
    }

    pub(crate) fn disable_rtc_clock(&self) {
        unimplemented!("disable_rtc_clock");
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub(crate) enum PLLP {
    DivideBy2 = 0b00,
    DivideBy4 = 0b01,
    DivideBy6 = 0b10,
    DivideBy8 = 0b11,
}

impl From<PLLP> for usize {
    // (variant_value + 1) * 2 = X for X in DivideByX
    fn from(item: PLLP) -> Self {
        (item as usize + 1) << 1
    }
}

// Theoretically, the PLLM value can range from 2 to 63. However, the current implementation was
// designed to support 1MHz frequency precision. In a future update, PLLM will become a usize.
#[allow(dead_code)]
pub(crate) enum PLLM {
    DivideBy8 = 8,
    DivideBy16 = 16,
}

#[derive(Copy, Clone, Debug, PartialEq)]
// Due to the restricted values for PLLM, PLLQ 2/10-15 values are meaningless.
pub(crate) enum PLLQ {
    DivideBy3 = 3,
    DivideBy4,
    DivideBy5,
    DivideBy6,
    DivideBy7,
    DivideBy8,
    DivideBy9,
}

/// Clock sources for the CPU
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum SysClockSource {
    HSI = 0b00,
    HSE = 0b01,
    PLL = 0b10,
    // NOTE: not all STM32F4xx boards support this source.
    //PPLLR = 0b11, Uncomment this when support for PPLLR is added
}

pub enum PllSource {
    HSI = 0b0,
    HSE = 0b1,
}

pub enum MCO1Source {
    HSI = 0b00,
    //LSE = 0b01, // When support for LSE is added, uncomment this
    HSE = 0b10,
    PLL = 0b11,
}

pub enum MCO1Divider {
    DivideBy1 = 0b000,
    DivideBy2 = 0b100,
    DivideBy3 = 0b101,
    DivideBy4 = 0b110,
    DivideBy5 = 0b111,
}

/// HSE Mode
#[derive(PartialEq)]
pub enum HseMode {
    BYPASS,
    CRYSTAL,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum AHBPrescaler {
    DivideBy1 = 0b0000,
    DivideBy2 = 0b1000,
    DivideBy4 = 0b1001,
    DivideBy8 = 0b1010,
    DivideBy16 = 0b1011,
    DivideBy64 = 0b1100,
    DivideBy128 = 0b1101,
    DivideBy256 = 0b1110,
    DivideBy512 = 0b1111,
}

impl From<AHBPrescaler> for usize {
    fn from(item: AHBPrescaler) -> usize {
        match item {
            AHBPrescaler::DivideBy1 => 1,
            AHBPrescaler::DivideBy2 => 2,
            AHBPrescaler::DivideBy4 => 4,
            AHBPrescaler::DivideBy8 => 8,
            AHBPrescaler::DivideBy16 => 16,
            AHBPrescaler::DivideBy64 => 64,
            AHBPrescaler::DivideBy128 => 128,
            AHBPrescaler::DivideBy256 => 256,
            AHBPrescaler::DivideBy512 => 512,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum APBPrescaler {
    DivideBy1 = 0b000, // No division
    DivideBy2 = 0b100,
    DivideBy4 = 0b101,
    DivideBy8 = 0b110,
    DivideBy16 = 0b111,
}

impl From<APBPrescaler> for usize {
    fn from(item: APBPrescaler) -> Self {
        match item {
            APBPrescaler::DivideBy1 => 1,
            APBPrescaler::DivideBy2 => 2,
            APBPrescaler::DivideBy4 => 4,
            APBPrescaler::DivideBy8 => 8,
            APBPrescaler::DivideBy16 => 16,
        }
    }
}
