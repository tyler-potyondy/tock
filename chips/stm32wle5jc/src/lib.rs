#![no_std]
pub use stm32wle5xx::{chip, clocks, gpio, rcc};

pub mod chip_specs;
pub mod interrupt_service;

pub unsafe fn init() {
    stm32wle5xx::init();
}
