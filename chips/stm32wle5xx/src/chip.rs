// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Chip trait setup.

use core::fmt::Write;
use cortexm4::{CortexM4, CortexMVariant};
use kernel::platform::chip::Chip;
use kernel::platform::chip::InterruptService;

use crate::chip_specific::chip_specs::ChipSpecs as ChipSpecsTrait;

pub struct Stm32wle5xx<'a, I: InterruptService + 'a> {
    mpu: cortexm4::mpu::MPU,
    userspace_kernel_boundary: cortexm4::syscall::SysCall,
    interrupt_service: &'a I,
}

pub struct Stm32wle5xxDefaultPeripherals<'a, ChipSpecs> {
    pub clocks: &'a crate::clocks::Clocks<'a, ChipSpecs>,
    pub gpio_ports: crate::gpio::GpioPorts<'a>,
}

impl<'a, ChipSpecs: ChipSpecsTrait> Stm32wle5xxDefaultPeripherals<'a, ChipSpecs> {
    pub fn new(clocks: &'a crate::clocks::Clocks<'a, ChipSpecs>) -> Self {
        Self {
            clocks,
            gpio_ports: crate::gpio::GpioPorts::new(clocks),
        }
    }

    // Setup any circular dependencies and register deferred calls
    pub fn setup_circular_deps(&'static self) {
        self.gpio_ports.setup_circular_deps();
    }
}

impl<'a, ChipSpecs: ChipSpecsTrait> InterruptService
    for Stm32wle5xxDefaultPeripherals<'a, ChipSpecs>
{
    unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
        match interrupt {
            _ => return false,
        }
        true
    }
}

impl<'a, I: InterruptService + 'a> Stm32wle5xx<'a, I> {
    pub unsafe fn new(interrupt_service: &'a I) -> Self {
        Self {
            mpu: cortexm4::mpu::MPU::new(),
            userspace_kernel_boundary: cortexm4::syscall::SysCall::new(),
            interrupt_service,
        }
    }
}

impl<'a, I: InterruptService + 'a> Chip for Stm32wle5xx<'a, I> {
    type MPU = cortexm4::mpu::MPU;
    type UserspaceKernelBoundary = cortexm4::syscall::SysCall;

    fn service_pending_interrupts(&self) {
        unsafe {
            loop {
                if let Some(interrupt) = cortexm4::nvic::next_pending() {
                    if !self.interrupt_service.service_interrupt(interrupt) {
                        panic!("unhandled interrupt {}", interrupt);
                    }

                    let n = cortexm4::nvic::Nvic::new(interrupt);
                    n.clear_pending();
                    n.enable();
                } else {
                    break;
                }
            }
        }
    }

    fn has_pending_interrupts(&self) -> bool {
        unsafe { cortexm4::nvic::has_pending() }
    }

    fn mpu(&self) -> &cortexm4::mpu::MPU {
        &self.mpu
    }

    fn userspace_kernel_boundary(&self) -> &cortexm4::syscall::SysCall {
        &self.userspace_kernel_boundary
    }

    fn sleep(&self) {
        unsafe {
            cortexm4::scb::unset_sleepdeep();
            cortexm4::support::wfi();
        }
    }

    unsafe fn atomic<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        cortexm4::support::atomic(f)
    }

    unsafe fn print_state(&self, write: &mut dyn Write) {
        CortexM4::print_cortexm_state(write);
    }
}
