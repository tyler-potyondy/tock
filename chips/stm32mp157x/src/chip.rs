// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! Chip trait setup.

use core::fmt::Write;
use cortexm4f::{CortexM4F, CortexMVariant};
use kernel::platform::chip::Chip;
use kernel::platform::chip::InterruptService;

use crate::chip_specific::chip_specs::ChipSpecs as ChipSpecsTrait;
use crate::nvic;

pub struct Stm32mp157x<'a, I: InterruptService + 'a> {
    mpu: cortexm4f::mpu::MPU,
    userspace_kernel_boundary: cortexm4f::syscall::SysCall,
    interrupt_service: &'a I,
}

pub struct Stm32mp157xDefaultPeripherals<'a, ChipSpecs> {
    pub gpio_ports: crate::gpio::GpioPorts<'a>,
    pub clocks: &'a crate::clocks::Clocks<'a, ChipSpecs>,
    pub usart4: crate::usart::Usart<'a>,
}

impl<'a, ChipSpecs: ChipSpecsTrait> Stm32mp157xDefaultPeripherals<'a, ChipSpecs> {
    pub fn new(clocks: &'a crate::clocks::Clocks<'a, ChipSpecs>) -> Self {
        Self {
            gpio_ports: crate::gpio::GpioPorts::new(clocks),
            clocks,
            usart4: crate::usart::Usart::new_uart4(clocks),
        }
    }

    // Setup any circular dependencies and register deferred calls
    pub fn setup_circular_deps(&'static self) {
        self.gpio_ports.setup_circular_deps();
        kernel::deferred_call::DeferredCallClient::register(&self.usart4);
    }
}

impl<'a, ChipSpecs: ChipSpecsTrait> InterruptService
    for Stm32mp157xDefaultPeripherals<'a, ChipSpecs>
{
    unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
        match interrupt {
            //            nvic::USART1 => self.usart1.handle_interrupt(),
            //            nvic::USART2 => self.usart2.handle_interrupt(),
            //
            //            nvic::TIM2 => self.tim2.handle_interrupt(),
            //
            _ => return false,
        }
        true
    }
}

impl<'a, I: InterruptService + 'a> Stm32mp157x<'a, I> {
    pub unsafe fn new(interrupt_service: &'a I) -> Self {
        Self {
            mpu: cortexm4f::mpu::new(),
            userspace_kernel_boundary: cortexm4f::syscall::SysCall::new(),
            interrupt_service,
        }
    }
}

impl<'a, I: InterruptService + 'a> Chip for Stm32mp157x<'a, I> {
    type MPU = cortexm4f::mpu::MPU;
    type UserspaceKernelBoundary = cortexm4f::syscall::SysCall;
    type ThreadIdProvider = cortexm4f::thread_id::CortexMThreadIdProvider;

    fn service_pending_interrupts(&self) {
        unsafe {
            while let Some(interrupt) = cortexm4f::nvic::next_pending() {
                if !self.interrupt_service.service_interrupt(interrupt) {
                    panic!("unhandled interrupt {}", interrupt);
                }

                let n = cortexm4f::nvic::Nvic::new(interrupt);
                n.clear_pending();
                n.enable();
            }
        }
    }

    fn has_pending_interrupts(&self) -> bool {
        unsafe { cortexm4f::nvic::has_pending() }
    }

    fn mpu(&self) -> &cortexm4f::mpu::MPU {
        &self.mpu
    }

    fn userspace_kernel_boundary(&self) -> &cortexm4f::syscall::SysCall {
        &self.userspace_kernel_boundary
    }

    fn sleep(&self) {
        unsafe {
            cortexm4f::scb::unset_sleepdeep();
            cortexm4f::support::wfi();
        }
    }

    unsafe fn with_interrupts_disabled<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        cortexm4f::support::with_interrupts_disabled(f)
    }

    unsafe fn print_state(_this: Option<&Self>, write: &mut dyn Write) {
        CortexM4F::print_cortexm_state(write);
    }
}
