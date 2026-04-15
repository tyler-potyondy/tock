// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2025.

use crate::chip_specs::Stm32mp157fSpecs;
use stm32mp157x::chip::Stm32mp157xDefaultPeripherals;

pub struct Stm32mp157fDefaultPeripherals {
    pub stm32mp157x: Stm32mp157xDefaultPeripherals<Stm32mp157fSpecs>,
}

impl Stm32mp157fDefaultPeripherals {
    pub unsafe fn new() -> Self {
        Self {
            stm32mp157x: Stm32mp157xDefaultPeripherals::new(),
        }
    }
    // Necessary for setting up circular dependencies & registering deferred
    // calls
    pub fn init(&'static self) {
        self.stm32mp157x.setup_circular_deps();
    }
}
impl kernel::platform::chip::InterruptService for Stm32mp157fDefaultPeripherals {
    unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
        #[allow(clippy::match_single_binding)]
        match interrupt {
            // put Stm32mp157f specific interrupts here
            _ => self.stm32mp157x.service_interrupt(interrupt),
        }
    }
}
