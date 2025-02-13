// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

use crate::chip_specs::Stm32wle5jcSpecs;
use stm32wle5xx::chip::Stm32wle5xxDefaultPeripherals;

pub struct Stm32wle5jcDefaultPeripherals<'a> {
    pub stm32wle: Stm32wle5xxDefaultPeripherals<'a, Stm32wle5jcSpecs>,
    // Once implemented, place Stm32f446re specific peripherals here
}

impl<'a> Stm32wle5jcDefaultPeripherals<'a> {
    pub unsafe fn new(clocks: &'a crate::clocks::Clocks<'a, Stm32wle5jcSpecs>) -> Self {
        Self {
            stm32wle: Stm32wle5xxDefaultPeripherals::new(clocks),
        }
    }
    // Necessary for setting up circular dependencies & registering deferred
    // calls
    pub fn init(&'static self) {
        self.stm32wle.setup_circular_deps();
    }
}
impl<'a> kernel::platform::chip::InterruptService for Stm32wle5jcDefaultPeripherals<'a> {
    unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
        match interrupt {
            // put Stm32f446re specific interrupts here
            _ => self.stm32wle.service_interrupt(interrupt),
        }
    }
}
