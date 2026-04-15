// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2025.

// (TODO): THESE VALUES ARE NOT FILLED IN...JUST COPIED FROM F4 for now.
use stm32mp157x::chip_specific::clock_constants::{PllConstants, SystemClockConstants};

pub enum Stm32mp157fSpecs {}

impl PllConstants for Stm32mp157fSpecs {
    const MIN_FREQ_MHZ: usize = 13; // TODO: this is random placeholder value.
}

impl SystemClockConstants for Stm32mp157fSpecs {
    const APB1_FREQUENCY_LIMIT_MHZ: usize = 45;
    const SYS_CLOCK_FREQUENCY_LIMIT_MHZ: usize = 168;
}
