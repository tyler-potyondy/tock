// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive SRL.
//
// Author: Ioan-Cristian CÎRSTEA <ioan.cirstea@oxidos.io>

//! Trait that encompasses chip specifications
//!
//! The main use of this trait is to be passed as a bound for the type parameter for chip
//! peripherals in crates such as `stm32f429zi`.

use crate::chip_specific::clock_constants::ClockConstants;

pub trait ChipSpecs: ClockConstants {}

impl<T: ClockConstants> ChipSpecs for T {}
