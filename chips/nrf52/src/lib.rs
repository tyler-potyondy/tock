// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

#![no_std]
#![crate_name = "nrf52"]
#![crate_type = "rlib"]

pub mod uart;

// pub use crate::crt1::init;
pub use nrf5x::{constants, peripheral_interrupts};
