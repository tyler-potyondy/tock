// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2023.

//! 7.5inch e-PaperV2

use core::cell::Cell;
use core::fmt::Error;
use core::panic;
use kernel::hil;
use kernel::hil::time::ConvertTicks;
use kernel::hil::time::{Alarm, AlarmClient};
use kernel::utilities::cells::{MapCell, OptionalCell, TakeCell};
use kernel::utilities::leasable_buffer::SubSliceMut;
use kernel::ErrorCode;

pub const BUFFER_SIZE: usize = 1032;

const IMAGE_DATA: [u8; 3096] = [
    0x00, 0x01, 0xF0, 0x00, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x78, 0x0F, 0x00, 0x0F, 0xFF, 0xFF,
    0xFF, 0xC0, 0x00, 0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x78, 0x0F, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xC0,
    0x00, 0x01, 0xF0, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x3F,
    0xFF, 0xFF, 0xFF, 0xFC, 0x03, 0xF0, 0x78, 0x0F, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x01,
    0xF0, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x3F, 0xFF, 0xFF,
    0xFF, 0xFC, 0x07, 0xEF, 0x7B, 0xCF, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x01, 0xF0, 0x00,
    0x00, 0x3F, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x3F, 0xFF, 0xFF, 0xFF, 0xFC,
    0x0F, 0xEF, 0x7B, 0xCF, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x03, 0xF0, 0x00, 0x00, 0x3F,
    0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x3F, 0xFF, 0xFF, 0xFF, 0xFC, 0x1F, 0xCF,
    0x7B, 0xDF, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xF0, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x3F, 0xFF, 0xFF,
    0xFC, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF8, 0x3F, 0xFF, 0xFF, 0xFF, 0xFC, 0x3F, 0x8F, 0x7B, 0xDF,
    0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xF0, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x1F, 0xFF, 0xFF, 0xF8, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFC, 0x3F, 0x0F, 0x7B, 0xDF, 0xFE, 0x3F,
    0xFF, 0xFF, 0xFF, 0xF0, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x1F, 0xF0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFC, 0xFF, 0xFC, 0x1E, 0xFF, 0xFF, 0xFF, 0xFE, 0x3F, 0xFF, 0xFF,
    0xFF, 0xF0, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x7F, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0F, 0xFF, 0xFF, 0xF8, 0xFF, 0xFC, 0x1D, 0xFF, 0xFF, 0xFF, 0xFE, 0x3C, 0x00, 0x7C, 0x01, 0xF0,
    0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF,
    0xFF, 0xF0, 0xFF, 0xFC, 0x09, 0xFF, 0xFF, 0xFF, 0xFE, 0x3D, 0xFE, 0x7F, 0xFF, 0xF0, 0x1F, 0x01,
    0xF0, 0x1F, 0x00, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xE0,
    0xFF, 0xFC, 0x03, 0xFF, 0xFF, 0xFE, 0xF8, 0x3D, 0xFE, 0x7F, 0xFF, 0xF0, 0x1F, 0x01, 0xF0, 0x1F,
    0x00, 0x00, 0x03, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xE0, 0xFF, 0xFC,
    0x03, 0xF0, 0x00, 0x7E, 0xF8, 0x3D, 0xFE, 0x7F, 0xFF, 0xF0, 0x1F, 0x01, 0xF0, 0x1F, 0x00, 0x00,
    0x03, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xE0, 0xFF, 0xFC, 0x07, 0xE0,
    0x00, 0xFE, 0xF8, 0x3C, 0x00, 0x7C, 0x01, 0xF0, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x0F, 0x0F, 0xC1, 0xF1, 0xFC, 0x0F, 0xEF, 0xFF, 0xFE,
    0xF8, 0x3D, 0xFE, 0x7F, 0xFF, 0xF0, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0E, 0x0F, 0x07, 0xC3, 0xE0, 0xFC, 0x1F, 0xEF, 0xFF, 0xFE, 0xF8, 0x3D,
    0xFE, 0x7F, 0xFE, 0xF0, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0E, 0x0F, 0x07, 0x83, 0xE0, 0xFC, 0x3F, 0xEF, 0xFF, 0xFE, 0xF8, 0x01, 0xFE, 0x7F,
    0xFE, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00,
    0x0F, 0x07, 0x07, 0x83, 0xC0, 0x7C, 0x3F, 0xEF, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x7C, 0x00, 0x00,
    0x1F, 0x03, 0xF0, 0x1F, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x07,
    0x03, 0x87, 0xC0, 0x7C, 0x3F, 0xE0, 0x00, 0x3F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x01,
    0xF0, 0x1F, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x06, 0x03, 0x07,
    0x80, 0x3C, 0x3F, 0xE3, 0xFF, 0x0F, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x01, 0xF0, 0x1F,
    0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x01, 0x0F, 0x80, 0x3C,
    0x1F, 0xE3, 0xFF, 0x0F, 0xF0, 0x07, 0xFF, 0xFF, 0xFF, 0x80, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
    0x07, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x0F, 0x80, 0x3C, 0x19, 0xE3,
    0xFF, 0x0F, 0xF0, 0x07, 0xFF, 0xFF, 0xFF, 0x80, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x60, 0x0F, 0x04, 0x1C, 0x01, 0xE3, 0xFF, 0x0F,
    0xE0, 0x07, 0xFF, 0xFF, 0xFF, 0x80, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x60, 0x1E, 0x04, 0x1C, 0x01, 0xE3, 0xCF, 0x0F, 0xE0, 0x03,
    0xFF, 0xFF, 0xFF, 0x80, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0F, 0xC0, 0x70, 0x1E, 0x0E, 0x0C, 0x01, 0xE3, 0xCF, 0xF7, 0xE0, 0x00, 0x00, 0x00,
    0x0F, 0x80, 0x1F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0F, 0xC0, 0xF0, 0x3C, 0x1F, 0x04, 0x01, 0xE3, 0xCF, 0xF7, 0xC0, 0x03, 0xFF, 0xFF, 0xFF, 0x80,
    0x1F, 0x01, 0xF0, 0x01, 0xC0, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xE0,
    0xF0, 0x78, 0x1F, 0x0C, 0x01, 0xE7, 0xCF, 0xFF, 0xE0, 0x03, 0xFF, 0xFF, 0xFF, 0x80, 0x1F, 0x01,
    0xF0, 0x03, 0xF0, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xE0, 0xF0, 0x78,
    0x3E, 0x0C, 0x01, 0xE7, 0xCF, 0xFF, 0xF0, 0x03, 0xFF, 0xFF, 0xFF, 0x80, 0x1F, 0x01, 0xF0, 0x03,
    0xF0, 0x00, 0x03, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xE0, 0xF8, 0x78, 0x3E, 0x1C,
    0x01, 0xE7, 0xCF, 0xFF, 0xF0, 0x01, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x01, 0xF0, 0x03, 0xF0, 0x00,
    0x03, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xF1, 0xF8, 0x78, 0x3C, 0x1C, 0x01, 0xEF,
    0x8F, 0xBF, 0xF8, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x01, 0xF0, 0x03, 0xE0, 0x00, 0x03, 0xE0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFC, 0x3C, 0x01, 0xEF, 0x8F, 0xFF,
    0xFC, 0x07, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x01, 0xFF, 0xFF, 0xE0, 0x03, 0xFF, 0xE0, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xF0, 0x3C, 0x01, 0xFF, 0x8F, 0xFE, 0xFF, 0x07,
    0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x01, 0xFF, 0xFF, 0xE0, 0x01, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xF0, 0x7C, 0x01, 0xFF, 0x01, 0xFC, 0x7E, 0x07, 0xFF, 0xFF,
    0xFF, 0x80, 0x00, 0x01, 0xFF, 0xFF, 0xC0, 0x01, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0F, 0xFF, 0xFF, 0xFF, 0xF0, 0x7C, 0x01, 0xE7, 0x00, 0xF8, 0x3C, 0x07, 0xFF, 0xFF, 0xFF, 0x80,
    0x00, 0x00, 0xFF, 0xFF, 0xC0, 0x01, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF,
    0xFF, 0xFF, 0xF0, 0x7C, 0x01, 0xE6, 0x00, 0xF0, 0x18, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00,
    0x7F, 0xFF, 0x80, 0x01, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF,
    0xF0, 0xFC, 0x01, 0xE0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xF1, 0xFC,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xF3, 0x83, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xF7, 0x83, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE,
    0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0x83, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00,
    0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0x83, 0xCE, 0x71, 0xFF, 0xFF, 0xFF, 0xF8, 0xFF, 0xFF,
    0xFF, 0xFF, 0x03, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x3F, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00,
    0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0x83, 0xC6, 0x33, 0xFF, 0xFF, 0xFF, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF,
    0x3F, 0x3F, 0xFF, 0xF3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xC6, 0x23, 0x8E, 0x79, 0x8F, 0x80, 0x1F, 0x1E, 0x1E, 0x3F, 0x3F, 0x3C,
    0x7C, 0x20, 0x83, 0x8E, 0x4F, 0x3C, 0x30, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0xC4, 0x23, 0x07, 0x33, 0x07, 0x08, 0x0E, 0x0C, 0x18, 0x1F, 0x3F, 0x30, 0x38, 0x20,
    0x82, 0x06, 0x06, 0x30, 0x30, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
    0xE4, 0x03, 0x33, 0x32, 0x32, 0x78, 0xC6, 0x44, 0x39, 0x9F, 0x07, 0x31, 0xB1, 0xF3, 0x8E, 0x66,
    0x22, 0x31, 0xE7, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xE0, 0x07,
    0xC3, 0x02, 0x03, 0x18, 0xC7, 0x84, 0x70, 0x1F, 0x3F, 0x30, 0x13, 0xF3, 0x9C, 0x62, 0x62, 0x33,
    0xE1, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xE1, 0x87, 0x03, 0x06,
    0x03, 0x88, 0xC6, 0x04, 0x70, 0x1F, 0x3F, 0x30, 0x13, 0xF3, 0x9C, 0x62, 0x62, 0x33, 0xF0, 0xFE,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xE1, 0x86, 0x33, 0x06, 0x7F, 0xC0,
    0xC4, 0x44, 0x71, 0xFF, 0x3F, 0x33, 0xF3, 0xF3, 0x9C, 0x62, 0x62, 0x33, 0xFC, 0xFE, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xE1, 0x8E, 0x23, 0x8F, 0x3F, 0xC8, 0xC4, 0x44,
    0x78, 0xFF, 0x3F, 0x31, 0xF9, 0xF1, 0x9E, 0x06, 0x62, 0x31, 0xFC, 0xFE, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xF1, 0x8F, 0x03, 0x8F, 0x86, 0x18, 0xC6, 0x04, 0x7C, 0x1F,
    0x03, 0x3C, 0x3C, 0x30, 0x9F, 0x0E, 0x62, 0x3C, 0x61, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];

// TODO - check if these are correct.
const WIDTH: usize = 800;
const HEIGHT: usize = 480;

// all commands that are on command table (pg 22.)
#[derive(Copy, Clone, PartialEq)]
#[repr(u8)]
enum VDHLevel {
    V2_4 = 0b000000,
    V2_6 = 0b000001,
    V2_8 = 0b000010,
    V3_0 = 0b000011,
    V3_2 = 0b000100,
    V3_4 = 0b000101,
    V3_6 = 0b000110,
    V3_8 = 0b000111,
    V4_0 = 0b001000,
    V4_2 = 0b001001,
    V4_4 = 0b001010,
    V4_6 = 0b001011,
    V4_8 = 0b001100,
    V5_0 = 0b001101,
    V5_2 = 0b001110,
    V5_4 = 0b001111,
    V5_6 = 0b010000,
    V5_8 = 0b010001,
    V6_0 = 0b010010,
    V6_2 = 0b010011,
    V6_4 = 0b010100,
    V6_6 = 0b010101,
    V6_8 = 0b010110,
    V7_0 = 0b010111,
    V7_2 = 0b011000,
    V7_4 = 0b011001,
    V7_6 = 0b011010,
    V7_8 = 0b011011,
    V8_0 = 0b011100,
    V8_2 = 0b011101,
    V8_4 = 0b011110,
    V8_6 = 0b011111,
    V8_8 = 0b100000,
    V9_0 = 0b100001,
    V9_2 = 0b100010,
    V9_4 = 0b100011,
    V9_6 = 0b100100,
    V9_8 = 0b100101,
    V10_0 = 0b100110,
    V10_2 = 0b100111,
    V10_4 = 0b101000,
    V10_6 = 0b101001,
    V10_8 = 0b101010,
    V11_0 = 0b101011,
    V11_2 = 0b101100,
    V11_4 = 0b101101,
    V11_6 = 0b101110,
    V11_8 = 0b101111,
    V12_0 = 0b110000,
    V12_2 = 0b110001,
    V12_4 = 0b110010,
    V12_6 = 0b110011,
    V12_8 = 0b110100,
    V13_0 = 0b110101,
    V13_2 = 0b110110,
    V13_4 = 0b110111,
    V13_6 = 0b111000,
    V13_8 = 0b111001,
    V14_0 = 0b111010,
    V14_2 = 0b111011,
    V14_4 = 0b111100,
    V14_6 = 0b111101,
    V14_8 = 0b111110,
    V15_0 = 0b111111,
}

#[repr(u8)]
enum VDLLevel {
    Neg2_4 = 0b000000,
    Neg2_6 = 0b000001,
    Neg2_8 = 0b000010,
    Neg3_0 = 0b000011,
    Neg3_2 = 0b000100,
    Neg3_4 = 0b000101,
    Neg3_6 = 0b000110,
    Neg3_8 = 0b000111,
    Neg4_0 = 0b001000,
    Neg4_2 = 0b001001,
    Neg4_4 = 0b001010,
    Neg4_6 = 0b001011,
    Neg4_8 = 0b001100,
    Neg5_0 = 0b001101,
    Neg5_2 = 0b001110,
    Neg5_4 = 0b001111,
    Neg5_6 = 0b010000,
    Neg5_8 = 0b010001,
    Neg6_0 = 0b010010,
    Neg6_2 = 0b010011,
    Neg6_4 = 0b010100,
    Neg6_6 = 0b010101,
    Neg6_8 = 0b010110,
    Neg7_0 = 0b010111,
    Neg7_2 = 0b011000,
    Neg7_4 = 0b011001,
    Neg7_6 = 0b011010,
    Neg7_8 = 0b011011,
    Neg8_0 = 0b011100,
    Neg8_2 = 0b011101,
    Neg8_4 = 0b011110,
    Neg8_6 = 0b011111,
    Neg8_8 = 0b100000,
    Neg9_0 = 0b100001,
    Neg9_2 = 0b100010,
    Neg9_4 = 0b100011,
    Neg9_6 = 0b100100,
    Neg9_8 = 0b100101,
    Neg10_0 = 0b100110,
    Neg10_2 = 0b100111,
    Neg10_4 = 0b101000,
    Neg10_6 = 0b101001,
    Neg10_8 = 0b101010,
    Neg11_0 = 0b101011,
    Neg11_2 = 0b101100,
    Neg11_4 = 0b101101,
    Neg11_6 = 0b101110,
    Neg11_8 = 0b101111,
    Neg12_0 = 0b110000,
    Neg12_2 = 0b110001,
    Neg12_4 = 0b110010,
    Neg12_6 = 0b110011,
    Neg12_8 = 0b110100,
    Neg13_0 = 0b110101,
    Neg13_2 = 0b110110,
    Neg13_4 = 0b110111,
    Neg13_6 = 0b111000,
    Neg13_8 = 0b111001,
    Neg14_0 = 0b111010,
    Neg14_2 = 0b111011,
    Neg14_4 = 0b111100,
    Neg14_6 = 0b111101,
    Neg14_8 = 0b111110,
    Neg15_0 = 0b111111,
}

#[repr(u8)]
enum VDHRLevel {
    V2_4 = 0b000000,
    V2_6 = 0b000001,
    V2_8 = 0b000010,
    V3_0 = 0b000011,
    V3_2 = 0b000100,
    V3_4 = 0b000101,
    V3_6 = 0b000110,
    V3_8 = 0b000111,
    V4_0 = 0b001000,
    V4_2 = 0b001001,
    V4_4 = 0b001010,
    V4_6 = 0b001011,
    V4_8 = 0b001100,
    V5_0 = 0b001101,
    V5_2 = 0b001110,
    V5_4 = 0b001111,
    V5_6 = 0b010000,
    V5_8 = 0b010001,
    V6_0 = 0b010010,
    V6_2 = 0b010011,
    V6_4 = 0b010100,
    V6_6 = 0b010101,
    V6_8 = 0b010110,
    V7_0 = 0b010111,
    V7_2 = 0b011000,
    V7_4 = 0b011001,
    V7_6 = 0b011010,
    V7_8 = 0b011011,
    V8_0 = 0b011100,
    V8_2 = 0b011101,
    V8_4 = 0b011110,
    V8_6 = 0b011111,
    V8_8 = 0b100000,
    V9_0 = 0b100001,
    V9_2 = 0b100010,
    V9_4 = 0b100011,
    V9_6 = 0b100100,
    V9_8 = 0b100101,
    V10_0 = 0b100110,
    V10_2 = 0b100111,
    V10_4 = 0b101000,
    V10_6 = 0b101001,
    V10_8 = 0b101010,
    V11_0 = 0b101011,
    V11_2 = 0b101100,
    V11_4 = 0b101101,
    V11_6 = 0b101110,
    V11_8 = 0b101111,
    V12_0 = 0b110000,
    V12_2 = 0b110001,
    V12_4 = 0b110010,
    V12_6 = 0b110011,
    V12_8 = 0b110100,
    V13_0 = 0b110101,
    V13_2 = 0b110110,
    V13_4 = 0b110111,
    V13_6 = 0b111000,
    V13_8 = 0b111001,
    V14_0 = 0b111010,
    V14_2 = 0b111011,
    V14_4 = 0b111100,
    V14_6 = 0b111101,
    V14_8 = 0b111110,
    V15_0 = 0b111111,
}

#[repr(u8)]
enum VGLevel {
    VGH9_VGLNeg9 = 0b000,   // VGH=9V, VGL=-9V
    VGH10_VGLNeg10 = 0b001, // VGH=10V, VGL=-10V
    VGH11_VGLNeg11 = 0b010, // VGH=11V, VGL=-11V
    VGH12_VGLNeg12 = 0b011, // VGH=12V, VGL=-12V
    VGH17_VGLNeg17 = 0b100, // VGH=17V, VGL=-17V
    VGH18_VGLNeg18 = 0b101, // VGH=18V, VGL=-18V
    VGH19_VGLNeg19 = 0b110, // VGH=19V, VGL=-19V
    VGH20_VGLNeg20 = 0b111, // VGH=20V, VGL=-20V (Default)
}

pub enum Command {
    PanelSetting,
    PowerSetting,
    PowerOff,
    PowerOffSequenceSetting,
    PowerOn,
    PowerOnMeasure,
    BoosterSoftStart,
    DeepSleep,
    DataStartTransmission1,
    DataStop,
    DisplayRefresh,
    DisplayStartTransmission2,
    DualSPI,
    AutoSequence,
    KWOPT,
    PLLControl,
    TemperatureSensorCalibration,
    TemperatureSensorSelection,
    TemperatureSensorWrite,
    TemperatureSensorRead,
    PanelBreakCheck,
    VCOMAndDataIntervalSetting,
    LowPowerDetection,
    TCONSetting,
    ResolutionSetting,
    GateSourceStartSetting,
    Revision,
    GetStatus,
    AutoMeasureVCOM,
    ReadVCOMValue,
    VCOMDCSetting,
    PartialWindow,
    PartialIn,
    PartialOut,
    ProgramMode,
    ActiveProgramming,
    ReadOTP,
    CascadeSetting,
    PowerSaving,
    LVDVoltageSelect,
    ForceTemperature,
    TemperatureBoundaryPhaseC2,
}

pub enum Data {
    VDHLevel,
    VDLLevel,
    VDHRLevel,
    VGLevel,
}

// in match statement add binary that corresponds to each
// command type
impl Command {
    fn encode(self, buffer: &mut SubSliceMut<'static, u8>) {
        let take = match self {
            _ => (),
        };

        // Move the available region of the buffer to what is remaining after
        // this command was encoded.
        // buffer.slice(take..);
    }
}

#[derive(Clone, Copy, PartialEq)]
enum TransferMode {
    Command,
    Data(usize),
}
// #[derive(Copy, Clone, PartialEq)]
#[derive(Clone, Copy, PartialEq)]
enum State {
    Idle,
    Init(usize, TransferMode),
    SimpleCommand,
    Write,
    BusyInit(usize),
}

const send_init_array: [&[u8]; 18] = [
    &[0x6, 0x17, 0x17, 0x27, 0x17],
    &[0x1, 0x07, 0x17, 0x3f, 0x3f],
    &[0x04],
    &[0],
    // check busy pin
    &[0x00, 0x1f],
    &[0x61, 0x03, 0x20, 0x01, 0xe0],
    &[0x15, 0x00],
    &[0x60, 0x22],
    &[0x50, 0x10, 0x07],
    &[0x10],
    &[
        0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
        0x1, 0x1, 0x1,
    ],
    &[0x13],
    &[
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0,
    ],
    &[0x12],
    // check busy pin
    &[0x00],
    &[0x2],
    // check busy pin
    &[0x00],
    &[0x07, 0xa5],
];

#[derive(Copy, Clone, PartialEq)]
enum InitState {
    ResetFirstClear,
    ResetFirstSet,
    ResetSecondClear,
}

pub struct EPaper<
    'a,
    S: hil::spi::SpiMasterDevice<'a>,
    G: hil::gpio::Output,
    GI: hil::gpio::Input,
    A: hil::time::Alarm<'a>,
> {
    spi: &'a S,
    gpio: &'a G,
    state: Cell<State>,
    client: OptionalCell<&'a dyn hil::screen::ScreenClient>,
    setup_client: OptionalCell<&'a dyn hil::screen::ScreenSetupClient>,
    buffer: TakeCell<'static, [u8]>,
    write_buffer: MapCell<SubSliceMut<'static, u8>>,
    pending: OptionalCell<bool>,
    busy_gpio: &'a GI,
    gpio_reset: &'a G,
    alarm: &'a A,
    init_state: Cell<InitState>,
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice<'a>,
        G: hil::gpio::Output,
        GI: hil::gpio::Input,
        A: hil::time::Alarm<'a>,
    > EPaper<'a, S, G, GI, A>
{
    pub fn new(
        spi: &'a S,
        gpio: &'a G,
        buffer: &'static mut [u8],
        enable_charge_pump: bool,
        busy_gpio: &'a GI,
        gpio_reset: &'a G,
        alarm: &'a A,
    ) -> EPaper<'a, S, G, GI, A> {
        EPaper {
            spi,
            gpio,
            state: Cell::new(State::Idle),
            client: OptionalCell::empty(),
            setup_client: OptionalCell::empty(),
            buffer: TakeCell::new(buffer),
            write_buffer: MapCell::empty(),
            pending: OptionalCell::empty(),
            busy_gpio,
            gpio_reset,
            alarm,
            init_state: Cell::new(InitState::ResetFirstClear),
        }
    }

    fn sendcommand(&self, buf: &'static mut [u8]) {
        self.gpio.clear();
        self.spi.read_write_bytes(buf, None, 1);
    }

    fn senddata(&self, buf: &'static mut [u8]) {
        self.gpio.set();
        self.spi.read_write_bytes(buf, None, 1);
    }

    pub fn test_init(&self) {
        self.gpio_reset.clear();
        self.gpio_reset.set();
    }

    pub fn init_screen(&self) {
        let state = self.state.get();
        match state {
            State::Idle => {
                let buf = self.buffer.take().unwrap();
                buf[0] = send_init_array[0][0];
                self.state.set(State::Init(0, TransferMode::Command));
                self.sendcommand(buf)
            }
            State::Init(ind, trans) => {
                if ind == 3 || ind == 14 || ind == 16 {
                    if !self.busy_gpio.read() {
                        let dt = self.alarm.ticks_from_ms(100);
                        self.alarm.set_alarm(self.alarm.now(), dt);
                        return;
                    }
                }
                let buf = self.buffer.take().unwrap();
                match trans {
                    TransferMode::Command => {
                        buf[0] = send_init_array[ind][0];
                        self.state.set(State::Init(ind, TransferMode::Command));
                        self.sendcommand(buf);
                    }
                    TransferMode::Data(ind_data) => {
                        buf[0] = send_init_array[ind][ind_data];
                        self.state
                            .set(State::Init(ind, TransferMode::Data(ind_data)));
                        self.senddata(buf);
                    }
                }
            }
            _ => (),
        }
    }

    fn send_image(&self) {}

    fn send_sequence(&self, sequence: &[Command]) -> Result<(), ErrorCode> {
        Ok(())
    }
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice<'a>,
        G: hil::gpio::Output,
        GI: hil::gpio::Input,
        A: hil::time::Alarm<'a>,
    > hil::screen::ScreenSetup<'a> for EPaper<'a, S, G, GI, A>
{
    fn set_client(&self, client: &'a dyn hil::screen::ScreenSetupClient) {
        self.setup_client.set(client);
    }

    fn set_resolution(&self, _resolution: (usize, usize)) -> Result<(), ErrorCode> {
        // todo
        Err(ErrorCode::NOSUPPORT)
    }

    fn set_pixel_format(&self, _depth: hil::screen::ScreenPixelFormat) -> Result<(), ErrorCode> {
        // todo
        Err(ErrorCode::NOSUPPORT)
    }

    fn set_rotation(&self, _rotation: hil::screen::ScreenRotation) -> Result<(), ErrorCode> {
        // todo
        Err(ErrorCode::NOSUPPORT)
    }

    fn get_num_supported_resolutions(&self) -> usize {
        // todo
        1
    }

    fn get_supported_resolution(&self, index: usize) -> Option<(usize, usize)> {
        // todo
        match index {
            0 => Some((WIDTH, HEIGHT)),
            _ => None,
        }
    }

    fn get_num_supported_pixel_formats(&self) -> usize {
        // todo
        1
    }

    fn get_supported_pixel_format(&self, index: usize) -> Option<hil::screen::ScreenPixelFormat> {
        // todo
        match index {
            0 => Some(hil::screen::ScreenPixelFormat::Mono),
            _ => None,
        }
    }
}

// TODO
impl<
        'a,
        S: hil::spi::SpiMasterDevice<'a>,
        G: hil::gpio::Output,
        GI: hil::gpio::Input,
        A: hil::time::Alarm<'a>,
    > hil::screen::Screen<'a> for EPaper<'a, S, G, GI, A>
{
    fn set_client(&self, client: &'a dyn hil::screen::ScreenClient) {
        self.client.set(client);
    }

    fn get_resolution(&self) -> (usize, usize) {
        (WIDTH, HEIGHT)
    }

    fn get_pixel_format(&self) -> hil::screen::ScreenPixelFormat {
        hil::screen::ScreenPixelFormat::Mono
    }

    fn get_rotation(&self) -> hil::screen::ScreenRotation {
        hil::screen::ScreenRotation::Normal
    }

    fn set_write_frame(
        &self,
        x: usize,
        y: usize,
        width: usize,
        height: usize,
    ) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn write(&self, data: SubSliceMut<'static, u8>, _continue: bool) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn set_brightness(&self, brightness: u16) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn set_power(&self, enabled: bool) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn set_invert(&self, enabled: bool) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice<'a>,
        G: hil::gpio::Output,
        GI: hil::gpio::Input,
        A: hil::time::Alarm<'a>,
    > hil::i2c::I2CClient for EPaper<'a, S, G, GI, A>
{
    fn command_complete(&self, buffer: &'static mut [u8], _status: Result<(), hil::i2c::Error>) {}
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice<'a>,
        G: hil::gpio::Output,
        GI: hil::gpio::Input,
        A: hil::time::Alarm<'a>,
    > hil::spi::SpiMasterClient for EPaper<'a, S, G, GI, A>
{
    fn read_write_done(
        &self,
        mut _write: &'static mut [u8],
        mut read: Option<&'static mut [u8]>,
        _len: usize,
        spi_status: Result<(), ErrorCode>,
    ) {
        self.buffer.replace(_write);
        match self.state.get() {
            State::Init(ind, trans) => {
                if ind >= send_init_array.len() - 1 {
                    self.state.set(State::Idle);
                    return;
                }

                match trans {
                    TransferMode::Command => {
                        if send_init_array[ind].len() == 1 {
                            self.state.set(State::Init(ind + 1, TransferMode::Command));
                        } else {
                            self.state.set(State::Init(ind, TransferMode::Data(1)));
                        }
                    }
                    TransferMode::Data(ind_data) => {
                        if ind_data >= send_init_array[ind].len() - 1 {
                            self.state.set(State::Init(ind + 1, TransferMode::Command));
                        } else {
                            self.state
                                .set(State::Init(ind, TransferMode::Data(ind_data + 1)));
                        }
                    }
                }

                self.init_screen();
            }
            _ => panic!(),
        }
    }
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice<'a>,
        G: hil::gpio::Output,
        GI: hil::gpio::Input,
        A: hil::time::Alarm<'a>,
    > kernel::hil::gpio::Client for EPaper<'a, S, G, GI, A>
{
    fn fired(&self) {
        kernel::debug!("fired");
    }
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice<'a>,
        G: hil::gpio::Output,
        GI: hil::gpio::Input,
        A: hil::time::Alarm<'a>,
    > kernel::hil::time::AlarmClient for EPaper<'a, S, G, GI, A>
{
    fn alarm(&self) {
        kernel::debug!("alarm");
        if self.busy_gpio.read() {
            match self.state.get() {
                State::Init(3, mode) => self.state.set(State::Init(4, TransferMode::Command)),
                State::Init(14, mode) => self.state.set(State::Init(15, TransferMode::Command)),
                State::Init(16, mode) => self.state.set(State::Init(17, TransferMode::Command)),
                _ => panic!("here is a panic"),
            }
            self.init_screen();
        } else {
            let dt = self.alarm.ticks_from_ms(100);
            self.alarm.set_alarm(self.alarm.now(), dt);
        }
    }
}
