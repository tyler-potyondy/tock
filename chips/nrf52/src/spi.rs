// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Implementation of SPI for NRF52 using EasyDMA.
//!
//! This file only implements support for the three SPI master (`SPIM`)
//! peripherals, and not SPI slave (`SPIS`).
//!
//! Although `kernel::hil::spi::SpiMaster` is implemented for `SPIM`,
//! only the functions marked with `x` are fully defined:
//!
//! * ✓ set_client
//! * ✓ init
//! * ✓ is_busy
//! * ✓ read_write_bytes
//! * write_byte
//! * read_byte
//! * read_write_byte
//! * ✓ specify_chip_select
//! * ✓ set_rate
//! * ✓ get_rate
//! * ✓ set_polarity
//! * ✓ get_polarity
//! * ✓ set_phase
//! * ✓ get_phase
//! * hold_low
//! * release_low
//!
//! Author
//! -------------------
//!
//! * Author: Jay Kickliter
//! * Date: Sep 10, 2017

use core::cell::Cell;
use core::{cmp, ptr};
use kernel::hil;
use kernel::hil::gpio::Configure;
use kernel::hil::spi::util::ChipSelectPolar;
use kernel::utilities::cells::{OptionalCell, TakeCell, VolatileCell};
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, ReadWrite, WriteOnly};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;
use nrf5x::pinmux::Pinmux;

const INSTANCES: [StaticRef<SpimRegisters>; 3] = unsafe {
    [
        StaticRef::new(0x40003000 as *const SpimRegisters),
        StaticRef::new(0x40004000 as *const SpimRegisters),
        StaticRef::new(0x40023000 as *const SpimRegisters),
    ]
};

#[repr(C)]
struct SpimRegisters {
    _reserved0: [u8; 16],                            // reserved
    tasks_start: WriteOnly<u32, TASK::Register>,     // Start SPI transaction
    tasks_stop: WriteOnly<u32, TASK::Register>,      // Stop SPI transaction
    _reserved1: [u8; 4],                             // reserved
    tasks_suspend: WriteOnly<u32, TASK::Register>,   // Suspend SPI transaction
    tasks_resume: WriteOnly<u32, TASK::Register>,    // Resume SPI transaction
    _reserved2: [u8; 224],                           // reserved
    events_stopped: ReadWrite<u32, EVENT::Register>, // SPI transaction has stopped
    _reserved3: [u8; 8],                             // reserved
    events_endrx: ReadWrite<u32, EVENT::Register>,   // End of RXD buffer reached
    _reserved4: [u8; 4],                             // reserved
    events_end: ReadWrite<u32, EVENT::Register>,     // End of RXD buffer and TXD buffer reached
    _reserved5: [u8; 4],                             // reserved
    events_endtx: ReadWrite<u32, EVENT::Register>,   // End of TXD buffer reached
    _reserved6: [u8; 40],                            // reserved
    events_started: ReadWrite<u32, EVENT::Register>, // Transaction started
    _reserved7: [u8; 176],                           // reserved
    shorts: ReadWrite<u32>,                          // Shortcut register
    _reserved8: [u8; 256],                           // reserved
    intenset: ReadWrite<u32, INTE::Register>,        // Enable interrupt
    intenclr: ReadWrite<u32, INTE::Register>,        // Disable interrupt
    _reserved9: [u8; 500],                           // reserved
    enable: ReadWrite<u32, ENABLE::Register>,        // Enable SPIM
    _reserved10: [u8; 4],                            // reserved
    psel_sck: VolatileCell<Pinmux>,                  // Pin select for SCK
    psel_mosi: VolatileCell<Pinmux>,                 // Pin select for MOSI signal
    psel_miso: VolatileCell<Pinmux>,                 // Pin select for MISO signal
    _reserved11: [u8; 16],                           // reserved
    frequency: ReadWrite<u32>,                       // SPI frequency
    _reserved12: [u8; 12],                           // reserved
    rxd_ptr: VolatileCell<*mut u8>,                  // Data pointer
    rxd_maxcnt: ReadWrite<u32, MAXCNT::Register>,    // Maximum number of bytes in receive buffer
    rxd_amount: ReadWrite<u32>,                      // Number of bytes transferred
    rxd_list: ReadWrite<u32>,                        // EasyDMA list type
    txd_ptr: VolatileCell<*const u8>,                // Data pointer
    txd_maxcnt: ReadWrite<u32, MAXCNT::Register>,    // Maximum number of bytes in transmit buffer
    txd_amount: ReadWrite<u32>,                      // Number of bytes transferred
    txd_list: ReadWrite<u32>,                        // EasyDMA list type
    config: ReadWrite<u32, CONFIG::Register>,        // Configuration register
    _reserved13: [u8; 104],                          // reserved
    orc: ReadWrite<u32>,                             // Over-read character.
}

register_bitfields![u32,
    INTE [
        /// Write '1' to Enable interrupt on EVENTS_STOPPED event
        STOPPED OFFSET(1) NUMBITS(1) [
            /// Read: Disabled
            ReadDisabled = 0,
            /// Enable
            Enable = 1
        ],
        /// Write '1' to Enable interrupt on EVENTS_ENDRX event
        ENDRX OFFSET(4) NUMBITS(1) [
            /// Read: Disabled
            ReadDisabled = 0,
            /// Enable
            Enable = 1
        ],
        /// Write '1' to Enable interrupt on EVENTS_END event
        END OFFSET(6) NUMBITS(1) [
            /// Read: Disabled
            ReadDisabled = 0,
            /// Enable
            Enable = 1
        ],
        /// Write '1' to Enable interrupt on EVENTS_ENDTX event
        ENDTX OFFSET(8) NUMBITS(1) [
            /// Read: Disabled
            ReadDisabled = 0,
            /// Enable
            Enable = 1
        ],
        /// Write '1' to Enable interrupt on EVENTS_STARTED event
        STARTED OFFSET(19) NUMBITS(1) [
            /// Read: Disabled
            ReadDisabled = 0,
            /// Enable
            Enable = 1
        ]
    ],
    MAXCNT [
        /// Maximum number of bytes in buffer
        MAXCNT OFFSET(0) NUMBITS(16)
    ],
    CONFIG [
        /// Bit order
        ORDER OFFSET(0) NUMBITS(1) [
            /// Most significant bit shifted out first
            MostSignificantBitShiftedOutFirst = 0,
            /// Least significant bit shifted out first
            LeastSignificantBitShiftedOutFirst = 1
        ],
        /// Serial clock (SCK) phase
        CPHA OFFSET(1) NUMBITS(1) [
            /// Sample on leading edge of clock, shift serial data on trailing edge
            SampleOnLeadingEdge = 0,
            /// Sample on trailing edge of clock, shift serial data on leading edge
            SampleOnTrailingEdge = 1
        ],
        /// Serial clock (SCK) polarity
        CPOL OFFSET(2) NUMBITS(1) [
            /// Active high
            ActiveHigh = 0,
            /// Active low
            ActiveLow = 1
        ]
    ],
    ENABLE [
        ENABLE OFFSET(0) NUMBITS(4) [
            Disable = 0,
            Enable = 7
        ]
    ],
    EVENT [
        EVENT 0
    ],
    TASK [
        TASK 0
    ]
];

/// An enum representing all allowable `frequency` register values.
#[repr(u32)]
#[derive(Copy, Clone)]
pub enum Frequency {
    K125 = 0x02000000,
    K250 = 0x04000000,
    K500 = 0x08000000,
    M1 = 0x10000000,
    M2 = 0x20000000,
    M4 = 0x40000000,
    M8 = 0x80000000,
}

impl Frequency {
    pub fn from_register(reg: u32) -> Option<Frequency> {
        match reg {
            0x02000000 => Some(Frequency::K125),
            0x04000000 => Some(Frequency::K250),
            0x08000000 => Some(Frequency::K500),
            0x10000000 => Some(Frequency::M1),
            0x20000000 => Some(Frequency::M2),
            0x40000000 => Some(Frequency::M4),
            0x80000000 => Some(Frequency::M8),
            _ => None,
        }
    }

    pub fn into_spi_rate(&self) -> u32 {
        match *self {
            Frequency::K125 => 125_000,
            Frequency::K250 => 250_000,
            Frequency::K500 => 500_000,
            Frequency::M1 => 1_000_000,
            Frequency::M2 => 2_000_000,
            Frequency::M4 => 4_000_000,
            Frequency::M8 => 8_000_000,
        }
    }

    pub fn from_spi_rate(freq: u32) -> Frequency {
        if freq < 250_000 {
            Frequency::K125
        } else if freq < 500_000 {
            Frequency::K250
        } else if freq < 1_000_000 {
            Frequency::K500
        } else if freq < 2_000_000 {
            Frequency::M1
        } else if freq < 4_000_000 {
            Frequency::M2
        } else if freq < 8_000_000 {
            Frequency::M4
        } else {
            Frequency::M8
        }
    }
}

/// A SPI master device.
///
/// A `SPIM` instance wraps a `registers::spim::SPIM` together with
/// addition data necessary to implement an asynchronous interface.
pub struct SPIM<'a> {
    registers: StaticRef<SpimRegisters>,
    client: OptionalCell<&'a dyn hil::spi::SpiMasterClient>,
    chip_select: OptionalCell<ChipSelectPolar<&'a crate::gpio::GPIOPin<'a>>>,
    busy: Cell<bool>,
    tx_buf: TakeCell<'static, [u8]>,
    rx_buf: TakeCell<'static, [u8]>,
    transfer_len: Cell<usize>,
}

impl<'a> SPIM<'a> {
    pub const fn new(instance: usize) -> SPIM<'a> {
        SPIM {
            registers: INSTANCES[instance],
            client: OptionalCell::empty(),
            chip_select: OptionalCell::empty(),
            busy: Cell::new(false),
            tx_buf: TakeCell::empty(),
            rx_buf: TakeCell::empty(),
            transfer_len: Cell::new(0),
        }
    }

    #[inline(never)]
    pub fn handle_interrupt(&self) {
        if self.registers.events_end.is_set(EVENT::EVENT) {
            // End of RXD buffer and TXD buffer reached

            if self.chip_select.is_none() {
                debug_assert!(false, "Invariant violated. Chip-select must be Some.");
                return;
            }

            self.chip_select.map(|cs| cs.deactivate());
            self.registers.events_end.write(EVENT::EVENT::CLEAR);

            // When we are no longer active or busy we can disable the
            // peripheral.
            self.disable();
            self.busy.set(false);

            self.client.map(|client| match self.tx_buf.take() {
                None => (),
                Some(tx_buf) => client.read_write_done(
                    tx_buf,
                    self.rx_buf.take(),
                    self.transfer_len.take(),
                    Ok(()),
                ),
            });
        }

        // Although we only configured the chip interrupt on the
        // above 'end' event, the other event fields also get set by
        // the chip. Let's clear those flags.

        if self.registers.events_stopped.is_set(EVENT::EVENT) {
            // SPI transaction has stopped
            self.registers.events_stopped.write(EVENT::EVENT::CLEAR);
        }

        if self.registers.events_endrx.is_set(EVENT::EVENT) {
            // End of RXD buffer reached
            self.registers.events_endrx.write(EVENT::EVENT::CLEAR);
        }

        if self.registers.events_endtx.is_set(EVENT::EVENT) {
            // End of TXD buffer reached
            self.registers.events_endtx.write(EVENT::EVENT::CLEAR);
        }

        if self.registers.events_started.is_set(EVENT::EVENT) {
            // Transaction started
            self.registers.events_started.write(EVENT::EVENT::CLEAR);
        }
    }

    /// Configures an already constructed `SPIM`.
    pub fn configure(&self, mosi: Pinmux, miso: Pinmux, sck: Pinmux) {
        self.registers.psel_mosi.set(mosi);
        self.registers.psel_miso.set(miso);
        self.registers.psel_sck.set(sck);
    }

    /// Enables `SPIM` peripheral.
    pub fn enable(&self) {
        self.registers.enable.write(ENABLE::ENABLE::Enable);
    }

    /// Disables `SPIM` peripheral.
    pub fn disable(&self) {
        self.registers.enable.write(ENABLE::ENABLE::Disable);
    }

    pub fn is_enabled(&self) -> bool {
        self.registers.enable.matches_all(ENABLE::ENABLE::Enable)
    }
}

impl<'a> hil::spi::SpiMaster<'a> for SPIM<'a> {
    type ChipSelect = ChipSelectPolar<&'a crate::gpio::GPIOPin<'a>>;

    fn set_client(&self, client: &'a dyn hil::spi::SpiMasterClient) {
        self.client.set(client);
    }

    fn init(&self) -> Result<(), ErrorCode> {
        Ok(())
    }

    fn is_busy(&self) -> bool {
        self.busy.get()
    }

    fn read_write_bytes(
        &self,
        tx_buf: &'static mut [u8],
        rx_buf: Option<&'static mut [u8]>,
        len: usize,
    ) -> Result<(), (ErrorCode, &'static mut [u8], Option<&'static mut [u8]>)> {
        debug_assert!(!self.busy.get());
        debug_assert!(self.tx_buf.is_none());
        debug_assert!(self.rx_buf.is_none());

        // Clear (set to low) chip-select
        if self.chip_select.is_none() {
            return Err((ErrorCode::NODEVICE, tx_buf, rx_buf));
        }
        self.chip_select.map(|cs| cs.activate());

        // Setup transmit data registers
        let tx_len: u32 = cmp::min(len, tx_buf.len()) as u32;
        self.registers.txd_ptr.set(tx_buf.as_ptr());
        self.registers.txd_maxcnt.write(MAXCNT::MAXCNT.val(tx_len));
        self.tx_buf.replace(tx_buf);

        // Setup receive data registers
        match rx_buf {
            None => {
                self.registers.rxd_ptr.set(ptr::null_mut());
                self.registers.rxd_maxcnt.write(MAXCNT::MAXCNT.val(0));
                self.transfer_len.set(tx_len as usize);
                self.rx_buf.put(None);
            }
            Some(buf) => {
                self.registers.rxd_ptr.set(buf.as_mut_ptr());
                let rx_len: u32 = cmp::min(len, buf.len()) as u32;
                self.registers.rxd_maxcnt.write(MAXCNT::MAXCNT.val(rx_len));
                self.transfer_len.set(cmp::min(tx_len, rx_len) as usize);
                self.rx_buf.put(Some(buf));
            }
        }

        // Start the transfer
        self.busy.set(true);

        // Start and enable the SPIM peripheral. The SPIM peripheral is only
        // enabled when the busy flag is set.
        self.registers.intenset.write(INTE::END::Enable);
        self.enable();

        self.registers.tasks_start.write(TASK::TASK::SET);
        Ok(())
    }

    fn write_byte(&self, _val: u8) -> Result<(), ErrorCode> {
        unimplemented!("SPI: Use `read_write_bytes()` instead.");
    }

    fn read_byte(&self) -> Result<u8, ErrorCode> {
        unimplemented!("SPI: Use `read_write_bytes()` instead.");
    }

    fn read_write_byte(&self, _val: u8) -> Result<u8, ErrorCode> {
        unimplemented!("SPI: Use `read_write_bytes()` instead.");
    }

    // Tell the SPI peripheral what to use as a chip select pin.
    // The type of the argument is based on what makes sense for the
    // peripheral when this trait is implemented.
    fn specify_chip_select(&self, cs: Self::ChipSelect) -> Result<(), ErrorCode> {
        cs.pin.make_output();
        cs.deactivate();
        self.chip_select.set(cs);
        Ok(())
    }

    // Returns the actual rate set
    fn set_rate(&self, rate: u32) -> Result<u32, ErrorCode> {
        let f = Frequency::from_spi_rate(rate);
        self.registers.frequency.set(f as u32);
        Ok(f.into_spi_rate())
    }

    fn get_rate(&self) -> u32 {
        // Reset value is a valid frequency (250kbps), so .expect
        // should be safe here
        let f = Frequency::from_register(self.registers.frequency.get()).unwrap(); // Unwrap fail = nrf52 unknown spi rate
        f.into_spi_rate()
    }

    fn set_polarity(&self, polarity: hil::spi::ClockPolarity) -> Result<(), ErrorCode> {
        let new_polarity = match polarity {
            hil::spi::ClockPolarity::IdleLow => CONFIG::CPOL::ActiveHigh,
            hil::spi::ClockPolarity::IdleHigh => CONFIG::CPOL::ActiveLow,
        };
        self.registers.config.modify(new_polarity);
        Ok(())
    }

    fn get_polarity(&self) -> hil::spi::ClockPolarity {
        match self.registers.config.read(CONFIG::CPOL) {
            0 => hil::spi::ClockPolarity::IdleLow,
            1 => hil::spi::ClockPolarity::IdleHigh,
            _ => unreachable!(),
        }
    }

    fn set_phase(&self, phase: hil::spi::ClockPhase) -> Result<(), ErrorCode> {
        let new_phase = match phase {
            hil::spi::ClockPhase::SampleLeading => CONFIG::CPHA::SampleOnLeadingEdge,
            hil::spi::ClockPhase::SampleTrailing => CONFIG::CPHA::SampleOnTrailingEdge,
        };
        self.registers.config.modify(new_phase);
        Ok(())
    }

    fn get_phase(&self) -> hil::spi::ClockPhase {
        match self.registers.config.read(CONFIG::CPHA) {
            0 => hil::spi::ClockPhase::SampleLeading,
            1 => hil::spi::ClockPhase::SampleTrailing,
            _ => unreachable!(),
        }
    }

    // The following two trait functions are not implemented for
    // SAM4L, and appear to not provide much functionality. Let's not
    // bother implementing them unless needed.
    fn hold_low(&self) {
        unimplemented!("SPI: Use `read_write_bytes()` instead.");
    }

    fn release_low(&self) {
        unimplemented!("SPI: Use `read_write_bytes()` instead.");
    }
}
