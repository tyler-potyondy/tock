// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Non-Volatile Memory Controller
//!
//! Used in order read and write to internal flash.

use core::cell::Cell;
use core::ops::{Index, IndexMut};
use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::hil;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::cells::TakeCell;
use kernel::utilities::cells::VolatileCell;
use kernel::utilities::registers::interfaces::{Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, ReadOnly, ReadWrite};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

const NVMC_BASE: StaticRef<NvmcRegisters> =
    unsafe { StaticRef::new(0x4001E400 as *const NvmcRegisters) };

#[repr(C)]
struct NvmcRegisters {
    /// Ready flag
    /// Address 0x400 - 0x404
    pub ready: ReadOnly<u32, Ready::Register>,
    _reserved0: [u32; 4],
    /// Ready flag
    /// Address 0x408 - 0x40C
    pub ready_next: ReadOnly<u32, Ready::Register>,
    /// Reserved
    _reserved1: [u32; 59],
    /// Configuration register
    /// Address: 0x504 - 0x508
    pub config: ReadWrite<u32, Configuration::Register>,
    /// Register for erasing a page in Code area
    /// Address: 0x508 - 0x50C
    pub erasepage: ReadWrite<u32, ErasePage::Register>,
    /// Register for erasing all non-volatile user memory
    /// Address: 0x50C - 0x510
    pub eraseall: ReadWrite<u32, EraseAll::Register>,
    _reserved2: u32,
    /// Register for erasing User Information Configuration Registers
    /// Address: 0x514 - 0x518
    pub eraseuicr: ReadWrite<u32, EraseUicr::Register>,
    /// Reserved
    _reserved3: [u32; 10],
    /// Configuration register
    /// Address: 0x540 - 0x544
    pub icachecnf: ReadWrite<u32, CacheConfiguration::Register>,
    /// Reserved
    _reserved4: u32,
    /// Configuration register
    /// Address: 0x548 - 0x54c
    pub ihit: ReadWrite<u32, CacheHit::Register>,
    /// Configuration register
    /// Address: 0x54C - 0x550
    pub imiss: ReadWrite<u32, CacheMiss::Register>,
}

register_bitfields! [u32,
    /// Ready flag
    Ready [
        /// NVMC is ready or busy
        READY OFFSET(0) NUMBITS(1) [
            /// NVMC is busy (on-going write or erase operation)
            BUSY = 0,
            /// NVMC is ready
            READY = 1
        ]
    ],
    /// Configuration register
    Configuration [
        /// Program memory access mode. It is strongly recommended
        /// to only activate erase and write modes when they are actively
        /// used. Enabling write or erase will invalidate the cache and keep
        /// it invalidated.
        WEN OFFSET(0) NUMBITS(2) [
            /// Read only access
            Ren = 0,
            /// Write Enabled
            Wen = 1,
            /// Erase enabled
            Een = 2
        ]
    ],
    /// Register for erasing a page in Code area
    ErasePage [
        /// Register for starting erase of a page in Code area
        ERASEPAGE OFFSET(0) NUMBITS(32) []
    ],
    /// Register for erasing all non-volatile user memory
    EraseAll [
        /// Erase all non-volatile memory including UICR registers. Note
        /// that code erase has to be enabled by CONFIG.EEN before the
        /// UICR can be erased
        ERASEALL OFFSET(0) NUMBITS(1) [
            /// No operation
            NOOPERATION = 0,
            /// Start chip erase
            ERASE = 1
        ]
    ],
    /// Register for erasing User Information Configuration Registers
    EraseUicr [
        /// Register starting erase of all User Information Configuration Registers.
        /// Note that code erase has to be enabled by CONFIG.EEN before the UICR can be erased
        ERASEUICR OFFSET(0) NUMBITS(1) [
            /// No operation
            NOOPERATION = 0,
            /// Start erase of UICR
            ERASE = 1
        ]
    ],
    /// I-Code cache configuration register
    CacheConfiguration [
        /// Cache enabled
        CACHEEN OFFSET(0) NUMBITS(1) [
            /// Disable cache. Invalidates all cache entries
            DISABLED = 0,
            /// Enable cache
            ENABLED = 1
        ],
        /// Cache profiling enable
        CACHEPROFEN OFFSET(8) NUMBITS(1) [
            /// Disable cache profiling
            DISABLED = 0,
            /// Enable cache profiling
            ENABLED = 1
        ]
    ],
    /// I-Code cache hit counter
    CacheHit [
        /// Number of cache hits
        HITS OFFSET(0) NUMBITS(32) []
    ],
    /// I-Code cache miss counter
    CacheMiss [
        /// Number of cache misses
        MISSES OFFSET(0) NUMBITS(32) []
    ]
];

const PAGE_SIZE: usize = 4096;

/// This is a wrapper around a u8 array that is sized to a single page for the
/// nrf. Users of this module must pass an object of this type to use the
/// `hil::flash::Flash` interface.
///
/// An example looks like:
///
/// ```rust
/// # extern crate nrf52;
/// # use nrf52::nvmc::NrfPage;
/// # use kernel::static_init;
///
/// let pagebuffer = unsafe { static_init!(NrfPage, NrfPage::default()) };
/// ```
pub struct NrfPage(pub [u8; PAGE_SIZE]);

impl Default for NrfPage {
    fn default() -> Self {
        Self([0; PAGE_SIZE])
    }
}
impl NrfPage {
    fn len(&self) -> usize {
        self.0.len()
    }
}

impl Index<usize> for NrfPage {
    type Output = u8;

    fn index(&self, idx: usize) -> &u8 {
        &self.0[idx]
    }
}

impl IndexMut<usize> for NrfPage {
    fn index_mut(&mut self, idx: usize) -> &mut u8 {
        &mut self.0[idx]
    }
}

impl AsMut<[u8]> for NrfPage {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

/// FlashState is used to track the current state and command of the flash.
#[derive(Clone, Copy, PartialEq)]
pub enum FlashState {
    Ready, // Flash is ready to complete a command.
    Read,  // Performing a read operation.
    Write, // Performing a write operation.
    Erase, // Performing an erase operation.
}

pub struct Nvmc {
    registers: StaticRef<NvmcRegisters>,
    client: OptionalCell<&'static dyn hil::flash::Client<Nvmc>>,
    buffer: TakeCell<'static, NrfPage>,
    state: Cell<FlashState>,
    deferred_call: DeferredCall,
}

impl Nvmc {
    pub fn new() -> Self {
        Self {
            registers: NVMC_BASE,
            client: OptionalCell::empty(),
            buffer: TakeCell::empty(),
            state: Cell::new(FlashState::Ready),
            deferred_call: DeferredCall::new(),
        }
    }

    /// Configure the NVMC to allow writes to flash.
    pub fn configure_writeable(&self) {
        self.registers.config.write(Configuration::WEN::Wen);
    }

    pub fn configure_eraseable(&self) {
        self.registers.config.write(Configuration::WEN::Een);
    }

    pub fn erase_uicr(&self) {
        self.registers.config.write(Configuration::WEN::Een);
        while !self.is_ready() {}
        self.registers
            .erasepage
            .write(ErasePage::ERASEPAGE.val(0x10001000));
        while !self.is_ready() {}
    }

    /// Check if there is an ongoing operation with the NVMC peripheral.
    pub fn is_ready(&self) -> bool {
        self.registers.ready.is_set(Ready::READY)
    }

    pub fn handle_interrupt(&self) {
        let state = self.state.get();
        self.state.set(FlashState::Ready);

        match state {
            FlashState::Read => {
                self.client.map(|client| {
                    self.buffer.take().map(|buffer| {
                        client.read_complete(buffer, Ok(()));
                    });
                });
            }
            FlashState::Write => {
                self.client.map(|client| {
                    self.buffer.take().map(|buffer| {
                        client.write_complete(buffer, Ok(()));
                    });
                });
            }
            FlashState::Erase => {
                self.client.map(|client| {
                    client.erase_complete(Ok(()));
                });
            }
            _ => {}
        }
    }

    fn erase_page_helper(&self, page_number: usize) {
        // Put the NVMC in erase mode.
        self.registers.config.write(Configuration::WEN::Een);

        // Tell the NVMC to erase the correct page by passing in the correct
        // address.
        self.registers
            .erasepage
            .write(ErasePage::ERASEPAGE.val((page_number * PAGE_SIZE) as u32));

        // Make sure that the NVMC is done. The CPU should be blocked while the
        // erase is happening, but it doesn't hurt to check too.
        while !self.registers.ready.is_set(Ready::READY) {}
    }

    fn read_range(
        &self,
        page_number: usize,
        buffer: &'static mut NrfPage,
    ) -> Result<(), (ErrorCode, &'static mut NrfPage)> {
        // Actually do a copy from flash into the buffer.
        let mut byte: *const u8 = (page_number * PAGE_SIZE) as *const u8;
        unsafe {
            for i in 0..buffer.len() {
                buffer[i] = *byte;
                byte = byte.offset(1);
            }
        }

        // Hold on to the buffer for the callback.
        self.buffer.replace(buffer);

        // Mark the need for an interrupt so we can call the read done
        // callback.
        self.state.set(FlashState::Read);
        self.deferred_call.set();

        Ok(())
    }

    fn write_page(
        &self,
        page_number: usize,
        data: &'static mut NrfPage,
    ) -> Result<(), (ErrorCode, &'static mut NrfPage)> {
        // Need to erase the page first.
        self.erase_page_helper(page_number);

        // Put the NVMC in write mode.
        self.registers.config.write(Configuration::WEN::Wen);

        for i in (0..data.len()).step_by(4) {
            let word: u32 = (data[i + 0] as u32) << 0
                | (data[i + 1] as u32) << 8
                | (data[i + 2] as u32) << 16
                | (data[i + 3] as u32) << 24;

            let address = ((page_number * PAGE_SIZE) + i) as u32;
            let location = unsafe { &*(address as *const VolatileCell<u32>) };
            location.set(word);
            while !self.registers.ready.is_set(Ready::READY) {}
        }

        // Make sure that the NVMC is done. The CPU should be blocked while the
        // write is happening, but it doesn't hurt to check too.
        while !self.registers.ready.is_set(Ready::READY) {}

        // Save the buffer so we can return it with the callback.
        self.buffer.replace(data);

        // Mark the need for an interrupt so we can call the write done
        // callback.
        self.state.set(FlashState::Write);
        self.deferred_call.set();

        Ok(())
    }

    fn erase_page(&self, page_number: usize) -> Result<(), ErrorCode> {
        // Do the basic erase.
        self.erase_page_helper(page_number);

        // Mark that we want to trigger a pseudo interrupt so that we can issue
        // the callback even though the NVMC is completely blocking.
        self.state.set(FlashState::Erase);
        self.deferred_call.set();

        Ok(())
    }
}

impl<C: hil::flash::Client<Self>> hil::flash::HasClient<'static, C> for Nvmc {
    fn set_client(&self, client: &'static C) {
        self.client.set(client);
    }
}

impl hil::flash::Flash for Nvmc {
    type Page = NrfPage;

    fn read_page(
        &self,
        page_number: usize,
        buf: &'static mut Self::Page,
    ) -> Result<(), (ErrorCode, &'static mut Self::Page)> {
        self.read_range(page_number, buf)
    }

    fn write_page(
        &self,
        page_number: usize,
        buf: &'static mut Self::Page,
    ) -> Result<(), (ErrorCode, &'static mut Self::Page)> {
        self.write_page(page_number, buf)
    }

    fn erase_page(&self, page_number: usize) -> Result<(), ErrorCode> {
        self.erase_page(page_number)
    }
}

impl DeferredCallClient for Nvmc {
    fn handle_deferred_call(&self) {
        self.handle_interrupt();
    }

    fn register(&'static self) {
        self.deferred_call.register(self);
    }
}
