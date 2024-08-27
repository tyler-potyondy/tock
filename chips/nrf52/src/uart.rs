// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Universal asynchronous receiver/transmitter with EasyDMA (UARTE)
//!
//! Author
//! -------------------
//!
//! * Author: Niklas Adolfsson <niklasadolfsson1@gmail.com>
//! * Date: March 10 2018

use core::cell::Cell;
use core::cmp::min;
use kernel::hil::uart;
use kernel::utilities::cells::{MapCell, OptionalCell};
use kernel::utilities::registers::interfaces::{Readable, Writeable};
use kernel::utilities::registers::{
    register_bitfields, PowerControl, PowerOff, PowerOn, PowerWrite, PrePowerConfig, ReadOnly,
    ReadWrite, UartReadOnly, UartReadWrite, UartWriteOnly, WriteOnly,
};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;
use nrf5x::pinmux;

use crate::power::{self, Power};

const UARTE_MAX_BUFFER_SIZE: u32 = 0xff;

static mut BYTE: u8 = 0;

//pub const UARTE0_BASE: StaticRef<UarteRegisters> =
//    unsafe { StaticRef::new(0x40002000 as *const UarteRegisters) };
pub const UART0_BASE_ADDR: u32 = 0x40002000;

enum UartState {
    Disabled(PowerOff<UarteRegisters>),
    Transmit(PowerOn<UarteRegisters>),
    Receive(PowerOn<UarteRegisters>),
    Aborting,
    Idle,
}

#[repr(C)]
pub struct UarteRegisters {
    task_startrx: UartWriteOnly<u32, Self, Task::Register>,
    task_stoprx: UartWriteOnly<u32, Self, Task::Register>,
    task_starttx: UartWriteOnly<u32, Self, Task::Register>,
    task_stoptx: UartWriteOnly<u32, Self, Task::Register>,
    _reserved1: [u32; 7],
    task_flush_rx: UartWriteOnly<u32, Self, Task::Register>,
    _reserved2: [u32; 52],
    event_cts: UartReadWrite<u32, Self, Event::Register>,
    event_ncts: UartReadWrite<u32, Self, Event::Register>,
    _reserved3: [u32; 2],
    event_endrx: UartReadWrite<u32, Self, Event::Register>,
    _reserved4: [u32; 3],
    event_endtx: UartReadWrite<u32, Self, Event::Register>,
    event_error: UartReadWrite<u32, Self, Event::Register>,
    _reserved6: [u32; 7],
    event_rxto: UartReadWrite<u32, Self, Event::Register>,
    _reserved7: [u32; 1],
    event_rxstarted: UartReadWrite<u32, Self, Event::Register>,
    event_txstarted: UartReadWrite<u32, Self, Event::Register>,
    _reserved8: [u32; 1],
    event_txstopped: UartReadWrite<u32, Self, Event::Register>,
    _reserved9: [u32; 41],
    shorts: UartReadWrite<u32, Self, Shorts::Register>,
    _reserved10: [u32; 64],
    intenset: UartReadWrite<u32, Self, Interrupt::Register>,
    intenclr: UartReadWrite<u32, Self, Interrupt::Register>,
    _reserved11: [u32; 93],
    errorsrc: UartReadWrite<u32, Self, ErrorSrc::Register>,
    _reserved12: [u32; 31],
    enable: PowerWrite<u32, Self, Uart::Register>,
    _reserved13: [u32; 1],
    pselrts: PrePowerConfig<u32, Self, Psel::Register>,
    pseltxd: PrePowerConfig<u32, Self, Psel::Register>,
    pselcts: PrePowerConfig<u32, Self, Psel::Register>,
    pselrxd: PrePowerConfig<u32, Self, Psel::Register>,
    _reserved14: [u32; 3],
    baudrate: UartReadWrite<u32, Self, Baudrate::Register>,
    _reserved15: [u32; 3],
    rxd_ptr: UartReadWrite<u32, Self, Pointer::Register>,
    rxd_maxcnt: UartReadWrite<u32, Self, Counter::Register>,
    rxd_amount: UartReadOnly<u32, Self, Counter::Register>,
    _reserved16: [u32; 1],
    txd_ptr: UartReadWrite<u32, Self, Pointer::Register>,
    txd_maxcnt: UartReadWrite<u32, Self, Counter::Register>,
    txd_amount: UartReadOnly<u32, Self, Counter::Register>,
    _reserved17: [u32; 7],
    config: UartReadWrite<u32, Self, Config::Register>,
}

impl PowerControl<UarteRegisters> for UarteRegisters {
    fn power_off() -> kernel::utilities::registers::PowerOff<UarteRegisters> {
        // Disable the peripheral
        let (raw_registers, power) = UarteRegisters::create(UART0_BASE_ADDR);
        let (registers, power_on) = unsafe {
            let registers = StaticRef::new(raw_registers);
            let power_on =
                core::mem::transmute::<PowerOff<UarteRegisters>, PowerOn<UarteRegisters>>(power);
            (registers, power_on)
        };
        // turn off interrupts (TODO)

        // turn off peripheral
        registers.enable.power_off(power_on, Uart::ENABLE::CLEAR)
    }
}

register_bitfields! [u32,
    /// Start task
    Task [
        ENABLE OFFSET(0) NUMBITS(1)
    ],

    /// Read event
    Event [
        READY OFFSET(0) NUMBITS(1)
    ],

    /// Shortcuts
    Shorts [
        // Shortcut between ENDRX and STARTRX
        ENDRX_STARTRX OFFSET(5) NUMBITS(1),
        // Shortcut between ENDRX and STOPRX
        ENDRX_STOPRX OFFSET(6) NUMBITS(1)
    ],

    /// UART Interrupts
    Interrupt [
        CTS OFFSET(0) NUMBITS(1),
        NCTS OFFSET(1) NUMBITS(1),
        ENDRX OFFSET(4) NUMBITS(1),
        ENDTX OFFSET(8) NUMBITS(1),
        ERROR OFFSET(9) NUMBITS(1),
        RXTO OFFSET(17) NUMBITS(1),
        RXSTARTED OFFSET(19) NUMBITS(1),
        TXSTARTED OFFSET(20) NUMBITS(1),
        TXSTOPPED OFFSET(22) NUMBITS(1)
    ],

    /// UART Errors
    ErrorSrc [
        OVERRUN OFFSET(0) NUMBITS(1),
        PARITY OFFSET(1) NUMBITS(1),
        FRAMING OFFSET(2) NUMBITS(1),
        BREAK OFFSET(3) NUMBITS(1)
    ],

    /// Enable UART
    Uart [
        ENABLE OFFSET(0) NUMBITS(4) [
            ON = 8,
            OFF = 0
        ]
    ],

    /// Pin select
    Psel [
        // Pin number. MSB is actually the port indicator, but since we number
        // pins sequentially the binary representation of the pin number has
        // the port bit set correctly. So, for simplicity we just treat the
        // pin number as a 6 bit field.
        PIN OFFSET(0) NUMBITS(6),
        // Connect/Disconnect
        CONNECT OFFSET(31) NUMBITS(1)
    ],

    /// Baudrate
    Baudrate [
        BAUDRAUTE OFFSET(0) NUMBITS(32)
    ],

    /// DMA pointer
    Pointer [
        POINTER OFFSET(0) NUMBITS(32)
    ],

    /// Counter value
    Counter [
        COUNTER OFFSET(0) NUMBITS(8)
    ],

    /// Configuration of parity and flow control
    Config [
        HWFC OFFSET(0) NUMBITS(1),
        PARITY OFFSET(1) NUMBITS(3)
    ]
];

/// UARTE
// It should never be instanced outside this module but because a static mutable reference to it
// is exported outside this module it must be `pub`
pub struct Uarte<'a> {
    registers: StaticRef<UarteRegisters>,
    tx_client: OptionalCell<&'a dyn uart::TransmitClient<UarteRegisters>>,
    tx_buffer: kernel::utilities::cells::TakeCell<'static, [u8]>,
    tx_len: Cell<usize>,
    tx_remaining_bytes: Cell<usize>,
    rx_client: OptionalCell<&'a dyn uart::ReceiveClient>,
    rx_buffer: kernel::utilities::cells::TakeCell<'static, [u8]>,
    rx_remaining_bytes: Cell<usize>,
    offset: Cell<usize>,
    uart_state: MapCell<UartState>,
}

#[derive(Copy, Clone)]
pub struct UARTParams {
    pub baud_rate: u32,
}

impl<'a> Uarte<'a> {
    /// Constructor
    // This should only be constructed once
    pub fn new(reg_addr: u32) -> Uarte<'a> {
        let (raw_registers, power_off) = UarteRegisters::create(reg_addr);
        let registers = unsafe { StaticRef::new(raw_registers) };

        Uarte {
            registers: registers,
            tx_client: OptionalCell::empty(),
            tx_buffer: kernel::utilities::cells::TakeCell::empty(),
            tx_len: Cell::new(0),
            tx_remaining_bytes: Cell::new(0),
            rx_client: OptionalCell::empty(),
            rx_buffer: kernel::utilities::cells::TakeCell::empty(),
            rx_remaining_bytes: Cell::new(0),
            offset: Cell::new(0),
            uart_state: MapCell::new(UartState::Disabled(power_off)),
        }
    }

    /// Configure which pins the UART should use for txd, rxd, cts and rts
    pub fn initialize(
        &self,
        txd: pinmux::Pinmux,
        rxd: pinmux::Pinmux,
        cts: Option<pinmux::Pinmux>,
        rts: Option<pinmux::Pinmux>,
    ) -> PowerOn<UarteRegisters> {
        self.uart_state
            .take()
            .map(|state| {
                let mut power_off = match state {
                    UartState::Disabled(power_off) => power_off,
                    _ => unreachable!(),
                };

                power_off = self
                    .registers
                    .pseltxd
                    .write(power_off, Psel::PIN.val(txd.into()));

                power_off = self
                    .registers
                    .pselrxd
                    .write(power_off, Psel::PIN.val(rxd.into()));

                power_off = match cts {
                    // If no CTS pin is provided, then we need to mark it as
                    // disconnected in the register.
                    Some(c) => self
                        .registers
                        .pselcts
                        .write(power_off, Psel::PIN.val(c.into())),
                    None => self.registers.pselcts.write(power_off, Psel::CONNECT::SET),
                };

                power_off = match rts {
                    // If no RTS pin is provided, then we need to mark it as
                    // disconnected in the register.
                    Some(r) => self
                        .registers
                        .pselrts
                        .write(power_off, Psel::PIN.val(r.into())),
                    None => self.registers.pselrts.write(power_off, Psel::CONNECT::SET),
                };

                self.enable_uart(power_off)
            })
            .unwrap()
        // Make sure we clear the endtx interrupt since that is what we rely on
        // to know when the DMA TX finishes. Normally, we clear this interrupt
        // as we handle it, so this is not necessary. However, a bootloader (or
        // some other startup code) may have setup TX interrupts, and there may
        // be one pending. We clear it to be safe.
        // self.registers.event_endtx.write(Event::READY::CLEAR);
    }

    fn set_baud_rate(
        &self,
        power_on: PowerOn<UarteRegisters>,
        baud_rate: u32,
    ) -> PowerOn<UarteRegisters> {
        match baud_rate {
            1200 => self.registers.baudrate.set(0x0004F000, power_on),
            2400 => self.registers.baudrate.set(0x0009D000, power_on),
            4800 => self.registers.baudrate.set(0x0013B000, power_on),
            9600 => self.registers.baudrate.set(0x00275000, power_on),
            14400 => self.registers.baudrate.set(0x003AF000, power_on),
            19200 => self.registers.baudrate.set(0x004EA000, power_on),
            28800 => self.registers.baudrate.set(0x0075C000, power_on),
            38400 => self.registers.baudrate.set(0x009D0000, power_on),
            57600 => self.registers.baudrate.set(0x00EB0000, power_on),
            76800 => self.registers.baudrate.set(0x013A9000, power_on),
            115200 => self.registers.baudrate.set(0x01D60000, power_on),
            230400 => self.registers.baudrate.set(0x03B00000, power_on),
            250000 => self.registers.baudrate.set(0x04000000, power_on),
            460800 => self.registers.baudrate.set(0x07400000, power_on),
            921600 => self.registers.baudrate.set(0x0F000000, power_on),
            1000000 => self.registers.baudrate.set(0x10000000, power_on),
            _ => self.registers.baudrate.set(0x01D60000, power_on), //setting default to 115200
        }
    }

    // Enable UART peripheral, this need to disabled for low power applications
    fn enable_uart(&self, power_off: PowerOff<UarteRegisters>) -> PowerOn<UarteRegisters> {
        self.uart_state.replace(UartState::Idle);
        self.registers.enable.power_on(power_off, Uart::ENABLE::ON)
    }

    #[allow(dead_code)]
    fn disable_uart(&self, power_on: PowerOn<UarteRegisters>) {
        let power_off = self.registers.enable.power_off(power_on, Uart::ENABLE::OFF);
        self.uart_state.replace(UartState::Disabled(power_off));
    }

    fn enable_rx_interrupts(&self, power_on: PowerOn<UarteRegisters>) -> PowerOn<UarteRegisters> {
        self.registers
            .intenset
            .write(Interrupt::ENDRX::SET, power_on)
    }

    fn enable_tx_interrupts(&self, power_on: PowerOn<UarteRegisters>) -> PowerOn<UarteRegisters> {
        self.registers
            .intenset
            .write(Interrupt::ENDTX::SET, power_on)
    }

    fn disable_rx_interrupts(&self, power_on: PowerOn<UarteRegisters>) -> PowerOn<UarteRegisters> {
        self.registers
            .intenclr
            .write(Interrupt::ENDRX::SET, power_on)
    }

    fn disable_tx_interrupts(&self, power_on: PowerOn<UarteRegisters>) -> PowerOn<UarteRegisters> {
        self.registers
            .intenclr
            .write(Interrupt::ENDTX::SET, power_on)
    }

    /// UART interrupt handler that listens for both tx_end and rx_end events
    #[inline(never)]
    pub fn handle_interrupt(&self) {
        match self.uart_state.take().unwrap() {
            UartState::Transmit(mut power_on) => {
                if self.tx_ready() {
                    power_on = self.disable_tx_interrupts(power_on);
                    power_on = self
                        .registers
                        .event_endtx
                        .write(Event::READY::CLEAR, power_on);
                    let tx_bytes = self.registers.txd_amount.get() as usize;

                    let rem = match self.tx_remaining_bytes.get().checked_sub(tx_bytes) {
                        None => return,
                        Some(r) => r,
                    };

                    // All bytes have been transmitted
                    if rem == 0 {
                        // Signal client write done
                        // disable uart here
                        self.uart_state.replace(UartState::Idle);
                        self.tx_client.map(|client| {
                            self.tx_buffer.take().map(|tx_buffer| {
                                client.transmitted_buffer(
                                    tx_buffer,
                                    self.tx_len.get(),
                                    Ok(()),
                                    power_on,
                                );
                            });
                        });
                    } else {
                        // Not all bytes have been transmitted then update offset and continue transmitting
                        self.offset.set(self.offset.get() + tx_bytes);
                        self.tx_remaining_bytes.set(rem);
                        power_on = self.set_tx_dma_pointer_to_buffer(power_on);
                        power_on = self.registers.txd_maxcnt.write(
                            Counter::COUNTER.val(min(rem as u32, UARTE_MAX_BUFFER_SIZE)),
                            power_on,
                        );

                        power_on = self
                            .registers
                            .task_starttx
                            .write(Task::ENABLE::SET, power_on);
                        power_on = self.enable_tx_interrupts(power_on);
                        self.uart_state.replace(UartState::Transmit(power_on));
                    }
                }
            }
            UartState::Receive(mut power_on) => {
                if self.rx_ready() {
                    power_on = self.disable_rx_interrupts(power_on);

                    // Clear the ENDRX event
                    power_on = self
                        .registers
                        .event_endrx
                        .write(Event::READY::CLEAR, power_on);

                    // Get the number of bytes in the buffer that was received this time
                    let rx_bytes = self.registers.rxd_amount.get() as usize;

                    // Check if this ENDRX is due to an abort. If so, we want to
                    // do the receive callback immediately.
                    if let UartState::Aborting = self.uart_state.take().unwrap() {
                        self.uart_state.replace(UartState::Idle);
                        self.rx_client.map(|client| {
                            self.uart_state.replace(UartState::Idle);
                            self.rx_buffer.take().map(|rx_buffer| {
                                client.received_buffer(
                                    rx_buffer,
                                    self.offset.get() + rx_bytes,
                                    Err(ErrorCode::CANCEL),
                                    uart::Error::None,
                                );
                            });
                        });
                    } else {
                        // In the normal case, we need to either pass call the callback
                        // or do another read to get more bytes.

                        // Update how many bytes we still need to receive and
                        // where we are storing in the buffer.
                        self.rx_remaining_bytes
                            .set(self.rx_remaining_bytes.get().saturating_sub(rx_bytes));
                        self.offset.set(self.offset.get() + rx_bytes);

                        let rem = self.rx_remaining_bytes.get();
                        if rem == 0 {
                            self.uart_state.replace(UartState::Idle);
                            // Signal client that the read is done
                            self.rx_client.map(|client| {
                                self.rx_buffer.take().map(|rx_buffer| {
                                    client.received_buffer(
                                        rx_buffer,
                                        self.offset.get(),
                                        Ok(()),
                                        uart::Error::None,
                                    );
                                });
                            });
                        } else {
                            // Setup how much we can read. We already made sure that
                            // this will fit in the buffer.
                            let to_read = core::cmp::min(rem, 255);
                            power_on = self
                                .registers
                                .rxd_maxcnt
                                .write(Counter::COUNTER.val(to_read as u32), power_on);

                            // Actually do the receive.
                            power_on = self.set_rx_dma_pointer_to_buffer(power_on);
                            power_on = self
                                .registers
                                .task_startrx
                                .write(Task::ENABLE::SET, power_on);
                            power_on = self.enable_rx_interrupts(power_on);
                            self.uart_state.replace(UartState::Receive(power_on));
                        }
                    }
                }
            }
            _ => {}
        }
    }

    /// Transmit one byte at the time and the client is responsible for polling
    /// This is used by the panic handler
    pub unsafe fn send_byte(&self, byte: u8, power_on: PowerOn<UarteRegisters>) {
        self.tx_remaining_bytes.set(1);
        let mut power_on = self
            .registers
            .event_endtx
            .write(Event::READY::CLEAR, power_on);
        // precaution: copy value into variable with static lifetime
        BYTE = byte;

        power_on = self
            .registers
            .txd_ptr
            .set(core::ptr::addr_of!(BYTE) as u32, power_on);

        power_on = self
            .registers
            .txd_maxcnt
            .write(Counter::COUNTER.val(1), power_on);

        power_on = self
            .registers
            .task_starttx
            .write(Task::ENABLE::SET, power_on);

        self.uart_state.replace(UartState::Transmit(power_on));
    }

    /// Check if the UART transmission is done
    pub fn tx_ready(&self) -> bool {
        self.registers.event_endtx.is_set(Event::READY)
    }

    /// Check if either the rx_buffer is full or the UART has timed out
    pub fn rx_ready(&self) -> bool {
        self.registers.event_endrx.is_set(Event::READY)
    }

    fn set_tx_dma_pointer_to_buffer(
        &self,
        power_on: PowerOn<UarteRegisters>,
    ) -> PowerOn<UarteRegisters> {
        self.tx_buffer
            .map(|tx_buffer| {
                self.registers
                    .txd_ptr
                    .set(tx_buffer[self.offset.get()..].as_ptr() as u32, power_on)
            })
            .unwrap()
    }

    fn set_rx_dma_pointer_to_buffer(
        &self,
        power_on: PowerOn<UarteRegisters>,
    ) -> PowerOn<UarteRegisters> {
        self.rx_buffer
            .map(|rx_buffer| {
                self.registers
                    .rxd_ptr
                    .set(rx_buffer[self.offset.get()..].as_ptr() as u32, power_on)
            })
            .unwrap()
    }

    // Helper function used by both transmit_word and transmit_buffer
    fn setup_buffer_transmit(
        &self,
        buf: &'static mut [u8],
        tx_len: usize,
        power_on: PowerOn<UarteRegisters>,
    ) {
        self.tx_remaining_bytes.set(tx_len);
        self.tx_len.set(tx_len);
        self.offset.set(0);
        self.tx_buffer.replace(buf);
        let mut power_on = self.set_tx_dma_pointer_to_buffer(power_on);

        power_on = self.registers.txd_maxcnt.write(
            Counter::COUNTER.val(min(tx_len as u32, UARTE_MAX_BUFFER_SIZE)),
            power_on,
        );

        power_on = self
            .registers
            .task_starttx
            .write(Task::ENABLE::SET, power_on);
        power_on = self.enable_tx_interrupts(power_on);
        self.uart_state.replace(UartState::Transmit(power_on));
    }
}

impl<'a> uart::Transmit<'a, UarteRegisters> for Uarte<'a> {
    fn set_transmit_client(&self, client: &'a dyn uart::TransmitClient<UarteRegisters>) {
        self.tx_client.set(client);
    }

    fn transmit_buffer(
        &self,
        tx_data: &'static mut [u8],
        tx_len: usize,
        power_on: PowerOn<UarteRegisters>,
    ) -> Result<(), (ErrorCode, &'static mut [u8], PowerOn<UarteRegisters>)> {
        if tx_len == 0 || tx_len > tx_data.len() {
            Err((ErrorCode::SIZE, tx_data, power_on))
        } else if self.tx_buffer.is_some() {
            Err((ErrorCode::BUSY, tx_data, power_on))
        } else {
            self.setup_buffer_transmit(tx_data, tx_len, power_on);
            Ok(())
        }
    }

    fn transmit_word(
        &self,
        _data: u32,
        power_on: PowerOn<UarteRegisters>,
    ) -> Result<(), (ErrorCode, PowerOn<UarteRegisters>)> {
        Err((ErrorCode::FAIL, power_on))
    }

    fn transmit_abort(
        &self,
        power_on: PowerOn<UarteRegisters>,
    ) -> Result<(), (ErrorCode, PowerOn<UarteRegisters>)> {
        Err((ErrorCode::FAIL, power_on))
    }
}

impl<'a> uart::Configure<UarteRegisters> for Uarte<'a> {
    fn configure(
        &self,
        params: uart::Parameters,
        power_on: PowerOn<UarteRegisters>,
    ) -> Result<(), ErrorCode> {
        // These could probably be implemented, but are currently ignored, so
        // throw an error.
        if params.stop_bits != uart::StopBits::One {
            return Err(ErrorCode::NOSUPPORT);
        }
        if params.parity != uart::Parity::None {
            return Err(ErrorCode::NOSUPPORT);
        }
        if params.hw_flow_control {
            return Err(ErrorCode::NOSUPPORT);
        }

        self.set_baud_rate(power_on, params.baud_rate);

        Ok(())
    }
}

impl<'a> uart::Receive<'a, UarteRegisters> for Uarte<'a> {
    fn set_receive_client(&self, client: &'a dyn uart::ReceiveClient) {
        self.rx_client.set(client);
    }

    fn receive_buffer(
        &self,
        rx_buf: &'static mut [u8],
        rx_len: usize,
        power_on: PowerOn<UarteRegisters>,
    ) -> Result<(), (ErrorCode, &'static mut [u8])> {
        if self.rx_buffer.is_some() {
            return Err((ErrorCode::BUSY, rx_buf));
        }
        // truncate rx_len if necessary
        let truncated_length = core::cmp::min(rx_len, rx_buf.len());

        self.rx_remaining_bytes.set(truncated_length);
        self.offset.set(0);
        self.rx_buffer.replace(rx_buf);
        let mut power_on = self.set_rx_dma_pointer_to_buffer(power_on);

        let truncated_uart_max_length = core::cmp::min(truncated_length, 255);

        power_on = self.registers.rxd_maxcnt.write(
            Counter::COUNTER.val(truncated_uart_max_length as u32),
            power_on,
        );

        power_on = self
            .registers
            .task_stoprx
            .write(Task::ENABLE::SET, power_on);

        power_on = self
            .registers
            .task_startrx
            .write(Task::ENABLE::SET, power_on);

        power_on = self.enable_rx_interrupts(power_on);

        self.uart_state.replace(UartState::Receive(power_on));
        Ok(())
    }

    fn receive_word(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::FAIL)
    }

    fn receive_abort(&self) -> Result<(), ErrorCode> {
        let mut err = Ok(());
        if let UartState::Receive(power_on) = self.uart_state.take().unwrap() {
            // Trigger the STOPRX event to cancel the current receive call.
            if self.rx_buffer.is_none() {
                return err;
            } else {
                self.uart_state.replace(UartState::Aborting);
                self.registers
                    .task_stoprx
                    .write(Task::ENABLE::SET, power_on);
                err = Err(ErrorCode::BUSY)
            }
            return err;
        } else {
            return Err(ErrorCode::FAIL);
        }
    }
}
