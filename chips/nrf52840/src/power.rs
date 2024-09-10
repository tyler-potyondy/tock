use core::{
    cell::Cell,
    fmt,
    fmt::{Arguments, Display, Formatter},
};

use kernel::utilities::registers::PowerManager;
use nrf52::uart;

use crate::ieee802154_radio;

pub struct PowerManagerObj {
    uart_power: Cell<u32>,
    ieee802154_radio_power: Cell<u32>,
}

impl PowerManagerObj {
    pub fn new() -> Self {
        Self {
            uart_power: Cell::new(0),
            ieee802154_radio_power: Cell::new(0),
        }
    }
}

impl fmt::Debug for PowerManagerObj {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "UARTE0: {} uA\r\n 15.4 Radio: {} uA\r\n",
            self.uart_power.get(),
            "N/A"
        )
    }
}

impl PowerManager for PowerManagerObj {
    fn update(&self, addr: u32, power: usize) {
        match addr & 0xFFFFF000 {
            uart::UARTE0_BASE_ADDR => {
                self.uart_power.set(power as u32);
            }
            ieee802154_radio::RADIO_BASE_ADDR => {
                self.ieee802154_radio_power.set(power as u32);
            }

            _ => unimplemented!(),
        }
    }
}
