use core::cell::Cell;

use kernel::utilities::registers::PowerManager;
use nrf52::uart;

pub struct PowerManagerObj {
    uart_power: Cell<u32>,
}

impl PowerManagerObj {
    pub fn new() -> Self {
        Self {
            uart_power: Cell::new(0),
        }
    }
}

impl PowerManager for PowerManagerObj {
    fn update(&self, addr: u32, power: usize) {
        match addr & 0xFFFFF000 {
            uart::UARTE0_BASE_ADDR => {
                kernel::debug!("UARTE0 power: {}", power);
                self.uart_power.set(power as u32);
            }
            _ => unimplemented!(),
        }
    }
}
