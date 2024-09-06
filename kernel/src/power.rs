use tock_registers::interfaces::TockRegister;

struct PowerPeripheral {}

impl PowerPeripheral {
    pub fn new() -> Self {
        PowerPeripheral {}
    }

    // Interface with all uart register types
}
struct PowerManager {
    uart: PowerPeripheral,
}

pub struct PowerRegisterSync<const POWER: usize> {
    reg: &'static dyn TockRegister,
}

impl<const POWER: usize> PowerRegisterSync<POWER> {
    pub fn new(reg: &'static dyn TockRegister) -> Self {
        PowerRegisterSync { reg }
    }
}

pub struct PowerRegisterAsync<const POWER: usize> {
    reg: &'static dyn TockRegister,
}

impl<const POWER: usize> PowerRegisterAsync<POWER> {
    pub fn new(reg: &'static dyn TockRegister) -> Self {
        PowerRegisterAsync { reg }
    }
}
