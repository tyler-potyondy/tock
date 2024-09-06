struct PowerManager {}

impl PowerManager {
    // somehow we need this to only be created once
    pub fn new() -> Self {
        PowerManager {}
    }

    fn update_power(&self) {}
}
