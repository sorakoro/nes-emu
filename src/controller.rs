pub struct Controller {
    button_state: u8,
    shift_register: u8,
    strobe: bool,
}

impl Controller {
    pub fn new() -> Self {
        Controller {
            button_state: 0,
            shift_register: 0,
            strobe: false,
        }
    }

    pub fn write(&mut self, value: u8) {
        self.strobe = value & 1 == 1;
        if self.strobe {
            self.shift_register = self.button_state;
        }
    }

    pub fn read(&mut self) -> u8 {
        if self.strobe {
            return self.button_state & 1;
        }
        let bit = self.shift_register & 1;
        self.shift_register >>= 1;
        // After all 8 bits are read, subsequent reads return 1
        self.shift_register |= 0x80;
        bit
    }

    pub fn update(&mut self, state: u8) {
        self.button_state = state;
    }
}
