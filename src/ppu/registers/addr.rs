// PPU Address Register ($2006)
// 7  bit  0
// ---- ----
// HHHH HHHH (first write)
// LLLL LLLL (second write)
//
// Address is 14-bit (0x0000 - 0x3FFF)
// Two writes are required to set the full address:
// 1st write: high byte (MSB)
// 2nd write: low byte (LSB)
//
// The write latch is reset when Status Register ($2002) is read

#[derive(Debug)]
pub struct AddrReg {
    hi_byte: u8,
    low_byte: u8,
    is_latch: bool,
}

impl AddrReg {
    pub fn new() -> Self {
        AddrReg {
            hi_byte: 0,
            low_byte: 0,
            is_latch: false,
        }
    }

    pub fn write(&mut self, value: u8) {
        if !self.is_latch {
            self.hi_byte = value & 0x3F;
            self.is_latch = true;
        } else {
            self.low_byte = value;
            self.is_latch = false;
        }
    }

    pub fn read(&self) -> u16 {
        ((self.hi_byte as u16) << 8) | (self.low_byte as u16)
    }

    pub fn increment(&mut self, value: u8) {
        let addr = self.read().wrapping_add(value as u16) & 0x3FFF;
        self.hi_byte = (addr >> 8) as u8;
        self.low_byte = (addr & 0xFF) as u8;
    }

    pub fn reset_latch(&mut self) {
        self.is_latch = false;
    }
}
