// PPU Control Register ($2000)
// 7  bit  0
// ---- ----
// VPHB SINN
// |||| ||||
// |||| ||++- Base nametable address (0 = $2000; 1 = $2400; 2 = $2800; 3 = $2C00)
// |||| |+--- VRAM address increment (0 = add 1, horizontal; 1 = add 32, vertical)
// |||| +---- Sprite pattern table address (0 = $0000; 1 = $1000; ignored in 8x16 mode)
// |||+------ Background pattern table address (0 = $0000; 1 = $1000)
// ||+------- Sprite size (0 = 8x8 pixels; 1 = 8x16 pixels)
// |+-------- PPU master/slave select (not commonly used)
// +--------- NMI enable (0 = disabled; 1 = generate NMI on VBlank)

#[derive(Debug)]
pub struct CtrlReg {
    value: u8,
}

impl CtrlReg {
    pub fn new() -> Self {
        CtrlReg { value: 0 }
    }

    pub fn nametable_addr(&self) -> u16 {
        0x2000 + ((self.value & 0b11) as u16) * 0x400
    }

    pub fn vram_increment(&self) -> u8 {
        if self.value & 0b100 != 0 { 32 } else { 1 }
    }

    pub fn sprite_pattern_addr(&self) -> u16 {
        if self.value & 0b1000 != 0 {
            0x1000
        } else {
            0x0000
        }
    }

    pub fn bg_pattern_addr(&self) -> u16 {
        if self.value & 0b1_0000 != 0 {
            0x1000
        } else {
            0x000
        }
    }

    pub fn sprite_size(&self) -> u8 {
        if self.value & 0b10_0000 != 0 { 16 } else { 8 }
    }

    pub fn generate_nmi(&self) -> bool {
        self.value & 0b1000_0000 != 0
    }

    pub fn update(&mut self, value: u8) {
        self.value = value;
    }
}
