// PPU Status Register ($2002)
// 7  bit  0
// ---- ----
// VSO. ....
// |||| ||||
// |||+-++++- PPU open bus (unused bits, return last PPU bus write value)
// ||+------- Sprite overflow (0: no overflow; 1: more than 8 sprites on scanline)
// |+-------- Sprite 0 Hit (0: no hit; 1: sprite 0 overlaps background)
// +--------- VBlank flag (0: not in VBlank; 1: in VBlank)

#[derive(Debug)]
pub struct StatusReg {
    value: u8,
}

impl StatusReg {
    pub fn new() -> Self {
        StatusReg { value: 0 }
    }

    pub fn vblank(&self) -> bool {
        self.value & 0b1000_0000 != 0
    }

    pub fn sprite_zero_hit(&self) -> bool {
        self.value & 0b0100_0000 != 0
    }

    pub fn sprite_overflow(&self) -> bool {
        self.value & 0b0010_0000 != 0
    }

    pub fn set_vblank(&mut self, enable: bool) {
        if enable {
            self.value |= 0b1000_0000;
        } else {
            self.value &= !0b1000_0000;
        }
    }

    pub fn set_sprite_zero_hit(&mut self, enable: bool) {
        if enable {
            self.value |= 0b0100_0000;
        } else {
            self.value &= !0b0100_0000;
        }
    }

    pub fn set_sprite_overflow(&mut self, enable: bool) {
        if enable {
            self.value |= 0b0010_0000;
        } else {
            self.value &= !0b0010_0000;
        }
    }

    pub fn read(&mut self) -> u8 {
        let value = self.value;
        self.set_vblank(false);
        value
    }
}
