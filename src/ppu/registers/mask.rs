// PPU Mask Register ($2001)
// 7  bit  0
// ---- ----
// BGRs bMmG
// |||| ||||
// |||| |||+- Greyscale (0: normal color, 1: greyscale)
// |||| ||+-- Show background in leftmost 8 pixels (0: hide, 1: show)
// |||| |+--- Show sprites in leftmost 8 pixels (0: hide, 1: show)
// |||| +---- Show background (0: hide, 1: show)
// |||+------ Show sprites (0: hide, 1: show)
// ||+------- Emphasize red (green on PAL/Dendy)
// |+-------- Emphasize green (red on PAL/Dendy)
// +--------- Emphasize blue

#[derive(Debug)]
pub struct MaskReg {
    value: u8,
}

impl MaskReg {
    pub fn new() -> Self {
        MaskReg { value: 0 }
    }

    pub fn greyscale(&self) -> bool {
        self.value & 0b0000_0001 != 0
    }

    pub fn show_bg_leftmost(&self) -> bool {
        self.value & 0b0000_0010 != 0
    }

    pub fn show_sprites_leftmost(&self) -> bool {
        self.value & 0b0000_0100 != 0
    }

    pub fn show_background(&self) -> bool {
        self.value & 0b0000_1000 != 0
    }

    pub fn show_sprites(&self) -> bool {
        self.value & 0b0001_0000 != 0
    }

    pub fn emphasize_red(&self) -> bool {
        self.value & 0b0010_0000 != 0
    }

    pub fn emphasize_green(&self) -> bool {
        self.value & 0b0100_0000 != 0
    }

    pub fn emphasize_blue(&self) -> bool {
        self.value & 0b1000_0000 != 0
    }

    pub fn rendering_enabled(&self) -> bool {
        self.show_background() || self.show_sprites()
    }

    pub fn update(&mut self, value: u8) {
        self.value = value;
    }
}
