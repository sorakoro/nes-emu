mod registers;

use crate::cart::Cartridge;
use registers::{CtrlReg, MaskReg, StatusReg};

pub const SCREEN_WIDTH: usize = 256;
pub const SCREEN_HEIGHT: usize = 240;

#[rustfmt::skip]
const NES_PALETTE: [[u8; 3]; 64] = [
    [84, 84, 84],    [0, 30, 116],    [8, 16, 144],    [48, 0, 136],
    [68, 0, 100],    [92, 0, 48],     [84, 4, 0],      [60, 24, 0],
    [32, 42, 0],     [8, 58, 0],      [0, 64, 0],      [0, 60, 0],
    [0, 50, 60],     [0, 0, 0],       [0, 0, 0],       [0, 0, 0],
    [152, 150, 152], [8, 76, 196],    [48, 50, 236],   [92, 30, 228],
    [136, 20, 176],  [160, 20, 100],  [152, 34, 32],   [120, 60, 0],
    [84, 90, 0],     [40, 114, 0],    [8, 124, 0],     [0, 118, 40],
    [0, 102, 120],   [0, 0, 0],       [0, 0, 0],       [0, 0, 0],
    [236, 238, 236], [76, 154, 236],  [120, 124, 236], [176, 98, 236],
    [228, 84, 236],  [236, 88, 180],  [236, 106, 100], [212, 136, 32],
    [160, 170, 0],   [116, 196, 0],   [76, 208, 32],   [56, 204, 108],
    [56, 180, 204],  [60, 60, 60],    [0, 0, 0],       [0, 0, 0],
    [236, 238, 236], [168, 204, 236], [188, 188, 236], [212, 178, 236],
    [236, 174, 236], [236, 174, 212], [236, 180, 176], [228, 196, 144],
    [204, 210, 120], [180, 222, 120], [168, 226, 144], [152, 226, 180],
    [160, 214, 228], [160, 162, 160], [0, 0, 0],       [0, 0, 0],
];

pub struct Ppu {
    ctrl: CtrlReg,
    mask: MaskReg,
    status: StatusReg,
    oam_addr: u8,
    v: u16,      // 15-bit current VRAM address
    t: u16,      // 15-bit temporary VRAM address
    fine_x: u8,  // 3-bit fine X scroll
    w: bool,     // write latch (shared $2005/$2006)

    vram: [u8; 2048],
    palette_ram: [u8; 32],
    oam: [u8; 256],

    internal_data_buffer: u8,
    scanline: u16,
    cycles: u16,
    nmi_occurred: bool,
    pub frame_ready: bool,
    pub frame_buffer: [u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3],
    bg_opaque: [bool; SCREEN_WIDTH * SCREEN_HEIGHT],
}

impl Ppu {
    pub fn new() -> Self {
        Ppu {
            ctrl: CtrlReg::new(),
            mask: MaskReg::new(),
            status: StatusReg::new(),
            oam_addr: 0,
            v: 0,
            t: 0,
            fine_x: 0,
            w: false,

            vram: [0; 2048],
            palette_ram: [0; 32],
            oam: [0; 256],

            internal_data_buffer: 0,
            scanline: 0,
            cycles: 0,
            nmi_occurred: false,
            frame_ready: false,
            frame_buffer: [0; SCREEN_WIDTH * SCREEN_HEIGHT * 3],
            bg_opaque: [false; SCREEN_WIDTH * SCREEN_HEIGHT],
        }
    }

    pub fn read_register(&mut self, addr: u16, cart: &Cartridge) -> u8 {
        match addr {
            0x2000 => panic!("Attempted to read from write-only register PPUCTRL ($2000)"),
            0x2001 => panic!("Attempted to read from write-only register PPUMASK ($2001)"),
            0x2002 => {
                let value = self.status.read();
                self.w = false;
                value
            }
            0x2003 => panic!("Attempted to read from write-only register OAMADDR ($2003)"),
            0x2004 => self.oam[self.oam_addr as usize],
            0x2005 => panic!("Attempted to read from write-only register PPUSCROLL ($2005)"),
            0x2006 => panic!("Attempted to read from write-only register PPUADDR ($2006)"),
            0x2007 => self.read_data(cart),
            _ => panic!("Invalid PPU register address: ${:04X}", addr),
        }
    }

    fn read_data(&mut self, cart: &Cartridge) -> u8 {
        let addr = self.v;
        self.v = self.v.wrapping_add(self.ctrl.vram_increment() as u16) & 0x3FFF;

        match addr {
            0x0000..=0x1FFF => {
                let buffer = self.internal_data_buffer;
                self.internal_data_buffer = cart.read_chr_rom(addr);
                buffer
            }
            0x2000..=0x3EFF => {
                let buffer = self.internal_data_buffer;
                let vram_addr = self.mirror_vram_addr(addr, cart);
                self.internal_data_buffer = self.vram[vram_addr as usize];
                buffer
            }
            0x3F00..=0x3FFF => {
                let palette_addr = (addr - 0x3F00) & 0x1F;
                self.palette_ram[palette_addr as usize]
            }
            _ => unreachable!(),
        }
    }

    fn mirror_vram_addr(&self, addr: u16, cart: &Cartridge) -> u16 {
        let mirrored = addr & 0x2FFF;
        let nametable_index = (mirrored - 0x2000) / 0x400;
        let offset = (mirrored - 0x2000) % 0x400;

        let is_vertical = cart.is_vertical_mirroring();

        match (nametable_index, is_vertical) {
            (0, _) => offset,
            (1, true) => 0x400 + offset,
            (1, false) => offset,
            (2, true) => offset,
            (2, false) => 0x400 + offset,
            (3, _) => 0x400 + offset,
            _ => unreachable!(),
        }
    }

    pub fn write_register(&mut self, addr: u16, value: u8, cart: &mut Cartridge) {
        match addr {
            0x2000 => {
                self.ctrl.update(value);
                // t: ...GH.. ........ = d: ......GH
                self.t = (self.t & 0xF3FF) | (((value as u16) & 0x03) << 10);
            }
            0x2001 => self.mask.update(value),
            0x2002 => panic!("Attempted to write to read-only register PPUSTATUS ($2002)"),
            0x2003 => self.oam_addr = value,
            0x2004 => {
                self.oam[self.oam_addr as usize] = value;
                self.oam_addr = self.oam_addr.wrapping_add(1);
            }
            0x2005 => {
                if !self.w {
                    // 1st write: coarse X + fine X
                    // t: ....... ...ABCDE = d: ABCDE...
                    self.t = (self.t & 0xFFE0) | ((value as u16) >> 3);
                    self.fine_x = value & 0x07;
                    self.w = true;
                } else {
                    // 2nd write: fine Y + coarse Y
                    // t: FGH..AB CDE..... = d: ABCDEFGH
                    self.t = (self.t & 0x8FFF) | (((value as u16) & 0x07) << 12);
                    self.t = (self.t & 0xFC1F) | (((value as u16) & 0xF8) << 2);
                    self.w = false;
                }
            }
            0x2006 => {
                if !self.w {
                    // 1st write: high byte
                    // t: .CDEFGH ........ = d: ..CDEFGH  (bit 14 cleared)
                    self.t = (self.t & 0x00FF) | (((value as u16) & 0x3F) << 8);
                    self.w = true;
                } else {
                    // 2nd write: low byte, then v = t
                    // t: ....... ABCDEFGH = d: ABCDEFGH
                    self.t = (self.t & 0xFF00) | (value as u16);
                    self.v = self.t;
                    self.w = false;
                }
            }
            0x2007 => self.write_data(value, cart),
            _ => panic!("Invalid PPU register address: ${:04X}", addr),
        }
    }

    fn write_data(&mut self, value: u8, cart: &mut Cartridge) {
        let addr = self.v;
        self.v = self.v.wrapping_add(self.ctrl.vram_increment() as u16) & 0x3FFF;

        match addr {
            0x0000..=0x1FFF => {
                cart.write_chr_ram(addr, value);
            }
            0x2000..=0x3EFF => {
                let vram_addr = self.mirror_vram_addr(addr, cart);
                self.vram[vram_addr as usize] = value;
            }
            0x3F00..=0x3FFF => {
                let palette_addr = (addr - 0x3F00) & 0x1F;
                self.palette_ram[palette_addr as usize] = value;
            }
            _ => unreachable!(),
        }
    }

    pub fn write_oam_dma(&mut self, value: &[u8; 256]) {
        self.oam.copy_from_slice(value);
    }

    fn render_background(&mut self, cart: &Cartridge) {
        self.bg_opaque = [false; SCREEN_WIDTH * SCREEN_HEIGHT];
        let pattern_base = self.ctrl.bg_pattern_addr();

        // Extract scroll position from t register
        let coarse_x = self.t & 0x001F;
        let coarse_y = (self.t >> 5) & 0x001F;
        let base_nt = (self.t >> 10) & 0x0003;
        let fine_y = (self.t >> 12) & 0x0007;

        let tile_cols = if self.fine_x > 0 { 33u16 } else { 32u16 };
        let tile_rows = if fine_y > 0 { 31u16 } else { 30u16 };

        for ty in 0..tile_rows {
            for tx in 0..tile_cols {
                let mut nt = base_nt;
                let mut cur_x = coarse_x + tx;
                let mut cur_y = coarse_y + ty;

                // X wrapping: at tile 32, flip horizontal nametable
                if cur_x >= 32 {
                    cur_x -= 32;
                    nt ^= 1;
                }

                // Y wrapping: at row 30, flip vertical nametable
                while cur_y >= 30 {
                    cur_y -= 30;
                    nt ^= 2;
                }

                let nametable_addr = 0x2000 + nt * 0x400 + cur_y * 32 + cur_x;
                let vram_offset = self.mirror_vram_addr(nametable_addr, cart);
                let tile_index = self.vram[vram_offset as usize] as u16;

                // attribute table
                let attr_addr = 0x2000 + nt * 0x400 + 0x3C0 + (cur_y / 4) * 8 + (cur_x / 4);
                let attr_offset = self.mirror_vram_addr(attr_addr, cart);
                let attr_byte = self.vram[attr_offset as usize];
                let shift = ((cur_y % 4) / 2 * 2 + (cur_x % 4) / 2) * 2;
                let palette_index = ((attr_byte >> shift) & 0x03) as u16;

                let tile_addr = pattern_base + tile_index * 16;

                for row in 0..8u16 {
                    let screen_y = ty as i16 * 8 - fine_y as i16 + row as i16;
                    if screen_y < 0 || screen_y >= 240 {
                        continue;
                    }
                    let py = screen_y as usize;

                    let lo = cart.read_chr_rom(tile_addr + row);
                    let hi = cart.read_chr_rom(tile_addr + row + 8);

                    for col in 0..8u16 {
                        let screen_x = tx as i16 * 8 - self.fine_x as i16 + col as i16;
                        if screen_x < 0 || screen_x >= 256 {
                            continue;
                        }
                        let px = screen_x as usize;

                        let bit = 7 - col;
                        let color_index = ((hi >> bit) & 1) << 1 | ((lo >> bit) & 1);

                        let palette_addr = if color_index == 0 {
                            0
                        } else {
                            self.bg_opaque[py * SCREEN_WIDTH + px] = true;
                            (palette_index * 4 + color_index as u16) as usize
                        };
                        let nes_color = self.palette_ram[palette_addr] as usize;
                        let rgb = NES_PALETTE[nes_color & 0x3F];

                        let offset = (py * SCREEN_WIDTH + px) * 3;
                        self.frame_buffer[offset] = rgb[0];
                        self.frame_buffer[offset + 1] = rgb[1];
                        self.frame_buffer[offset + 2] = rgb[2];
                    }
                }
            }
        }
    }

    fn render_sprites(&mut self, cart: &Cartridge) {
        if !self.mask.show_sprites() {
            return;
        }

        let sprite_height = self.ctrl.sprite_size() as u16;

        // Sprite overflow detection: count sprites per scanline
        for scanline in 0..240u16 {
            let mut count = 0u8;
            for i in 0..64 {
                let y = self.oam[i * 4] as u16 + 1;
                if scanline >= y && scanline < y + sprite_height {
                    count += 1;
                    if count > 8 {
                        self.status.set_sprite_overflow(true);
                        break;
                    }
                }
            }
            if count > 8 {
                break;
            }
        }

        // Draw sprites in reverse order (63→0) so lower index sprites appear on top
        for i in (0..64).rev() {
            let oam_offset = i * 4;
            let sprite_y = self.oam[oam_offset] as u16 + 1;
            let tile_index = self.oam[oam_offset + 1] as u16;
            let attributes = self.oam[oam_offset + 2];
            let sprite_x = self.oam[oam_offset + 3] as usize;

            let palette_index = (attributes & 0x03) as u16;
            let behind_bg = attributes & 0x20 != 0;
            let flip_h = attributes & 0x40 != 0;
            let flip_v = attributes & 0x80 != 0;

            for row in 0..sprite_height {
                let py = sprite_y + row;
                if py >= 240 {
                    continue;
                }

                let actual_row = if flip_v { sprite_height - 1 - row } else { row };

                let (pattern_base, tile_num) = if sprite_height == 16 {
                    // 8x16 mode: bit 0 of tile index selects pattern table
                    let bank = (tile_index & 1) * 0x1000;
                    let base_tile = tile_index & 0xFE;
                    if actual_row < 8 {
                        (bank, base_tile)
                    } else {
                        (bank, base_tile + 1)
                    }
                } else {
                    // 8x8 mode
                    (self.ctrl.sprite_pattern_addr(), tile_index)
                };

                let tile_row = actual_row % 8;
                let tile_addr = pattern_base + tile_num * 16 + tile_row;
                let lo = cart.read_chr_rom(tile_addr);
                let hi = cart.read_chr_rom(tile_addr + 8);

                for col in 0..8u16 {
                    let px = sprite_x + col as usize;
                    if px >= 256 {
                        continue;
                    }

                    // Left 8px clipping
                    if px < 8 && !self.mask.show_sprites_leftmost() {
                        continue;
                    }

                    let bit = if flip_h { col } else { 7 - col };
                    let color_index = ((hi >> bit) & 1) << 1 | ((lo >> bit) & 1);

                    // Transparent pixel
                    if color_index == 0 {
                        continue;
                    }

                    let pixel_idx = py as usize * SCREEN_WIDTH + px;

                    // Sprite 0 Hit: sprite 0 opaque pixel overlaps opaque BG pixel
                    // Does not trigger at x=255
                    if i == 0 && self.bg_opaque[pixel_idx] && px != 255 {
                        self.status.set_sprite_zero_hit(true);
                    }

                    // Behind-BG priority: don't draw sprite if BG is opaque
                    if behind_bg && self.bg_opaque[pixel_idx] {
                        continue;
                    }

                    let palette_addr = (0x10 + palette_index * 4 + color_index as u16) as usize;
                    let nes_color = self.palette_ram[palette_addr] as usize;
                    let rgb = NES_PALETTE[nes_color & 0x3F];

                    let fb_offset = pixel_idx * 3;
                    self.frame_buffer[fb_offset] = rgb[0];
                    self.frame_buffer[fb_offset + 1] = rgb[1];
                    self.frame_buffer[fb_offset + 2] = rgb[2];
                }
            }
        }
    }

    pub fn tick(&mut self, cart: &Cartridge) {
        self.cycles += 1;

        if self.cycles == 341 {
            self.cycles = 0;
            self.scanline += 1;

            if self.scanline == 262 {
                self.scanline = 0;
            }
        }

        if self.scanline == 241 && self.cycles == 1 {
            self.render_background(cart);
            self.render_sprites(cart);
            self.frame_ready = true;
            self.status.set_vblank(true);
            if self.ctrl.generate_nmi() {
                self.nmi_occurred = true;
            }
        }

        if self.scanline == 261 && self.cycles == 1 {
            self.status.set_vblank(false);
            self.status.set_sprite_zero_hit(false);
            self.status.set_sprite_overflow(false);
        }
    }

    pub fn poll_nmi(&mut self) -> bool {
        if self.nmi_occurred {
            self.nmi_occurred = false;
            true
        } else {
            false
        }
    }
}
