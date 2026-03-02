mod registers;

use crate::cart::Cartridge;
use registers::{AddrReg, CtrlReg, MaskReg, StatusReg};

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
    addr: AddrReg,

    vram: [u8; 2048],
    palette_ram: [u8; 32],
    oam: [u8; 256],

    internal_data_buffer: u8,
    scanline: u16,
    cycles: u16,
    nmi_occurred: bool,
    pub frame_ready: bool,
    pub frame_buffer: [u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3],
}

impl Ppu {
    pub fn new() -> Self {
        Ppu {
            ctrl: CtrlReg::new(),
            mask: MaskReg::new(),
            status: StatusReg::new(),
            oam_addr: 0,
            addr: AddrReg::new(),

            vram: [0; 2048],
            palette_ram: [0; 32],
            oam: [0; 256],

            internal_data_buffer: 0,
            scanline: 0,
            cycles: 0,
            nmi_occurred: false,
            frame_ready: false,
            frame_buffer: [0; SCREEN_WIDTH * SCREEN_HEIGHT * 3],
        }
    }

    pub fn read_register(&mut self, addr: u16, cart: &Cartridge) -> u8 {
        match addr {
            0x2000 => panic!("Attempted to read from write-only register PPUCTRL ($2000)"),
            0x2001 => panic!("Attempted to read from write-only register PPUMASK ($2001)"),
            0x2002 => {
                let value = self.status.read();
                self.addr.reset_latch();
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
        let addr = self.addr.read();
        self.addr.increment(self.ctrl.vram_increment());

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
            0x2000 => self.ctrl.update(value),
            0x2001 => self.mask.update(value),
            0x2002 => panic!("Attempted to write to read-only register PPUSTATUS ($2002)"),
            0x2003 => self.oam_addr = value,
            0x2004 => {
                self.oam[self.oam_addr as usize] = value;
                self.oam_addr = self.oam_addr.wrapping_add(1);
            }
            0x2005 => { /* TODO: PPUSCROLL */ }
            0x2006 => self.addr.write(value),
            0x2007 => self.write_data(value, cart),
            _ => panic!("Invalid PPU register address: ${:04X}", addr),
        }
    }

    fn write_data(&mut self, value: u8, cart: &mut Cartridge) {
        let addr = self.addr.read();
        self.addr.increment(self.ctrl.vram_increment());

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
        let nametable_base = self.ctrl.nametable_addr();
        let pattern_base = self.ctrl.bg_pattern_addr();

        for tile_y in 0..30u16 {
            for tile_x in 0..32u16 {
                let nametable_addr = nametable_base + tile_y * 32 + tile_x;
                let vram_offset = self.mirror_vram_addr(nametable_addr, cart);
                let tile_index = self.vram[vram_offset as usize] as u16;

                // attribute table
                let attr_addr = nametable_base + 0x3C0 + (tile_y / 4) * 8 + (tile_x / 4);
                let attr_offset = self.mirror_vram_addr(attr_addr, cart);
                let attr_byte = self.vram[attr_offset as usize];
                let shift = ((tile_y % 4) / 2 * 2 + (tile_x % 4) / 2) * 2;
                let palette_index = ((attr_byte >> shift) & 0x03) as u16;

                let tile_addr = pattern_base + tile_index * 16;

                for row in 0..8u16 {
                    let lo = cart.read_chr_rom(tile_addr + row);
                    let hi = cart.read_chr_rom(tile_addr + row + 8);

                    for col in 0..8u16 {
                        let bit = 7 - col;
                        let color_index = ((hi >> bit) & 1) << 1 | ((lo >> bit) & 1);

                        let palette_addr = if color_index == 0 {
                            0
                        } else {
                            (palette_index * 4 + color_index as u16) as usize
                        };
                        let nes_color = self.palette_ram[palette_addr] as usize;
                        let rgb = NES_PALETTE[nes_color & 0x3F];

                        let px = (tile_x * 8 + col) as usize;
                        let py = (tile_y * 8 + row) as usize;
                        let offset = (py * SCREEN_WIDTH + px) * 3;
                        self.frame_buffer[offset] = rgb[0];
                        self.frame_buffer[offset + 1] = rgb[1];
                        self.frame_buffer[offset + 2] = rgb[2];
                    }
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
