mod registers;

use crate::cart::Cartridge;
use registers::{AddrReg, CtrlReg, MaskReg, StatusReg};

pub struct PPU<'a> {
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

    cartridge: &'a Cartridge,
}

impl<'a> PPU<'a> {
    pub fn new(cartridge: &'a Cartridge) -> Self {
        PPU {
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

            cartridge,
        }
    }

    pub fn read_register(&mut self, addr: u16) -> u8 {
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
            0x2007 => self.read_data(),
            _ => panic!("Invalid PPU register address: ${:04X}", addr),
        }
    }

    fn read_data(&mut self) -> u8 {
        let addr = self.addr.read();
        self.addr.increment(self.ctrl.vram_increment());

        match addr {
            0x0000..=0x1FFF => {
                let buffer = self.internal_data_buffer;
                self.internal_data_buffer = self.cartridge.read_chr_rom(addr);
                buffer
            }
            0x2000..=0x3EFF => {
                let buffer = self.internal_data_buffer;
                let vram_addr = self.mirror_vram_addr(addr);
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

    fn mirror_vram_addr(&self, addr: u16) -> u16 {
        let mirrored = addr & 0x2FFF;
        let nametable_index = (mirrored - 0x2000) / 0x400;
        let offset = (mirrored - 0x2000) % 0x400;

        let is_vertical = self.cartridge.is_vertical_mirroring();

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

    pub fn write_register(&mut self, addr: u16, value: u8) {
        match addr {
            0x2000 => self.ctrl.update(value),
            0x2001 => self.mask.update(value),
            0x2002 => panic!("Attempted to write to read-only register PPUSTATUS ($2002)"),
            0x2003 => self.oam_addr = value,
            0x2004 => {
                self.oam[self.oam_addr as usize] = value;
                self.oam_addr = self.oam_addr.wrapping_add(1);
            }
            0x2005 => panic!("Attempted to write to unimplemented register PPUSCROLL ($2005)"),
            0x2006 => self.addr.write(value),
            0x2007 => self.write_data(value),
            _ => panic!("Invalid PPU register address: ${:04X}", addr),
        }
    }

    fn write_data(&mut self, value: u8) {
        let addr = self.addr.read();
        self.addr.increment(self.ctrl.vram_increment());

        match addr {
            0x0000..=0x1FFF => {
                panic!(
                    "Attempted to write to CHR-ROM at ${:04X} (read-only for Mapper 0)",
                    addr
                )
            }
            0x2000..=0x3EFF => {
                let vram_addr = self.mirror_vram_addr(addr);
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

    pub fn tick(&mut self) {
        self.cycles += 1;

        if self.cycles == 341 {
            self.cycles = 0;
            self.scanline += 1;

            if self.scanline == 262 {
                self.scanline = 0;
            }
        }

        if self.scanline == 241 && self.cycles == 1 {
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
