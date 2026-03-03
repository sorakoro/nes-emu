use crate::apu::Apu;
use crate::cart::Cartridge;
use crate::controller::Controller;
use crate::ppu::Ppu;

pub struct Bus {
    pub ram: [u8; 2048],
    pub ppu: Ppu,
    pub apu: Apu,
    pub cartridge: Cartridge,
    pub controller: Controller,
}

impl Bus {
    pub fn new(cartridge: Cartridge) -> Self {
        Bus {
            ram: [0; 2048],
            ppu: Ppu::new(),
            apu: Apu::new(),
            cartridge,
            controller: Controller::new(),
        }
    }

    pub fn read(&mut self, addr: u16) -> u8 {
        match addr {
            0x0000..=0x1FFF => {
                let mirror_addr = addr & 0x07FF;
                self.ram[mirror_addr as usize]
            }
            0x2000..=0x3FFF => {
                let mirror_addr = 0x2000 + (addr & 0x0007);
                self.ppu.read_register(mirror_addr, &self.cartridge)
            }
            0x4014 => panic!("Attempted to read from write-only register OAMDMA ($4014)"),
            0x4015 => self.apu.read_status(),
            0x4016 => self.controller.read(),
            0x4017 => 0, // Player 2 not implemented
            0x8000..=0xFFFF => self.cartridge.read_prg_rom(addr),
            _ => 0,
        }
    }

    pub fn write(&mut self, addr: u16, value: u8) {
        match addr {
            0x0000..=0x1FFF => {
                let mirror_addr = addr & 0x07FF;
                self.ram[mirror_addr as usize] = value;
            }
            0x2000..=0x3FFF => {
                let mirror_addr = 0x2000 + (addr & 0x0007);
                self.ppu
                    .write_register(mirror_addr, value, &mut self.cartridge);
            }
            0x4000..=0x400F | 0x4015 => self.apu.write_register(addr, value),
            0x4016 => self.controller.write(value),
            0x4017 => self.apu.write_register(addr, value),
            0x8000..=0xFFFF => {
                panic!("Attempted to write to ROM at ${:04X}", addr)
            }
            _ => {}
        }
    }

    pub fn peek(&self, addr: u16) -> u8 {
        match addr {
            0x0000..=0x1FFF => self.ram[(addr & 0x07FF) as usize],
            0x8000..=0xFFFF => self.cartridge.read_prg_rom(addr),
            _ => 0,
        }
    }

    pub fn tick_ppu(&mut self, cycles: u16) {
        for _ in 0..cycles {
            self.ppu.tick(&self.cartridge);
        }
    }

    pub fn tick_apu(&mut self, cpu_cycles: u16) {
        self.apu.tick(cpu_cycles);
    }
}
