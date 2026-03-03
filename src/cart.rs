const INES_HEADER_SIZE: usize = 0x0010;
const NES_TAG: [u8; 4] = [0x4E, 0x45, 0x53, 0x1A];
const PRG_ROM_PAGE_SIZE: usize = 0x4000;
const CHR_ROM_PAGE_SIZE: usize = 0x2000;

#[derive(Debug)]
enum Mapper {
    Nrom,
    Mmc3 {
        bank_select: u8,
        bank_data: [u8; 8],
        prg_ram: Vec<u8>,
        irq_counter: u8,
        irq_reload: u8,
        irq_enabled: bool,
        irq_pending: bool,
        irq_reload_flag: bool,
    },
}

#[derive(Debug)]
pub struct Cartridge {
    is_vertical_mirroring: bool,
    mapper: Mapper,
    has_chr_ram: bool,
    prg_rom: Vec<u8>,
    chr_rom: Vec<u8>,
}

impl Cartridge {
    pub fn new(raw: &[u8]) -> Result<Cartridge, String> {
        if raw[0..4] != NES_TAG {
            return Err("File is not in iNES file format.".to_string());
        }

        let prg_rom_pages = raw[4] as usize;
        let chr_rom_pages = raw[5] as usize;

        let is_vertical_mirroring = raw[6] & 0b1 != 0;
        let skip_trainer = raw[6] & 0b100 != 0;
        let mapper_num = (raw[7] & 0b1111_0000) | (raw[6] >> 4);

        let prg_rom_start = INES_HEADER_SIZE + if skip_trainer { 512 } else { 0 };
        let prg_rom_size = PRG_ROM_PAGE_SIZE * prg_rom_pages;

        let chr_rom_start = prg_rom_start + prg_rom_size;
        let chr_rom_size = CHR_ROM_PAGE_SIZE * chr_rom_pages;

        let mapper = match mapper_num {
            0 => Mapper::Nrom,
            4 => Mapper::Mmc3 {
                bank_select: 0,
                bank_data: [0; 8],
                prg_ram: vec![0; 0x2000],
                irq_counter: 0,
                irq_reload: 0,
                irq_enabled: false,
                irq_pending: false,
                irq_reload_flag: false,
            },
            _ => return Err(format!("Unsupported mapper: {}", mapper_num)),
        };

        Ok(Cartridge {
            is_vertical_mirroring,
            mapper,
            has_chr_ram: chr_rom_pages == 0,
            prg_rom: raw[prg_rom_start..prg_rom_start + prg_rom_size].to_vec(),
            chr_rom: if chr_rom_pages > 0 {
                raw[chr_rom_start..chr_rom_start + chr_rom_size].to_vec()
            } else {
                vec![0; CHR_ROM_PAGE_SIZE]
            },
        })
    }

    pub fn read_prg_rom(&self, addr: u16) -> u8 {
        match &self.mapper {
            Mapper::Nrom => {
                let offset = (addr - 0x8000) as usize % self.prg_rom.len();
                self.prg_rom[offset]
            }
            Mapper::Mmc3 {
                bank_select,
                bank_data,
                ..
            } => {
                let prg_banks = self.prg_rom.len() / 0x2000; // number of 8KB banks
                let prg_mode = (bank_select >> 6) & 1;
                let bank = match addr {
                    0x8000..=0x9FFF => {
                        if prg_mode == 0 {
                            bank_data[6] as usize % prg_banks
                        } else {
                            prg_banks - 2 // fixed second-to-last
                        }
                    }
                    0xA000..=0xBFFF => bank_data[7] as usize % prg_banks,
                    0xC000..=0xDFFF => {
                        if prg_mode == 0 {
                            prg_banks - 2 // fixed second-to-last
                        } else {
                            bank_data[6] as usize % prg_banks
                        }
                    }
                    0xE000..=0xFFFF => prg_banks - 1, // fixed last
                    _ => unreachable!(),
                };
                let offset = bank * 0x2000 + (addr as usize & 0x1FFF);
                self.prg_rom[offset]
            }
        }
    }

    pub fn read_chr_rom(&self, addr: u16) -> u8 {
        match &self.mapper {
            Mapper::Nrom => self.chr_rom[addr as usize],
            Mapper::Mmc3 {
                bank_select,
                bank_data,
                ..
            } => {
                let chr_banks = self.chr_rom.len() / 0x0400; // number of 1KB banks
                let chr_mode = (bank_select >> 7) & 1;
                let slot = (addr as usize) / 0x0400; // 1KB slot 0-7
                let bank = if chr_mode == 0 {
                    match slot {
                        0 => bank_data[0] as usize & 0xFE,
                        1 => (bank_data[0] as usize & 0xFE) + 1,
                        2 => bank_data[1] as usize & 0xFE,
                        3 => (bank_data[1] as usize & 0xFE) + 1,
                        4 => bank_data[2] as usize,
                        5 => bank_data[3] as usize,
                        6 => bank_data[4] as usize,
                        7 => bank_data[5] as usize,
                        _ => unreachable!(),
                    }
                } else {
                    match slot {
                        0 => bank_data[2] as usize,
                        1 => bank_data[3] as usize,
                        2 => bank_data[4] as usize,
                        3 => bank_data[5] as usize,
                        4 => bank_data[0] as usize & 0xFE,
                        5 => (bank_data[0] as usize & 0xFE) + 1,
                        6 => bank_data[1] as usize & 0xFE,
                        7 => (bank_data[1] as usize & 0xFE) + 1,
                        _ => unreachable!(),
                    }
                } % chr_banks;
                let offset = addr as usize & 0x03FF;
                self.chr_rom[bank * 0x0400 + offset]
            }
        }
    }

    pub fn write_chr_ram(&mut self, addr: u16, value: u8) {
        if self.has_chr_ram {
            self.chr_rom[addr as usize] = value;
        }
    }

    pub fn write_mapper(&mut self, addr: u16, value: u8) {
        match &mut self.mapper {
            Mapper::Nrom => {} // writes to ROM ignored
            Mapper::Mmc3 {
                bank_select,
                bank_data,
                irq_reload,
                irq_reload_flag,
                irq_enabled,
                irq_pending,
                ..
            } => {
                let even = addr & 1 == 0;
                match addr {
                    0x8000..=0x9FFF => {
                        if even {
                            *bank_select = value;
                        } else {
                            let reg = (*bank_select & 0x07) as usize;
                            bank_data[reg] = value;
                        }
                    }
                    0xA000..=0xBFFF => {
                        if even {
                            self.is_vertical_mirroring = value & 1 == 0;
                        }
                        // odd: PRG RAM protect — ignored
                    }
                    0xC000..=0xDFFF => {
                        if even {
                            *irq_reload = value;
                        } else {
                            *irq_reload_flag = true;
                        }
                    }
                    0xE000..=0xFFFF => {
                        if even {
                            *irq_enabled = false;
                            *irq_pending = false;
                        } else {
                            *irq_enabled = true;
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    pub fn read_prg_ram(&self, addr: u16) -> u8 {
        match &self.mapper {
            Mapper::Mmc3 { prg_ram, .. } => prg_ram[(addr - 0x6000) as usize],
            _ => 0,
        }
    }

    pub fn write_prg_ram(&mut self, addr: u16, value: u8) {
        match &mut self.mapper {
            Mapper::Mmc3 { prg_ram, .. } => prg_ram[(addr - 0x6000) as usize] = value,
            _ => {}
        }
    }

    pub fn clock_irq_counter(&mut self) {
        match &mut self.mapper {
            Mapper::Mmc3 {
                irq_counter,
                irq_reload,
                irq_reload_flag,
                irq_enabled,
                irq_pending,
                ..
            } => {
                if *irq_reload_flag || *irq_counter == 0 {
                    *irq_counter = *irq_reload;
                    *irq_reload_flag = false;
                } else {
                    *irq_counter -= 1;
                }
                if *irq_counter == 0 && *irq_enabled {
                    *irq_pending = true;
                }
            }
            _ => {}
        }
    }

    pub fn irq_pending(&self) -> bool {
        match &self.mapper {
            Mapper::Mmc3 { irq_pending, .. } => *irq_pending,
            _ => false,
        }
    }

    pub fn is_vertical_mirroring(&self) -> bool {
        self.is_vertical_mirroring
    }

    #[cfg(test)]
    pub fn new_test(prg_rom: Vec<u8>) -> Self {
        Cartridge {
            is_vertical_mirroring: false,
            mapper: Mapper::Nrom,
            has_chr_ram: true,
            prg_rom,
            chr_rom: vec![0; 0x2000],
        }
    }
}
