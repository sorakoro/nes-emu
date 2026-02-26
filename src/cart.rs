const INES_HEADER_SIZE: usize = 0x0010;
const NES_TAG: [u8; 4] = [0x4E, 0x45, 0x53, 0x1A];
const PRG_ROM_PAGE_SIZE: usize = 0x4000;
const CHR_ROM_PAGE_SIZE: usize = 0x2000;

#[derive(Debug)]
pub struct Cartridge {
    is_vertical_mirroring: bool,
    mapper: u8,
    prg_rom: Vec<u8>,
    chr_rom: Vec<u8>,
}

impl Cartridge {
    pub fn new(raw: &Vec<u8>) -> Result<Cartridge, String> {
        if &raw[0..4] != NES_TAG {
            return Err("File is not in iNES file format.".to_string());
        }

        let prg_rom_pages = raw[4] as usize;
        let chr_rom_pages = raw[5] as usize;

        let is_vertical_mirroring = raw[6] & 0b1 != 0;
        let skip_trainer = raw[6] & 0b100 != 0;
        let mapper = (raw[7] & 0b1111_0000) | (raw[6] >> 4);

        let prg_rom_start = INES_HEADER_SIZE + if skip_trainer { 512 } else { 0 };
        let prg_rom_size = PRG_ROM_PAGE_SIZE * prg_rom_pages;

        let chr_rom_start = prg_rom_start + prg_rom_size;
        let chr_rom_size = CHR_ROM_PAGE_SIZE * chr_rom_pages;

        Ok(Cartridge {
            is_vertical_mirroring,
            mapper,
            prg_rom: raw[prg_rom_start..prg_rom_size].to_vec(),
            chr_rom: raw[chr_rom_start..chr_rom_size].to_vec(),
        })
    }

    pub fn read_prg_rom(&self, addr: u16) -> u8 {
        let addr = (addr - 0x8000) as usize;
        self.prg_rom[addr]
    }

    pub fn read_chr_rom(&self, addr: u16) -> u8 {
        self.chr_rom[addr as usize]
    }

    pub fn is_vertical_mirroring(&self) -> bool {
        self.is_vertical_mirroring
    }

    #[cfg(test)]
    pub fn new_test(prg_rom: Vec<u8>) -> Self {
        Cartridge {
            is_vertical_mirroring: false,
            mapper: 0,
            prg_rom,
            chr_rom: vec![0; 0x2000],
        }
    }
}
