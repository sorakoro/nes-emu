mod opcodes;

use crate::cart::Cartridge;
use crate::ppu::PPU;
use opcodes::AddressingMode::{self, *};
use opcodes::OPCODES;

const CARRY: u8 = 0b0000_0001;
const ZERO: u8 = 0b0000_0010;
const INTERRUPT_DISABLE: u8 = 0b0000_0100;
const DECIMAL: u8 = 0b0000_1000;
const BREAK: u8 = 0b0001_0000;
const UNUSED: u8 = 0b0010_0000;
const OVERFLOW: u8 = 0b0100_0000;
const NEGATIVE: u8 = 0b1000_0000;

pub struct CPU<'a> {
    a: u8,
    x: u8,
    y: u8,
    pc: u16,
    sp: u8,
    status: u8,

    ram: [u8; 2048],

    ppu: &'a mut PPU<'a>,
    cartridge: &'a Cartridge,

    cycles: u64,
}

impl<'a> CPU<'a> {
    pub fn new(ppu: &'a mut PPU<'a>, cartridge: &'a Cartridge) -> Self {
        CPU {
            a: 0,
            x: 0,
            y: 0,
            pc: 0,
            sp: 0xFD,
            status: UNUSED | INTERRUPT_DISABLE,

            ram: [0; 2048],

            ppu,
            cartridge,

            cycles: 0,
        }
    }

    pub fn reset(&mut self) {
        self.a = 0;
        self.x = 0;
        self.y = 0;
        self.sp = 0xFD;
        self.status = UNUSED | INTERRUPT_DISABLE;

        let lo = self.read(0xFFFC) as u16;
        let hi = self.read(0xFFFD) as u16;
        self.pc = (hi << 8) | lo;

        self.cycles = 0;
    }

    fn read(&mut self, addr: u16) -> u8 {
        match addr {
            0x0000..=0x1FFF => {
                let mirror_addr = addr & 0x07FF;
                self.ram[mirror_addr as usize]
            }
            0x2000..=0x3FFF => {
                let mirror_addr = 0x2000 + (addr & 0x0007);
                self.ppu.read_register(mirror_addr)
            }
            0x4014 => {
                panic!("Attempted to read from write-only register OAMDMA ($4014)")
            }
            0x8000..=0xFFFF => self.cartridge.read_prg_rom(addr),
            _ => 0,
        }
    }

    fn write(&mut self, addr: u16, value: u8) {
        match addr {
            0x0000..=0x1FFF => {
                let mirror_addr = addr & 0x07FF;
                self.ram[mirror_addr as usize] = value;
            }
            0x2000..=0x3FFF => {
                let mirror_addr = 0x2000 + (addr & 0x0007);
                self.ppu.write_register(mirror_addr, value);
            }
            0x4014 => {
                let page = value as u16;
                let start_addr = page * 0x100;

                let mut data = [0u8; 256];
                for i in 0..256 {
                    data[i] = self.read(start_addr + i as u16);
                }

                self.ppu.write_oam_dma(&data);

                self.cycles += 513 + (self.cycles & 1);
            }
            0x8000..=0xFFFF => {
                panic!("Attempted to write to ROM at ${:04X}", addr)
            }
            _ => {}
        }
    }

    fn fetch(&mut self) -> u8 {
        let value = self.read(self.pc);
        self.pc = self.pc.wrapping_add(1);
        value
    }

    fn fetch_u16(&mut self) -> u16 {
        let lo = self.fetch() as u16;
        let hi = self.fetch() as u16;
        (hi << 8) | lo
    }

    fn addr_immediate(&mut self) -> u16 {
        let addr = self.pc;
        self.pc = self.pc.wrapping_add(1);
        addr
    }

    fn addr_zero_page(&mut self) -> u16 {
        self.fetch() as u16
    }

    fn addr_zero_page_x(&mut self) -> u16 {
        self.fetch().wrapping_add(self.x) as u16
    }

    fn addr_zero_page_y(&mut self) -> u16 {
        self.fetch().wrapping_add(self.y) as u16
    }

    fn addr_absolute(&mut self) -> u16 {
        self.fetch_u16()
    }

    fn addr_absolute_x(&mut self) -> (u16, bool) {
        let base = self.fetch_u16();
        let addr = base.wrapping_add(self.x as u16);
        let page_crossed = (base & 0xFF00) != (addr & 0xFF00);
        (addr, page_crossed)
    }

    fn addr_absolute_y(&mut self) -> (u16, bool) {
        let base = self.fetch_u16();
        let addr = base.wrapping_add(self.y as u16);
        let page_crossed = (base & 0xFF00) != (addr & 0xFF00);
        (addr, page_crossed)
    }

    fn addr_indirect(&mut self) -> u16 {
        let ptr = self.fetch_u16();
        let lo = self.read(ptr) as u16;
        let hi_addr = (ptr & 0xFF00) | ((ptr.wrapping_add(1)) & 0x00FF);
        let hi = self.read(hi_addr) as u16;
        (hi << 8) | lo
    }

    fn addr_indirect_x(&mut self) -> u16 {
        let ptr = self.fetch().wrapping_add(self.x);
        let lo = self.read(ptr as u16) as u16;
        let hi = self.read(ptr.wrapping_add(1) as u16) as u16;
        (hi << 8) | lo
    }

    fn addr_indirect_y(&mut self) -> (u16, bool) {
        let ptr = self.fetch();
        let lo = self.read(ptr as u16) as u16;
        let hi = self.read(ptr.wrapping_add(1) as u16) as u16;
        let base = (hi << 8) | lo;
        let addr = base.wrapping_add(self.y as u16);
        let page_crossed = (base & 0xFF00) != (addr & 0xFF00);
        (addr, page_crossed)
    }

    fn addr_relative(&mut self) -> u16 {
        let offset = self.fetch() as i8;
        self.pc.wrapping_add(offset as u16)
    }

    fn set_flag(&mut self, flag: u8, on: bool) {
        if on {
            self.status |= flag;
        } else {
            self.status &= !flag;
        }
    }

    fn update_zero_negative(&mut self, value: u8) {
        self.set_flag(ZERO, value == 0);
        self.set_flag(NEGATIVE, value & 0x80 != 0);
    }

    fn fetch_operand_addr(&mut self, mode: AddressingMode) -> (u16, bool) {
        match mode {
            Implicit | Accumulator => (0, false),
            Immediate => (self.addr_immediate(), false),
            ZeroPage => (self.addr_zero_page(), false),
            ZeroPageX => (self.addr_zero_page_x(), false),
            ZeroPageY => (self.addr_zero_page_y(), false),
            Absolute => (self.addr_absolute(), false),
            AbsoluteX => self.addr_absolute_x(),
            AbsoluteY => self.addr_absolute_y(),
            Indirect => (self.addr_indirect(), false),
            IndirectX => (self.addr_indirect_x(), false),
            IndirectY => self.addr_indirect_y(),
            Relative => (self.addr_relative(), false),
        }
    }

    pub fn step(&mut self) -> u8 {
        let opcode = self.fetch();

        let inst = match OPCODES[opcode as usize] {
            Some(inst) => inst,
            None => panic!(
                "Unknown opcode: 0x{:02X} at PC=0x{:04X}",
                opcode,
                self.pc.wrapping_sub(1)
            ),
        };

        let (addr, page_cross) = self.fetch_operand_addr(inst.mode);

        match inst.name {
            "LDA" => self.lda(addr),
            _ => panic!("Unimplemented instruction: {}", inst.name),
        }

        let extra = if inst.page_cross_cycle && page_cross {
            1
        } else {
            0
        };
        let cycles = inst.cycles + extra;
        self.cycles += cycles as u64;
        cycles
    }

    fn lda(&mut self, addr: u16) {
        self.a = self.read(addr);
        self.update_zero_negative(self.a);
    }

    pub fn nmi(&mut self) {
        // TODO: NMI割り込み処理
    }

    pub fn irq(&mut self) {
        // TODO: IRQ割り込み処理
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cart::Cartridge;
    use crate::ppu::PPU;

    fn test_cartridge(program: &[u8]) -> Cartridge {
        let mut prg_rom = vec![0u8; 0x8000];
        prg_rom[..program.len()].copy_from_slice(program);
        // reset vector → $8000
        prg_rom[0x7FFC] = 0x00;
        prg_rom[0x7FFD] = 0x80;
        Cartridge::new_test(prg_rom)
    }

    #[test]
    fn lda_immediate() {
        let cart = test_cartridge(&[0xA9, 0x42]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.a, 0x42);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn lda_immediate_zero_flag() {
        let cart = test_cartridge(&[0xA9, 0x00]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn lda_immediate_negative_flag() {
        let cart = test_cartridge(&[0xA9, 0x80]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.a, 0x80);
        assert_eq!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn lda_zero_page() {
        let cart = test_cartridge(&[0xA5, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x10] = 0x55;
        cpu.step();

        assert_eq!(cpu.a, 0x55);
    }

    #[test]
    fn lda_zero_page_x() {
        let cart = test_cartridge(&[0xB5, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.x = 0x05;
        cpu.ram[0x15] = 0x66;
        cpu.step();

        assert_eq!(cpu.a, 0x66);
    }

    #[test]
    fn lda_zero_page_x_wraps() {
        let cart = test_cartridge(&[0xB5, 0xFF]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.x = 0x01;
        cpu.ram[0x00] = 0x77;
        cpu.step();

        assert_eq!(cpu.a, 0x77);
    }

    #[test]
    fn lda_absolute() {
        let cart = test_cartridge(&[0xAD, 0x00, 0x02]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x0200] = 0x88;
        cpu.step();

        assert_eq!(cpu.a, 0x88);
    }

    #[test]
    fn lda_absolute_x() {
        let cart = test_cartridge(&[0xBD, 0x00, 0x02]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.x = 0x05;
        cpu.ram[0x0205] = 0x99;
        cpu.step();

        assert_eq!(cpu.a, 0x99);
    }

    #[test]
    fn lda_absolute_y() {
        let cart = test_cartridge(&[0xB9, 0x00, 0x02]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.y = 0x03;
        cpu.ram[0x0203] = 0xAA;
        cpu.step();

        assert_eq!(cpu.a, 0xAA);
    }

    #[test]
    fn lda_indirect_x() {
        let cart = test_cartridge(&[0xA1, 0x20]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.x = 0x04;
        // pointer at ZP $24-$25 → $0300
        cpu.ram[0x24] = 0x00;
        cpu.ram[0x25] = 0x03;
        cpu.ram[0x0300] = 0xBB;
        cpu.step();

        assert_eq!(cpu.a, 0xBB);
    }

    #[test]
    fn lda_indirect_y() {
        let cart = test_cartridge(&[0xB1, 0x40]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.y = 0x10;
        // pointer at ZP $40-$41 → $0300
        cpu.ram[0x40] = 0x00;
        cpu.ram[0x41] = 0x03;
        // $0300 + $10 = $0310
        cpu.ram[0x0310] = 0xCC;
        cpu.step();

        assert_eq!(cpu.a, 0xCC);
    }
}
