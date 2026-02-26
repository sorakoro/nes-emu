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
            "ADC" => self.adc(addr),
            "AND" => self.and(addr),
            "ASL" => {
                if inst.mode == Accumulator {
                    self.asl_accumulator();
                } else {
                    self.asl_memory(addr);
                }
            }
            "BCC" => {
                if self.status & CARRY == 0 {
                    self.branch(addr);
                }
            }
            "BCS" => {
                if self.status & CARRY != 0 {
                    self.branch(addr);
                }
            }
            "BEQ" => {
                if self.status & ZERO != 0 {
                    self.branch(addr);
                }
            }
            "BIT" => self.bit(addr),
            "BMI" => {
                if self.status & NEGATIVE != 0 {
                    self.branch(addr);
                }
            }
            "BNE" => {
                if self.status & ZERO == 0 {
                    self.branch(addr);
                }
            }
            "BPL" => {
                if self.status & NEGATIVE == 0 {
                    self.branch(addr);
                }
            }
            "BRK" => self.brk(),
            "BVC" => {
                if self.status & OVERFLOW == 0 {
                    self.branch(addr);
                }
            }
            "BVS" => {
                if self.status & OVERFLOW != 0 {
                    self.branch(addr);
                }
            }
            "CLC" => self.set_flag(CARRY, false),
            "CLD" => self.set_flag(DECIMAL, false),
            "CLI" => self.set_flag(INTERRUPT_DISABLE, false),
            "CLV" => self.set_flag(OVERFLOW, false),
            "CMP" => self.compare(self.a, addr),
            "CPX" => self.compare(self.x, addr),
            "CPY" => self.compare(self.y, addr),
            "DEC" => self.dec(addr),
            "DEX" => {
                self.x = self.x.wrapping_sub(1);
                self.update_zero_negative(self.x);
            }
            "DEY" => {
                self.y = self.y.wrapping_sub(1);
                self.update_zero_negative(self.y);
            }
            "EOR" => {
                self.a ^= self.read(addr);
                self.update_zero_negative(self.a);
            }
            "INC" => self.inc(addr),
            "INX" => {
                self.x = self.x.wrapping_add(1);
                self.update_zero_negative(self.x);
            }
            "INY" => {
                self.y = self.y.wrapping_add(1);
                self.update_zero_negative(self.y);
            }
            "JMP" => self.pc = addr,
            "JSR" => {
                self.push_u16(self.pc.wrapping_sub(1));
                self.pc = addr;
            }
            "LDA" => self.lda(addr),
            "LDX" => {
                self.x = self.read(addr);
                self.update_zero_negative(self.x);
            }
            "LDY" => {
                self.y = self.read(addr);
                self.update_zero_negative(self.y);
            }
            "LSR" => {
                if inst.mode == Accumulator {
                    self.set_flag(CARRY, self.a & 0x01 != 0);
                    self.a >>= 1;
                    self.update_zero_negative(self.a);
                } else {
                    let mut value = self.read(addr);
                    self.set_flag(CARRY, value & 0x01 != 0);
                    value >>= 1;
                    self.write(addr, value);
                    self.update_zero_negative(value);
                }
            }

            "NOP" => {}
            "ORA" => {
                self.a |= self.read(addr);
                self.update_zero_negative(self.a);
            }
            "PHA" => self.push(self.a),
            "PHP" => self.push(self.status | BREAK | UNUSED),
            "PLA" => {
                self.a = self.pull();
                self.update_zero_negative(self.a);
            }
            "PLP" => {
                self.status = (self.pull() & !BREAK) | UNUSED;
            }
            "SBC" => self.sbc(addr),
            "SEC" => self.set_flag(CARRY, true),
            "SED" => self.set_flag(DECIMAL, true),
            "RTS" => {
                let lo = self.pull() as u16;
                let hi = self.pull() as u16;
                self.pc = ((hi << 8) | lo).wrapping_add(1);
            }
            "RTI" => {
                self.status = (self.pull() & !BREAK) | UNUSED;
                let lo = self.pull() as u16;
                let hi = self.pull() as u16;
                self.pc = (hi << 8) | lo;
            }
            "ROR" => {
                if inst.mode == Accumulator {
                    let old_carry = if self.status & CARRY != 0 { 0x80 } else { 0 };
                    self.set_flag(CARRY, self.a & 0x01 != 0);
                    self.a = (self.a >> 1) | old_carry;
                    self.update_zero_negative(self.a);
                } else {
                    let mut value = self.read(addr);
                    let old_carry = if self.status & CARRY != 0 { 0x80 } else { 0 };
                    self.set_flag(CARRY, value & 0x01 != 0);
                    value = (value >> 1) | old_carry;
                    self.write(addr, value);
                    self.update_zero_negative(value);
                }
            }
            "ROL" => {
                if inst.mode == Accumulator {
                    let old_carry = self.status & CARRY;
                    self.set_flag(CARRY, self.a & 0x80 != 0);
                    self.a = (self.a << 1) | old_carry;
                    self.update_zero_negative(self.a);
                } else {
                    let mut value = self.read(addr);
                    let old_carry = self.status & CARRY;
                    self.set_flag(CARRY, value & 0x80 != 0);
                    value = (value << 1) | old_carry;
                    self.write(addr, value);
                    self.update_zero_negative(value);
                }
            }
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

    fn push(&mut self, value: u8) {
        self.write(0x0100 + self.sp as u16, value);
        self.sp = self.sp.wrapping_sub(1);
    }

    fn pull(&mut self) -> u8 {
        self.sp = self.sp.wrapping_add(1);
        self.read(0x0100 + self.sp as u16)
    }

    fn push_u16(&mut self, value: u16) {
        self.push((value >> 8) as u8);
        self.push(value as u8);
    }

    fn brk(&mut self) {
        self.push_u16(self.pc);
        self.push(self.status | BREAK | UNUSED);
        self.set_flag(INTERRUPT_DISABLE, true);
        let lo = self.read(0xFFFE) as u16;
        let hi = self.read(0xFFFF) as u16;
        self.pc = (hi << 8) | lo;
    }

    fn adc(&mut self, addr: u16) {
        let value = self.read(addr);
        self.add_to_a(value);
    }

    fn sbc(&mut self, addr: u16) {
        let value = self.read(addr);
        // SBC = ADC with complement: A - M - (1-C) = A + ~M + C
        self.add_to_a(!value);
    }

    fn add_to_a(&mut self, value: u8) {
        let carry = if self.status & CARRY != 0 { 1u16 } else { 0 };
        let sum = self.a as u16 + value as u16 + carry;

        self.set_flag(CARRY, sum > 0xFF);
        let result = sum as u8;
        self.set_flag(
            OVERFLOW,
            (!(self.a ^ value) & (self.a ^ result)) & 0x80 != 0,
        );
        self.a = result;
        self.update_zero_negative(self.a);
    }

    fn branch(&mut self, addr: u16) {
        self.cycles += 1;
        if (self.pc & 0xFF00) != (addr & 0xFF00) {
            self.cycles += 1;
        }
        self.pc = addr;
    }

    fn compare(&mut self, reg: u8, addr: u16) {
        let value = self.read(addr);
        self.set_flag(CARRY, reg >= value);
        self.update_zero_negative(reg.wrapping_sub(value));
    }

    fn bit(&mut self, addr: u16) {
        let value = self.read(addr);
        self.set_flag(ZERO, self.a & value == 0);
        self.set_flag(OVERFLOW, value & 0x40 != 0);
        self.set_flag(NEGATIVE, value & 0x80 != 0);
    }

    fn inc(&mut self, addr: u16) {
        let value = self.read(addr).wrapping_add(1);
        self.write(addr, value);
        self.update_zero_negative(value);
    }

    fn dec(&mut self, addr: u16) {
        let value = self.read(addr).wrapping_sub(1);
        self.write(addr, value);
        self.update_zero_negative(value);
    }

    fn and(&mut self, addr: u16) {
        self.a &= self.read(addr);
        self.update_zero_negative(self.a);
    }

    fn asl_accumulator(&mut self) {
        self.set_flag(CARRY, self.a & 0x80 != 0);
        self.a <<= 1;
        self.update_zero_negative(self.a);
    }

    fn asl_memory(&mut self, addr: u16) {
        let mut value = self.read(addr);
        self.set_flag(CARRY, value & 0x80 != 0);
        value <<= 1;
        self.write(addr, value);
        self.update_zero_negative(value);
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

    #[test]
    fn adc_no_carry() {
        let cart = test_cartridge(&[0x69, 0x20]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x10;
        cpu.step();

        assert_eq!(cpu.a, 0x30);
        assert_eq!(cpu.status & CARRY, 0);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
        assert_eq!(cpu.status & OVERFLOW, 0);
    }

    #[test]
    fn adc_with_carry_in() {
        let cart = test_cartridge(&[0x69, 0x20]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x10;
        cpu.status |= CARRY;
        cpu.step();

        assert_eq!(cpu.a, 0x31);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn adc_carry_out() {
        let cart = test_cartridge(&[0x69, 0x01]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0xFF;
        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn adc_overflow_positive() {
        // 正+正=負 → V=1
        let cart = test_cartridge(&[0x69, 0x50]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x50;
        cpu.step();

        assert_eq!(cpu.a, 0xA0);
        assert_ne!(cpu.status & OVERFLOW, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn adc_overflow_negative() {
        // 負+負=正 → V=1, C=1
        let cart = test_cartridge(&[0x69, 0x80]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x80;
        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & OVERFLOW, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn adc_no_overflow_mixed_signs() {
        // 正+負 → V=0
        let cart = test_cartridge(&[0x69, 0xD0]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x50;
        cpu.step();

        assert_eq!(cpu.a, 0x20);
        assert_eq!(cpu.status & OVERFLOW, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn and_immediate() {
        let cart = test_cartridge(&[0x29, 0x0F]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0xAB;
        cpu.step();

        assert_eq!(cpu.a, 0x0B);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn and_zero_flag() {
        let cart = test_cartridge(&[0x29, 0x0F]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0xF0;
        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn and_negative_flag() {
        let cart = test_cartridge(&[0x29, 0xF0]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x85;
        cpu.step();

        assert_eq!(cpu.a, 0x80);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn asl_accumulator() {
        let cart = test_cartridge(&[0x0A]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x25;
        cpu.step();

        assert_eq!(cpu.a, 0x4A);
        assert_eq!(cpu.status & CARRY, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn asl_accumulator_carry() {
        // bit7が1 → C=1
        let cart = test_cartridge(&[0x0A]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x81;
        cpu.step();

        assert_eq!(cpu.a, 0x02);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn asl_accumulator_zero() {
        let cart = test_cartridge(&[0x0A]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x80;
        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn asl_memory_zero_page() {
        let cart = test_cartridge(&[0x06, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x10] = 0x25;
        cpu.step();

        assert_eq!(cpu.ram[0x10], 0x4A);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn asl_memory_carry() {
        let cart = test_cartridge(&[0x06, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x10] = 0xC0;
        cpu.step();

        assert_eq!(cpu.ram[0x10], 0x80);
        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn bcc_branch_taken() {
        // C=0 → 分岐する: PC=$8002, offset=0x04 → $8006
        let cart = test_cartridge(&[0x90, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bcc_branch_not_taken() {
        // C=1 → 分岐しない: PC=$8002
        let cart = test_cartridge(&[0x90, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.status |= CARRY;
        cpu.step();

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn bcc_branch_backward() {
        // C=0, offset=0xFC(-4) → PC=$8002+(-4)=$7FFE
        let cart = test_cartridge(&[0x90, 0xFC]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.pc, 0x7FFE);
    }

    #[test]
    fn bcs_branch_taken() {
        let cart = test_cartridge(&[0xB0, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.status |= CARRY;
        cpu.step();

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bcs_branch_not_taken() {
        let cart = test_cartridge(&[0xB0, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn beq_branch_taken() {
        let cart = test_cartridge(&[0xF0, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.status |= ZERO;
        cpu.step();

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn beq_branch_not_taken() {
        let cart = test_cartridge(&[0xF0, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn bit_zero_flag() {
        // A & M == 0 → Z=1
        let cart = test_cartridge(&[0x24, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x0F;
        cpu.ram[0x10] = 0xF0;
        cpu.step();

        assert_ne!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
        assert_ne!(cpu.status & OVERFLOW, 0);
    }

    #[test]
    fn bit_not_zero() {
        // A & M != 0 → Z=0
        let cart = test_cartridge(&[0x24, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0xFF;
        cpu.ram[0x10] = 0x3F;
        cpu.step();

        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
        assert_eq!(cpu.status & OVERFLOW, 0);
    }

    #[test]
    fn bit_flags_from_memory() {
        // N,V はメモリの値のbit7,bit6から取る（Aに関係なく）
        let cart = test_cartridge(&[0x24, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0xC0;
        cpu.ram[0x10] = 0xC0;
        cpu.step();

        assert_eq!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
        assert_ne!(cpu.status & OVERFLOW, 0);
    }

    #[test]
    fn bmi_branch_taken() {
        let cart = test_cartridge(&[0x30, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.status |= NEGATIVE;
        cpu.step();

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bmi_branch_not_taken() {
        let cart = test_cartridge(&[0x30, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn bne_branch_taken() {
        let cart = test_cartridge(&[0xD0, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bne_branch_not_taken() {
        let cart = test_cartridge(&[0xD0, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.status |= ZERO;
        cpu.step();

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn bpl_branch_taken() {
        let cart = test_cartridge(&[0x10, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bpl_branch_not_taken() {
        let cart = test_cartridge(&[0x10, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.status |= NEGATIVE;
        cpu.step();

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn brk() {
        // IRQベクタ ($FFFE-$FFFF) に $C000 を設定
        let mut prg_rom = vec![0u8; 0x8000];
        prg_rom[0] = 0x00; // BRK at $8000
        prg_rom[0x7FFC] = 0x00;
        prg_rom[0x7FFD] = 0x80; // reset → $8000
        prg_rom[0x7FFE] = 0x00;
        prg_rom[0x7FFF] = 0xC0; // IRQ → $C000
        let cart = Cartridge::new_test(prg_rom);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();

        let old_status = cpu.status;
        let old_sp = cpu.sp;
        cpu.step();

        // PCがIRQベクタに設定される
        assert_eq!(cpu.pc, 0xC000);
        // スタックにPC(2バイト)+status(1バイト)がpushされる
        assert_eq!(cpu.sp, old_sp.wrapping_sub(3));
        // Iフラグがセットされる
        assert_ne!(cpu.status & INTERRUPT_DISABLE, 0);
        // スタック上のstatusにBフラグが立っている
        let pushed_status = cpu.ram[0x0100 + cpu.sp.wrapping_add(1) as usize];
        assert_ne!(pushed_status & BREAK, 0);
        assert_ne!(pushed_status & UNUSED, 0);
    }

    #[test]
    fn bvc_branch_taken() {
        let cart = test_cartridge(&[0x50, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bvc_branch_not_taken() {
        let cart = test_cartridge(&[0x50, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.status |= OVERFLOW;
        cpu.step();

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn bvs_branch_taken() {
        let cart = test_cartridge(&[0x70, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.status |= OVERFLOW;
        cpu.step();

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bvs_branch_not_taken() {
        let cart = test_cartridge(&[0x70, 0x04]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn clc() {
        let cart = test_cartridge(&[0x18]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.status |= CARRY;
        cpu.step();

        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn cld() {
        let cart = test_cartridge(&[0xD8]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.status |= DECIMAL;
        cpu.step();

        assert_eq!(cpu.status & DECIMAL, 0);
    }

    #[test]
    fn cli() {
        let cart = test_cartridge(&[0x58]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        // resetでIフラグはセット済み
        assert_ne!(cpu.status & INTERRUPT_DISABLE, 0);
        cpu.step();

        assert_eq!(cpu.status & INTERRUPT_DISABLE, 0);
    }

    #[test]
    fn clv() {
        let cart = test_cartridge(&[0xB8]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.status |= OVERFLOW;
        cpu.step();

        assert_eq!(cpu.status & OVERFLOW, 0);
    }

    #[test]
    fn cmp_equal() {
        let cart = test_cartridge(&[0xC9, 0x42]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x42;
        cpu.step();

        assert_ne!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & CARRY, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn cmp_greater() {
        let cart = test_cartridge(&[0xC9, 0x20]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x42;
        cpu.step();

        assert_eq!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn cmp_less() {
        let cart = test_cartridge(&[0xC9, 0x50]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x42;
        cpu.step();

        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn cpx_equal() {
        let cart = test_cartridge(&[0xE0, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.x = 0x10;
        cpu.step();

        assert_ne!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn cpx_less() {
        let cart = test_cartridge(&[0xE0, 0x20]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.x = 0x10;
        cpu.step();

        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn cpy_equal() {
        let cart = test_cartridge(&[0xC0, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.y = 0x10;
        cpu.step();

        assert_ne!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn cpy_less() {
        let cart = test_cartridge(&[0xC0, 0x20]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.y = 0x10;
        cpu.step();

        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn dec() {
        let cart = test_cartridge(&[0xC6, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x10] = 0x05;
        cpu.step();

        assert_eq!(cpu.ram[0x10], 0x04);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn dec_zero() {
        let cart = test_cartridge(&[0xC6, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x10] = 0x01;
        cpu.step();

        assert_eq!(cpu.ram[0x10], 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn dec_wrap() {
        let cart = test_cartridge(&[0xC6, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x10] = 0x00;
        cpu.step();

        assert_eq!(cpu.ram[0x10], 0xFF);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn dex() {
        let cart = test_cartridge(&[0xCA]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.x = 0x05;
        cpu.step();

        assert_eq!(cpu.x, 0x04);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn dex_zero() {
        let cart = test_cartridge(&[0xCA]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.x = 0x01;
        cpu.step();

        assert_eq!(cpu.x, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn dey() {
        let cart = test_cartridge(&[0x88]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.y = 0x05;
        cpu.step();

        assert_eq!(cpu.y, 0x04);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn dey_zero() {
        let cart = test_cartridge(&[0x88]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.y = 0x01;
        cpu.step();

        assert_eq!(cpu.y, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn eor_immediate() {
        let cart = test_cartridge(&[0x49, 0xFF]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0xAA;
        cpu.step();

        assert_eq!(cpu.a, 0x55);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn eor_zero_flag() {
        let cart = test_cartridge(&[0x49, 0xFF]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0xFF;
        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn eor_negative_flag() {
        let cart = test_cartridge(&[0x49, 0x0F]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x8F;
        cpu.step();

        assert_eq!(cpu.a, 0x80);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn inc() {
        let cart = test_cartridge(&[0xE6, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x10] = 0x05;
        cpu.step();

        assert_eq!(cpu.ram[0x10], 0x06);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn inc_wrap() {
        let cart = test_cartridge(&[0xE6, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x10] = 0xFF;
        cpu.step();

        assert_eq!(cpu.ram[0x10], 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn inc_negative() {
        let cart = test_cartridge(&[0xE6, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x10] = 0x7F;
        cpu.step();

        assert_eq!(cpu.ram[0x10], 0x80);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn inx() {
        let cart = test_cartridge(&[0xE8]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.x = 0x05;
        cpu.step();

        assert_eq!(cpu.x, 0x06);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn inx_wrap() {
        let cart = test_cartridge(&[0xE8]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.x = 0xFF;
        cpu.step();

        assert_eq!(cpu.x, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn iny() {
        let cart = test_cartridge(&[0xC8]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.y = 0x05;
        cpu.step();

        assert_eq!(cpu.y, 0x06);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn iny_wrap() {
        let cart = test_cartridge(&[0xC8]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.y = 0xFF;
        cpu.step();

        assert_eq!(cpu.y, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn jmp_absolute() {
        let cart = test_cartridge(&[0x4C, 0x00, 0x90]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.pc, 0x9000);
    }

    #[test]
    fn jmp_indirect() {
        // $0200-$0201 に $9000 を格納
        let cart = test_cartridge(&[0x6C, 0x00, 0x02]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x0200] = 0x00;
        cpu.ram[0x0201] = 0x90;
        cpu.step();

        assert_eq!(cpu.pc, 0x9000);
    }

    #[test]
    fn jmp_indirect_page_boundary_bug() {
        // $02FF-$0200 (not $0300) から読む6502バグの再現
        let cart = test_cartridge(&[0x6C, 0xFF, 0x02]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x02FF] = 0x40;
        cpu.ram[0x0200] = 0x80; // $0300ではなく$0200から読む
        cpu.ram[0x0300] = 0xFF; // これは使われない
        cpu.step();

        assert_eq!(cpu.pc, 0x8040);
    }

    #[test]
    fn jsr() {
        // JSR $9000 at $8000
        let cart = test_cartridge(&[0x20, 0x00, 0x90]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        let old_sp = cpu.sp;
        cpu.step();

        assert_eq!(cpu.pc, 0x9000);
        assert_eq!(cpu.sp, old_sp.wrapping_sub(2));
        // スタックにはPC-1 ($8002) が格納される
        let hi = cpu.ram[0x0100 + old_sp as usize];
        let lo = cpu.ram[0x0100 + old_sp.wrapping_sub(1) as usize];
        assert_eq!((hi as u16) << 8 | lo as u16, 0x8002);
    }

    #[test]
    fn ldx_immediate() {
        let cart = test_cartridge(&[0xA2, 0x42]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.x, 0x42);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn ldx_zero_page_y() {
        let cart = test_cartridge(&[0xB6, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.y = 0x05;
        cpu.ram[0x15] = 0x99;
        cpu.step();

        assert_eq!(cpu.x, 0x99);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn ldy_immediate() {
        let cart = test_cartridge(&[0xA0, 0x42]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.step();

        assert_eq!(cpu.y, 0x42);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn ldy_zero_page_x() {
        let cart = test_cartridge(&[0xB4, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.x = 0x05;
        cpu.ram[0x15] = 0x99;
        cpu.step();

        assert_eq!(cpu.y, 0x99);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn lsr_accumulator() {
        let cart = test_cartridge(&[0x4A]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x4A;
        cpu.step();

        assert_eq!(cpu.a, 0x25);
        assert_eq!(cpu.status & CARRY, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn lsr_accumulator_carry() {
        let cart = test_cartridge(&[0x4A]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x03;
        cpu.step();

        assert_eq!(cpu.a, 0x01);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn lsr_accumulator_zero() {
        let cart = test_cartridge(&[0x4A]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x01;
        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn lsr_memory() {
        let cart = test_cartridge(&[0x46, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x10] = 0x81;
        cpu.step();

        assert_eq!(cpu.ram[0x10], 0x40);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn ora_immediate() {
        let cart = test_cartridge(&[0x09, 0xF0]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x0F;
        cpu.step();

        assert_eq!(cpu.a, 0xFF);
        assert_ne!(cpu.status & NEGATIVE, 0);
        assert_eq!(cpu.status & ZERO, 0);
    }

    #[test]
    fn ora_zero_flag() {
        let cart = test_cartridge(&[0x09, 0x00]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x00;
        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn pha() {
        let cart = test_cartridge(&[0x48]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x42;
        let old_sp = cpu.sp;
        cpu.step();

        assert_eq!(cpu.sp, old_sp.wrapping_sub(1));
        assert_eq!(cpu.ram[0x0100 + old_sp as usize], 0x42);
    }

    #[test]
    fn php() {
        let cart = test_cartridge(&[0x08]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.status = CARRY | NEGATIVE | UNUSED | INTERRUPT_DISABLE;
        let old_sp = cpu.sp;
        cpu.step();

        assert_eq!(cpu.sp, old_sp.wrapping_sub(1));
        let pushed = cpu.ram[0x0100 + old_sp as usize];
        assert_ne!(pushed & BREAK, 0);
        assert_ne!(pushed & UNUSED, 0);
        assert_ne!(pushed & CARRY, 0);
        assert_ne!(pushed & NEGATIVE, 0);
    }

    #[test]
    fn pla() {
        // PHA then PLA
        let cart = test_cartridge(&[0x48, 0xA9, 0x00, 0x68]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x42;
        cpu.step(); // PHA
        cpu.step(); // LDA #$00
        assert_eq!(cpu.a, 0x00);
        cpu.step(); // PLA

        assert_eq!(cpu.a, 0x42);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn pla_zero_flag() {
        let cart = test_cartridge(&[0x68]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0xFF;
        // スタックに0をpush
        cpu.push(0x00);
        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn plp() {
        let cart = test_cartridge(&[0x28]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.push(CARRY | NEGATIVE | BREAK);
        cpu.step();

        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
        // Bフラグは無視される
        assert_eq!(cpu.status & BREAK, 0);
        // Uフラグは常にセット
        assert_ne!(cpu.status & UNUSED, 0);
    }

    #[test]
    fn rol_accumulator_no_carry() {
        let cart = test_cartridge(&[0x2A]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x25;
        cpu.step();

        assert_eq!(cpu.a, 0x4A);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn rol_accumulator_with_carry_in() {
        let cart = test_cartridge(&[0x2A]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x25;
        cpu.status |= CARRY;
        cpu.step();

        assert_eq!(cpu.a, 0x4B);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn rol_accumulator_carry_out() {
        let cart = test_cartridge(&[0x2A]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x80;
        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn rol_memory() {
        let cart = test_cartridge(&[0x26, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x10] = 0x85;
        cpu.status |= CARRY;
        cpu.step();

        assert_eq!(cpu.ram[0x10], 0x0B);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn ror_accumulator_no_carry() {
        let cart = test_cartridge(&[0x6A]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x4A;
        cpu.step();

        assert_eq!(cpu.a, 0x25);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn ror_accumulator_with_carry_in() {
        let cart = test_cartridge(&[0x6A]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x4A;
        cpu.status |= CARRY;
        cpu.step();

        assert_eq!(cpu.a, 0xA5);
        assert_eq!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn ror_accumulator_carry_out() {
        let cart = test_cartridge(&[0x6A]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x01;
        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn ror_memory() {
        let cart = test_cartridge(&[0x66, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.ram[0x10] = 0x03;
        cpu.status |= CARRY;
        cpu.step();

        assert_eq!(cpu.ram[0x10], 0x81);
        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn rti() {
        // BRK → RTI の往復
        let mut prg_rom = vec![0u8; 0x8000];
        prg_rom[0] = 0x00; // BRK at $8000
        // IRQハンドラ at $C000: RTI
        prg_rom[0x4000] = 0x40;
        prg_rom[0x7FFC] = 0x00;
        prg_rom[0x7FFD] = 0x80;
        prg_rom[0x7FFE] = 0x00;
        prg_rom[0x7FFF] = 0xC0;
        let cart = Cartridge::new_test(prg_rom);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.status |= CARRY;

        let status_before = cpu.status;
        cpu.step(); // BRK
        assert_eq!(cpu.pc, 0xC000);

        cpu.step(); // RTI
        assert_eq!(cpu.pc, 0x8001);
        // Bフラグは無視、Uフラグは常にセット
        assert_eq!(cpu.status & BREAK, 0);
        assert_ne!(cpu.status & UNUSED, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn jsr_rts() {
        // JSR $8010, then RTS at $8010
        let mut prg_rom = vec![0u8; 0x8000];
        prg_rom[0] = 0x20; // JSR
        prg_rom[1] = 0x10;
        prg_rom[2] = 0x80; // → $8010
        prg_rom[0x10] = 0x60; // RTS at $8010
        prg_rom[0x7FFC] = 0x00;
        prg_rom[0x7FFD] = 0x80;
        let cart = Cartridge::new_test(prg_rom);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();

        cpu.step(); // JSR $8010
        assert_eq!(cpu.pc, 0x8010);

        cpu.step(); // RTS
        assert_eq!(cpu.pc, 0x8003);
    }

    #[test]
    fn sbc_no_borrow() {
        // 0x50 - 0x10 (C=1, no borrow) = 0x40
        let cart = test_cartridge(&[0xE9, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x50;
        cpu.status |= CARRY;
        cpu.step();

        assert_eq!(cpu.a, 0x40);
        assert_ne!(cpu.status & CARRY, 0);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
        assert_eq!(cpu.status & OVERFLOW, 0);
    }

    #[test]
    fn sbc_with_borrow() {
        // 0x50 - 0x10 - 1(borrow, C=0) = 0x3F
        let cart = test_cartridge(&[0xE9, 0x10]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x50;
        // C=0 (borrow)
        cpu.step();

        assert_eq!(cpu.a, 0x3F);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn sbc_underflow() {
        // 0x00 - 0x01 (C=1) = 0xFF, C=0
        let cart = test_cartridge(&[0xE9, 0x01]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x00;
        cpu.status |= CARRY;
        cpu.step();

        assert_eq!(cpu.a, 0xFF);
        assert_eq!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn sbc_overflow_positive() {
        // 0x50 - 0xB0 (C=1) → 正-負=負 → V=1
        let cart = test_cartridge(&[0xE9, 0xB0]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x50;
        cpu.status |= CARRY;
        cpu.step();

        assert_eq!(cpu.a, 0xA0);
        assert_ne!(cpu.status & OVERFLOW, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn sbc_overflow_negative() {
        // 0xD0 - 0x70 (C=1) → 負-正=正 → V=1
        let cart = test_cartridge(&[0xE9, 0x70]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0xD0;
        cpu.status |= CARRY;
        cpu.step();

        assert_eq!(cpu.a, 0x60);
        assert_ne!(cpu.status & OVERFLOW, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn sbc_zero() {
        // 0x50 - 0x50 (C=1) = 0x00
        let cart = test_cartridge(&[0xE9, 0x50]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        cpu.a = 0x50;
        cpu.status |= CARRY;
        cpu.step();

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn sec() {
        let cart = test_cartridge(&[0x38]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        assert_eq!(cpu.status & CARRY, 0);
        cpu.step();

        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn sed() {
        let cart = test_cartridge(&[0xF8]);
        let mut ppu = PPU::new(&cart);
        let mut cpu = CPU::new(&mut ppu, &cart);
        cpu.reset();
        assert_eq!(cpu.status & DECIMAL, 0);
        cpu.step();

        assert_ne!(cpu.status & DECIMAL, 0);
    }
}
