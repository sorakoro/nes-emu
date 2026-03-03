mod opcodes;

use crate::bus::Bus;
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

pub struct Cpu {
    a: u8,
    x: u8,
    y: u8,
    pc: u16,
    sp: u8,
    status: u8,

    cycles: u64,
    pub trace: bool,
}

impl Cpu {
    pub fn new() -> Self {
        Cpu {
            a: 0,
            x: 0,
            y: 0,
            pc: 0,
            sp: 0xFD,
            status: UNUSED | INTERRUPT_DISABLE,

            cycles: 0,
            trace: false,
        }
    }

    fn trace_instruction(&self, bus: &Bus) {
        let pc = self.pc;
        let opcode = bus.peek(pc);
        let inst = match OPCODES[opcode as usize] {
            Some(inst) => inst,
            None => {
                println!("{:04X}  {:02X}        ???", pc, opcode);
                return;
            }
        };

        let b1 = bus.peek(pc.wrapping_add(1));
        let b2 = bus.peek(pc.wrapping_add(2));

        let raw_bytes = match inst.mode {
            Implicit | Accumulator => format!("{:02X}         ", opcode),
            Immediate | ZeroPage | ZeroPageX | ZeroPageY | Relative | IndirectX | IndirectY => {
                format!("{:02X} {:02X}      ", opcode, b1)
            }
            Absolute | AbsoluteX | AbsoluteY | Indirect => {
                format!("{:02X} {:02X} {:02X}   ", opcode, b1, b2)
            }
        };

        let addr16 = (b2 as u16) << 8 | b1 as u16;
        let operand = match inst.mode {
            Implicit => String::new(),
            Accumulator => "A".to_string(),
            Immediate => format!("#${:02X}", b1),
            ZeroPage => format!("${:02X}", b1),
            ZeroPageX => format!("${:02X},X", b1),
            ZeroPageY => format!("${:02X},Y", b1),
            Absolute => format!("${:04X}", addr16),
            AbsoluteX => format!("${:04X},X", addr16),
            AbsoluteY => format!("${:04X},Y", addr16),
            Indirect => format!("(${:04X})", addr16),
            IndirectX => format!("(${:02X},X)", b1),
            IndirectY => format!("(${:02X}),Y", b1),
            Relative => {
                let offset = b1 as i8;
                let target = pc.wrapping_add(2).wrapping_add(offset as u16);
                format!("${:04X}", target)
            }
        };

        println!(
            "{:04X}  {}{:<32}A:{:02X} X:{:02X} Y:{:02X} P:{:02X} SP:{:02X} CYC:{}",
            pc,
            raw_bytes,
            format!("{} {}", inst.name, operand),
            self.a,
            self.x,
            self.y,
            self.status,
            self.sp,
            self.cycles
        );
    }

    pub fn reset(&mut self, bus: &mut Bus) {
        self.a = 0;
        self.x = 0;
        self.y = 0;
        self.sp = 0xFD;
        self.status = UNUSED | INTERRUPT_DISABLE;

        let lo = bus.read(0xFFFC) as u16;
        let hi = bus.read(0xFFFD) as u16;
        self.pc = (hi << 8) | lo;

        self.cycles = 0;
    }

    fn write(&mut self, bus: &mut Bus, addr: u16, value: u8) {
        if addr == 0x4014 {
            let page = value as u16;
            let start_addr = page * 0x100;
            let data: [u8; 256] = std::array::from_fn(|i| bus.read(start_addr + i as u16));
            bus.ppu.write_oam_dma(&data);
            let dma_cpu_cycles = 513 + (self.cycles & 1);
            self.cycles += dma_cpu_cycles;
            bus.tick_ppu(dma_cpu_cycles as u16 * 3);
            let dmc_stall = bus.tick_apu(dma_cpu_cycles as u16);
            if dmc_stall > 0 {
                self.cycles += dmc_stall as u64;
                bus.tick_ppu(dmc_stall * 3);
            }
        } else {
            bus.write(addr, value);
        }
    }

    fn fetch(&mut self, bus: &mut Bus) -> u8 {
        let value = bus.read(self.pc);
        self.pc = self.pc.wrapping_add(1);
        value
    }

    fn fetch_u16(&mut self, bus: &mut Bus) -> u16 {
        let lo = self.fetch(bus) as u16;
        let hi = self.fetch(bus) as u16;
        (hi << 8) | lo
    }

    fn addr_immediate(&mut self) -> u16 {
        let addr = self.pc;
        self.pc = self.pc.wrapping_add(1);
        addr
    }

    fn addr_zero_page(&mut self, bus: &mut Bus) -> u16 {
        self.fetch(bus) as u16
    }

    fn addr_zero_page_x(&mut self, bus: &mut Bus) -> u16 {
        self.fetch(bus).wrapping_add(self.x) as u16
    }

    fn addr_zero_page_y(&mut self, bus: &mut Bus) -> u16 {
        self.fetch(bus).wrapping_add(self.y) as u16
    }

    fn addr_absolute(&mut self, bus: &mut Bus) -> u16 {
        self.fetch_u16(bus)
    }

    fn addr_absolute_x(&mut self, bus: &mut Bus) -> (u16, bool) {
        let base = self.fetch_u16(bus);
        let addr = base.wrapping_add(self.x as u16);
        let page_crossed = (base & 0xFF00) != (addr & 0xFF00);
        (addr, page_crossed)
    }

    fn addr_absolute_y(&mut self, bus: &mut Bus) -> (u16, bool) {
        let base = self.fetch_u16(bus);
        let addr = base.wrapping_add(self.y as u16);
        let page_crossed = (base & 0xFF00) != (addr & 0xFF00);
        (addr, page_crossed)
    }

    fn addr_indirect(&mut self, bus: &mut Bus) -> u16 {
        let ptr = self.fetch_u16(bus);
        let lo = bus.read(ptr) as u16;
        let hi_addr = (ptr & 0xFF00) | ((ptr.wrapping_add(1)) & 0x00FF);
        let hi = bus.read(hi_addr) as u16;
        (hi << 8) | lo
    }

    fn addr_indirect_x(&mut self, bus: &mut Bus) -> u16 {
        let ptr = self.fetch(bus).wrapping_add(self.x);
        let lo = bus.read(ptr as u16) as u16;
        let hi = bus.read(ptr.wrapping_add(1) as u16) as u16;
        (hi << 8) | lo
    }

    fn addr_indirect_y(&mut self, bus: &mut Bus) -> (u16, bool) {
        let ptr = self.fetch(bus);
        let lo = bus.read(ptr as u16) as u16;
        let hi = bus.read(ptr.wrapping_add(1) as u16) as u16;
        let base = (hi << 8) | lo;
        let addr = base.wrapping_add(self.y as u16);
        let page_crossed = (base & 0xFF00) != (addr & 0xFF00);
        (addr, page_crossed)
    }

    fn addr_relative(&mut self, bus: &mut Bus) -> u16 {
        let offset = self.fetch(bus) as i8;
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

    fn fetch_operand_addr(&mut self, bus: &mut Bus, mode: AddressingMode) -> (u16, bool) {
        match mode {
            Implicit | Accumulator => (0, false),
            Immediate => (self.addr_immediate(), false),
            ZeroPage => (self.addr_zero_page(bus), false),
            ZeroPageX => (self.addr_zero_page_x(bus), false),
            ZeroPageY => (self.addr_zero_page_y(bus), false),
            Absolute => (self.addr_absolute(bus), false),
            AbsoluteX => self.addr_absolute_x(bus),
            AbsoluteY => self.addr_absolute_y(bus),
            Indirect => (self.addr_indirect(bus), false),
            IndirectX => (self.addr_indirect_x(bus), false),
            IndirectY => self.addr_indirect_y(bus),
            Relative => (self.addr_relative(bus), false),
        }
    }

    pub fn step(&mut self, bus: &mut Bus) -> bool {
        if self.trace {
            self.trace_instruction(bus);
        }
        let opcode = self.fetch(bus);

        let inst = match OPCODES[opcode as usize] {
            Some(inst) => inst,
            None => panic!(
                "Unknown opcode: 0x{:02X} at PC=0x{:04X}",
                opcode,
                self.pc.wrapping_sub(1)
            ),
        };

        let (addr, page_cross) = self.fetch_operand_addr(bus, inst.mode);

        match inst.name {
            "ADC" => self.adc(bus, addr),
            "AND" => self.and(bus, addr),
            "ASL" => {
                if inst.mode == Accumulator {
                    self.asl_accumulator();
                } else {
                    self.asl_memory(bus, addr);
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
            "BIT" => self.bit(bus, addr),
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
            "BRK" => self.brk(bus),
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
            "CMP" => self.compare(bus, self.a, addr),
            "CPX" => self.compare(bus, self.x, addr),
            "CPY" => self.compare(bus, self.y, addr),
            "DEC" => self.dec(bus, addr),
            "DEX" => {
                self.x = self.x.wrapping_sub(1);
                self.update_zero_negative(self.x);
            }
            "DEY" => {
                self.y = self.y.wrapping_sub(1);
                self.update_zero_negative(self.y);
            }
            "EOR" => {
                self.a ^= bus.read(addr);
                self.update_zero_negative(self.a);
            }
            "INC" => self.inc(bus, addr),
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
                self.push_u16(bus, self.pc.wrapping_sub(1));
                self.pc = addr;
            }
            "LDA" => self.lda(bus, addr),
            "LDX" => {
                self.x = bus.read(addr);
                self.update_zero_negative(self.x);
            }
            "LDY" => {
                self.y = bus.read(addr);
                self.update_zero_negative(self.y);
            }
            "LSR" => {
                if inst.mode == Accumulator {
                    self.set_flag(CARRY, self.a & 0x01 != 0);
                    self.a >>= 1;
                    self.update_zero_negative(self.a);
                } else {
                    let mut value = bus.read(addr);
                    self.set_flag(CARRY, value & 0x01 != 0);
                    value >>= 1;
                    self.write(bus, addr, value);
                    self.update_zero_negative(value);
                }
            }

            "NOP" => {}
            "ORA" => {
                self.a |= bus.read(addr);
                self.update_zero_negative(self.a);
            }
            "PHA" => self.push(bus, self.a),
            "PHP" => self.push(bus, self.status | BREAK | UNUSED),
            "PLA" => {
                self.a = self.pull(bus);
                self.update_zero_negative(self.a);
            }
            "PLP" => {
                self.status = (self.pull(bus) & !BREAK) | UNUSED;
            }
            "SBC" => self.sbc(bus, addr),
            "SEC" => self.set_flag(CARRY, true),
            "SED" => self.set_flag(DECIMAL, true),
            "SEI" => self.set_flag(INTERRUPT_DISABLE, true),
            "STA" => self.write(bus, addr, self.a),
            "STX" => self.write(bus, addr, self.x),
            "STY" => self.write(bus, addr, self.y),
            "TAX" => {
                self.x = self.a;
                self.update_zero_negative(self.x);
            }
            "TAY" => {
                self.y = self.a;
                self.update_zero_negative(self.y);
            }
            "TSX" => {
                self.x = self.sp;
                self.update_zero_negative(self.x);
            }
            "TXA" => {
                self.a = self.x;
                self.update_zero_negative(self.a);
            }
            "TXS" => {
                self.sp = self.x;
            }
            "TYA" => {
                self.a = self.y;
                self.update_zero_negative(self.a);
            }
            "RTS" => {
                let lo = self.pull(bus) as u16;
                let hi = self.pull(bus) as u16;
                self.pc = ((hi << 8) | lo).wrapping_add(1);
            }
            "RTI" => {
                self.status = (self.pull(bus) & !BREAK) | UNUSED;
                let lo = self.pull(bus) as u16;
                let hi = self.pull(bus) as u16;
                self.pc = (hi << 8) | lo;
            }
            "ROR" => {
                if inst.mode == Accumulator {
                    let old_carry = if self.status & CARRY != 0 { 0x80 } else { 0 };
                    self.set_flag(CARRY, self.a & 0x01 != 0);
                    self.a = (self.a >> 1) | old_carry;
                    self.update_zero_negative(self.a);
                } else {
                    let mut value = bus.read(addr);
                    let old_carry = if self.status & CARRY != 0 { 0x80 } else { 0 };
                    self.set_flag(CARRY, value & 0x01 != 0);
                    value = (value >> 1) | old_carry;
                    self.write(bus, addr, value);
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
                    let mut value = bus.read(addr);
                    let old_carry = self.status & CARRY;
                    self.set_flag(CARRY, value & 0x80 != 0);
                    value = (value << 1) | old_carry;
                    self.write(bus, addr, value);
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

        let ppu_cycles = cycles as u16 * 3;
        bus.tick_ppu(ppu_cycles);
        let dmc_stall = bus.tick_apu(cycles as u16);
        if dmc_stall > 0 {
            self.cycles += dmc_stall as u64;
            bus.tick_ppu(dmc_stall * 3);
        }
        if bus.ppu.poll_nmi() {
            self.nmi(bus);
        }
        if bus.apu.irq_pending() {
            self.irq(bus);
        }
        if bus.cartridge.irq_pending() {
            self.irq(bus);
        }

        if bus.ppu.frame_ready {
            bus.ppu.frame_ready = false;
            return true;
        }
        false
    }

    fn push(&mut self, bus: &mut Bus, value: u8) {
        bus.write(0x0100 + self.sp as u16, value);
        self.sp = self.sp.wrapping_sub(1);
    }

    fn pull(&mut self, bus: &mut Bus) -> u8 {
        self.sp = self.sp.wrapping_add(1);
        bus.read(0x0100 + self.sp as u16)
    }

    fn push_u16(&mut self, bus: &mut Bus, value: u16) {
        self.push(bus, (value >> 8) as u8);
        self.push(bus, value as u8);
    }

    fn brk(&mut self, bus: &mut Bus) {
        self.push_u16(bus, self.pc);
        self.push(bus, self.status | BREAK | UNUSED);
        self.set_flag(INTERRUPT_DISABLE, true);
        let lo = bus.read(0xFFFE) as u16;
        let hi = bus.read(0xFFFF) as u16;
        self.pc = (hi << 8) | lo;
    }

    fn adc(&mut self, bus: &mut Bus, addr: u16) {
        let value = bus.read(addr);
        self.add_to_a(value);
    }

    fn sbc(&mut self, bus: &mut Bus, addr: u16) {
        let value = bus.read(addr);
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

    fn compare(&mut self, bus: &mut Bus, reg: u8, addr: u16) {
        let value = bus.read(addr);
        self.set_flag(CARRY, reg >= value);
        self.update_zero_negative(reg.wrapping_sub(value));
    }

    fn bit(&mut self, bus: &mut Bus, addr: u16) {
        let value = bus.read(addr);
        self.set_flag(ZERO, self.a & value == 0);
        self.set_flag(OVERFLOW, value & 0x40 != 0);
        self.set_flag(NEGATIVE, value & 0x80 != 0);
    }

    fn inc(&mut self, bus: &mut Bus, addr: u16) {
        let value = bus.read(addr).wrapping_add(1);
        self.write(bus, addr, value);
        self.update_zero_negative(value);
    }

    fn dec(&mut self, bus: &mut Bus, addr: u16) {
        let value = bus.read(addr).wrapping_sub(1);
        self.write(bus, addr, value);
        self.update_zero_negative(value);
    }

    fn and(&mut self, bus: &mut Bus, addr: u16) {
        self.a &= bus.read(addr);
        self.update_zero_negative(self.a);
    }

    fn asl_accumulator(&mut self) {
        self.set_flag(CARRY, self.a & 0x80 != 0);
        self.a <<= 1;
        self.update_zero_negative(self.a);
    }

    fn asl_memory(&mut self, bus: &mut Bus, addr: u16) {
        let mut value = bus.read(addr);
        self.set_flag(CARRY, value & 0x80 != 0);
        value <<= 1;
        self.write(bus, addr, value);
        self.update_zero_negative(value);
    }

    fn lda(&mut self, bus: &mut Bus, addr: u16) {
        self.a = bus.read(addr);
        self.update_zero_negative(self.a);
    }

    pub fn nmi(&mut self, bus: &mut Bus) {
        self.push_u16(bus, self.pc);
        self.push(bus, (self.status & !BREAK) | UNUSED);
        self.set_flag(INTERRUPT_DISABLE, true);
        let lo = bus.read(0xFFFA) as u16;
        let hi = bus.read(0xFFFB) as u16;
        self.pc = (hi << 8) | lo;
    }

    pub fn irq(&mut self, bus: &mut Bus) {
        if self.status & INTERRUPT_DISABLE != 0 {
            return;
        }
        self.push_u16(bus, self.pc);
        self.push(bus, (self.status & !BREAK) | UNUSED);
        self.set_flag(INTERRUPT_DISABLE, true);
        let lo = bus.read(0xFFFE) as u16;
        let hi = bus.read(0xFFFF) as u16;
        self.pc = (hi << 8) | lo;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cart::Cartridge;

    fn test_cartridge(program: &[u8]) -> Cartridge {
        let mut prg_rom = vec![0u8; 0x8000];
        prg_rom[..program.len()].copy_from_slice(program);
        // reset vector → $8000
        prg_rom[0x7FFC] = 0x00;
        prg_rom[0x7FFD] = 0x80;
        Cartridge::new_test(prg_rom)
    }

    fn test_bus(program: &[u8]) -> Bus {
        Bus::new(test_cartridge(program))
    }

    #[test]
    fn lda_immediate() {
        let mut bus = test_bus(&[0xA9, 0x42]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x42);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn lda_immediate_zero_flag() {
        let mut bus = test_bus(&[0xA9, 0x00]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn lda_immediate_negative_flag() {
        let mut bus = test_bus(&[0xA9, 0x80]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x80);
        assert_eq!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn lda_zero_page() {
        let mut bus = test_bus(&[0xA5, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x10] = 0x55;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x55);
    }

    #[test]
    fn lda_zero_page_x() {
        let mut bus = test_bus(&[0xB5, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x05;
        bus.ram[0x15] = 0x66;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x66);
    }

    #[test]
    fn lda_zero_page_x_wraps() {
        let mut bus = test_bus(&[0xB5, 0xFF]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x01;
        bus.ram[0x00] = 0x77;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x77);
    }

    #[test]
    fn lda_absolute() {
        let mut bus = test_bus(&[0xAD, 0x00, 0x02]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x0200] = 0x88;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x88);
    }

    #[test]
    fn lda_absolute_x() {
        let mut bus = test_bus(&[0xBD, 0x00, 0x02]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x05;
        bus.ram[0x0205] = 0x99;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x99);
    }

    #[test]
    fn lda_absolute_y() {
        let mut bus = test_bus(&[0xB9, 0x00, 0x02]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.y = 0x03;
        bus.ram[0x0203] = 0xAA;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0xAA);
    }

    #[test]
    fn lda_indirect_x() {
        let mut bus = test_bus(&[0xA1, 0x20]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x04;
        // pointer at ZP $24-$25 → $0300
        bus.ram[0x24] = 0x00;
        bus.ram[0x25] = 0x03;
        bus.ram[0x0300] = 0xBB;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0xBB);
    }

    #[test]
    fn lda_indirect_y() {
        let mut bus = test_bus(&[0xB1, 0x40]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.y = 0x10;
        // pointer at ZP $40-$41 → $0300
        bus.ram[0x40] = 0x00;
        bus.ram[0x41] = 0x03;
        // $0300 + $10 = $0310
        bus.ram[0x0310] = 0xCC;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0xCC);
    }

    #[test]
    fn adc_no_carry() {
        let mut bus = test_bus(&[0x69, 0x20]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x10;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x30);
        assert_eq!(cpu.status & CARRY, 0);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
        assert_eq!(cpu.status & OVERFLOW, 0);
    }

    #[test]
    fn adc_with_carry_in() {
        let mut bus = test_bus(&[0x69, 0x20]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x10;
        cpu.status |= CARRY;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x31);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn adc_carry_out() {
        let mut bus = test_bus(&[0x69, 0x01]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0xFF;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn adc_overflow_positive() {
        // 正+正=負 → V=1
        let mut bus = test_bus(&[0x69, 0x50]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x50;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0xA0);
        assert_ne!(cpu.status & OVERFLOW, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn adc_overflow_negative() {
        // 負+負=正 → V=1, C=1
        let mut bus = test_bus(&[0x69, 0x80]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x80;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & OVERFLOW, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn adc_no_overflow_mixed_signs() {
        // 正+負 → V=0
        let mut bus = test_bus(&[0x69, 0xD0]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x50;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x20);
        assert_eq!(cpu.status & OVERFLOW, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn and_immediate() {
        let mut bus = test_bus(&[0x29, 0x0F]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0xAB;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x0B);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn and_zero_flag() {
        let mut bus = test_bus(&[0x29, 0x0F]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0xF0;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn and_negative_flag() {
        let mut bus = test_bus(&[0x29, 0xF0]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x85;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x80);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn asl_accumulator() {
        let mut bus = test_bus(&[0x0A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x25;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x4A);
        assert_eq!(cpu.status & CARRY, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn asl_accumulator_carry() {
        // bit7が1 → C=1
        let mut bus = test_bus(&[0x0A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x81;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x02);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn asl_accumulator_zero() {
        let mut bus = test_bus(&[0x0A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x80;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn asl_memory_zero_page() {
        let mut bus = test_bus(&[0x06, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x10] = 0x25;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0x4A);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn asl_memory_carry() {
        let mut bus = test_bus(&[0x06, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x10] = 0xC0;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0x80);
        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn bcc_branch_taken() {
        // C=0 → 分岐する: PC=$8002, offset=0x04 → $8006
        let mut bus = test_bus(&[0x90, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bcc_branch_not_taken() {
        // C=1 → 分岐しない: PC=$8002
        let mut bus = test_bus(&[0x90, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.status |= CARRY;
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn bcc_branch_backward() {
        // C=0, offset=0xFC(-4) → PC=$8002+(-4)=$7FFE
        let mut bus = test_bus(&[0x90, 0xFC]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x7FFE);
    }

    #[test]
    fn bcs_branch_taken() {
        let mut bus = test_bus(&[0xB0, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.status |= CARRY;
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bcs_branch_not_taken() {
        let mut bus = test_bus(&[0xB0, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn beq_branch_taken() {
        let mut bus = test_bus(&[0xF0, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.status |= ZERO;
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn beq_branch_not_taken() {
        let mut bus = test_bus(&[0xF0, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn bit_zero_flag() {
        // A & M == 0 → Z=1
        let mut bus = test_bus(&[0x24, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x0F;
        bus.ram[0x10] = 0xF0;
        cpu.step(&mut bus);

        assert_ne!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
        assert_ne!(cpu.status & OVERFLOW, 0);
    }

    #[test]
    fn bit_not_zero() {
        // A & M != 0 → Z=0
        let mut bus = test_bus(&[0x24, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0xFF;
        bus.ram[0x10] = 0x3F;
        cpu.step(&mut bus);

        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
        assert_eq!(cpu.status & OVERFLOW, 0);
    }

    #[test]
    fn bit_flags_from_memory() {
        // N,V はメモリの値のbit7,bit6から取る（Aに関係なく）
        let mut bus = test_bus(&[0x24, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0xC0;
        bus.ram[0x10] = 0xC0;
        cpu.step(&mut bus);

        assert_eq!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
        assert_ne!(cpu.status & OVERFLOW, 0);
    }

    #[test]
    fn bmi_branch_taken() {
        let mut bus = test_bus(&[0x30, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.status |= NEGATIVE;
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bmi_branch_not_taken() {
        let mut bus = test_bus(&[0x30, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn bne_branch_taken() {
        let mut bus = test_bus(&[0xD0, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bne_branch_not_taken() {
        let mut bus = test_bus(&[0xD0, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.status |= ZERO;
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn bpl_branch_taken() {
        let mut bus = test_bus(&[0x10, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bpl_branch_not_taken() {
        let mut bus = test_bus(&[0x10, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.status |= NEGATIVE;
        cpu.step(&mut bus);

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
        let mut bus = Bus::new(Cartridge::new_test(prg_rom));
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);

        let old_sp = cpu.sp;
        cpu.step(&mut bus);

        // PCがIRQベクタに設定される
        assert_eq!(cpu.pc, 0xC000);
        // スタックにPC(2バイト)+status(1バイト)がpushされる
        assert_eq!(cpu.sp, old_sp.wrapping_sub(3));
        // Iフラグがセットされる
        assert_ne!(cpu.status & INTERRUPT_DISABLE, 0);
        // スタック上のstatusにBフラグが立っている
        let pushed_status = bus.ram[0x0100 + cpu.sp.wrapping_add(1) as usize];
        assert_ne!(pushed_status & BREAK, 0);
        assert_ne!(pushed_status & UNUSED, 0);
    }

    #[test]
    fn bvc_branch_taken() {
        let mut bus = test_bus(&[0x50, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bvc_branch_not_taken() {
        let mut bus = test_bus(&[0x50, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.status |= OVERFLOW;
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn bvs_branch_taken() {
        let mut bus = test_bus(&[0x70, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.status |= OVERFLOW;
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8006);
    }

    #[test]
    fn bvs_branch_not_taken() {
        let mut bus = test_bus(&[0x70, 0x04]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8002);
    }

    #[test]
    fn clc() {
        let mut bus = test_bus(&[0x18]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.status |= CARRY;
        cpu.step(&mut bus);

        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn cld() {
        let mut bus = test_bus(&[0xD8]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.status |= DECIMAL;
        cpu.step(&mut bus);

        assert_eq!(cpu.status & DECIMAL, 0);
    }

    #[test]
    fn cli() {
        let mut bus = test_bus(&[0x58]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        // resetでIフラグはセット済み
        assert_ne!(cpu.status & INTERRUPT_DISABLE, 0);
        cpu.step(&mut bus);

        assert_eq!(cpu.status & INTERRUPT_DISABLE, 0);
    }

    #[test]
    fn clv() {
        let mut bus = test_bus(&[0xB8]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.status |= OVERFLOW;
        cpu.step(&mut bus);

        assert_eq!(cpu.status & OVERFLOW, 0);
    }

    #[test]
    fn cmp_equal() {
        let mut bus = test_bus(&[0xC9, 0x42]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x42;
        cpu.step(&mut bus);

        assert_ne!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & CARRY, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn cmp_greater() {
        let mut bus = test_bus(&[0xC9, 0x20]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x42;
        cpu.step(&mut bus);

        assert_eq!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn cmp_less() {
        let mut bus = test_bus(&[0xC9, 0x50]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x42;
        cpu.step(&mut bus);

        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn cpx_equal() {
        let mut bus = test_bus(&[0xE0, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x10;
        cpu.step(&mut bus);

        assert_ne!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn cpx_less() {
        let mut bus = test_bus(&[0xE0, 0x20]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x10;
        cpu.step(&mut bus);

        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn cpy_equal() {
        let mut bus = test_bus(&[0xC0, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.y = 0x10;
        cpu.step(&mut bus);

        assert_ne!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn cpy_less() {
        let mut bus = test_bus(&[0xC0, 0x20]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.y = 0x10;
        cpu.step(&mut bus);

        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn dec() {
        let mut bus = test_bus(&[0xC6, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x10] = 0x05;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0x04);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn dec_zero() {
        let mut bus = test_bus(&[0xC6, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x10] = 0x01;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn dec_wrap() {
        let mut bus = test_bus(&[0xC6, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x10] = 0x00;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0xFF);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn dex() {
        let mut bus = test_bus(&[0xCA]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x05;
        cpu.step(&mut bus);

        assert_eq!(cpu.x, 0x04);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn dex_zero() {
        let mut bus = test_bus(&[0xCA]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x01;
        cpu.step(&mut bus);

        assert_eq!(cpu.x, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn dey() {
        let mut bus = test_bus(&[0x88]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.y = 0x05;
        cpu.step(&mut bus);

        assert_eq!(cpu.y, 0x04);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn dey_zero() {
        let mut bus = test_bus(&[0x88]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.y = 0x01;
        cpu.step(&mut bus);

        assert_eq!(cpu.y, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn eor_immediate() {
        let mut bus = test_bus(&[0x49, 0xFF]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0xAA;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x55);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn eor_zero_flag() {
        let mut bus = test_bus(&[0x49, 0xFF]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0xFF;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn eor_negative_flag() {
        let mut bus = test_bus(&[0x49, 0x0F]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x8F;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x80);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn inc() {
        let mut bus = test_bus(&[0xE6, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x10] = 0x05;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0x06);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn inc_wrap() {
        let mut bus = test_bus(&[0xE6, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x10] = 0xFF;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn inc_negative() {
        let mut bus = test_bus(&[0xE6, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x10] = 0x7F;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0x80);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn inx() {
        let mut bus = test_bus(&[0xE8]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x05;
        cpu.step(&mut bus);

        assert_eq!(cpu.x, 0x06);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn inx_wrap() {
        let mut bus = test_bus(&[0xE8]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0xFF;
        cpu.step(&mut bus);

        assert_eq!(cpu.x, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn iny() {
        let mut bus = test_bus(&[0xC8]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.y = 0x05;
        cpu.step(&mut bus);

        assert_eq!(cpu.y, 0x06);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn iny_wrap() {
        let mut bus = test_bus(&[0xC8]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.y = 0xFF;
        cpu.step(&mut bus);

        assert_eq!(cpu.y, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn jmp_absolute() {
        let mut bus = test_bus(&[0x4C, 0x00, 0x90]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x9000);
    }

    #[test]
    fn jmp_indirect() {
        // $0200-$0201 に $9000 を格納
        let mut bus = test_bus(&[0x6C, 0x00, 0x02]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x0200] = 0x00;
        bus.ram[0x0201] = 0x90;
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x9000);
    }

    #[test]
    fn jmp_indirect_page_boundary_bug() {
        // $02FF-$0200 (not $0300) から読む6502バグの再現
        let mut bus = test_bus(&[0x6C, 0xFF, 0x02]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x02FF] = 0x40;
        bus.ram[0x0200] = 0x80; // $0300ではなく$0200から読む
        bus.ram[0x0300] = 0xFF; // これは使われない
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x8040);
    }

    #[test]
    fn jsr() {
        // JSR $9000 at $8000
        let mut bus = test_bus(&[0x20, 0x00, 0x90]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        let old_sp = cpu.sp;
        cpu.step(&mut bus);

        assert_eq!(cpu.pc, 0x9000);
        assert_eq!(cpu.sp, old_sp.wrapping_sub(2));
        // スタックにはPC-1 ($8002) が格納される
        let hi = bus.ram[0x0100 + old_sp as usize];
        let lo = bus.ram[0x0100 + old_sp.wrapping_sub(1) as usize];
        assert_eq!((hi as u16) << 8 | lo as u16, 0x8002);
    }

    #[test]
    fn ldx_immediate() {
        let mut bus = test_bus(&[0xA2, 0x42]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.x, 0x42);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn ldx_zero_page_y() {
        let mut bus = test_bus(&[0xB6, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.y = 0x05;
        bus.ram[0x15] = 0x99;
        cpu.step(&mut bus);

        assert_eq!(cpu.x, 0x99);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn ldy_immediate() {
        let mut bus = test_bus(&[0xA0, 0x42]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus);

        assert_eq!(cpu.y, 0x42);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn ldy_zero_page_x() {
        let mut bus = test_bus(&[0xB4, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x05;
        bus.ram[0x15] = 0x99;
        cpu.step(&mut bus);

        assert_eq!(cpu.y, 0x99);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn lsr_accumulator() {
        let mut bus = test_bus(&[0x4A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x4A;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x25);
        assert_eq!(cpu.status & CARRY, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn lsr_accumulator_carry() {
        let mut bus = test_bus(&[0x4A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x03;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x01);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn lsr_accumulator_zero() {
        let mut bus = test_bus(&[0x4A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x01;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn lsr_memory() {
        let mut bus = test_bus(&[0x46, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x10] = 0x81;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0x40);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn ora_immediate() {
        let mut bus = test_bus(&[0x09, 0xF0]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x0F;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0xFF);
        assert_ne!(cpu.status & NEGATIVE, 0);
        assert_eq!(cpu.status & ZERO, 0);
    }

    #[test]
    fn ora_zero_flag() {
        let mut bus = test_bus(&[0x09, 0x00]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x00;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn pha() {
        let mut bus = test_bus(&[0x48]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x42;
        let old_sp = cpu.sp;
        cpu.step(&mut bus);

        assert_eq!(cpu.sp, old_sp.wrapping_sub(1));
        assert_eq!(bus.ram[0x0100 + old_sp as usize], 0x42);
    }

    #[test]
    fn php() {
        let mut bus = test_bus(&[0x08]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.status = CARRY | NEGATIVE | UNUSED | INTERRUPT_DISABLE;
        let old_sp = cpu.sp;
        cpu.step(&mut bus);

        assert_eq!(cpu.sp, old_sp.wrapping_sub(1));
        let pushed = bus.ram[0x0100 + old_sp as usize];
        assert_ne!(pushed & BREAK, 0);
        assert_ne!(pushed & UNUSED, 0);
        assert_ne!(pushed & CARRY, 0);
        assert_ne!(pushed & NEGATIVE, 0);
    }

    #[test]
    fn pla() {
        // PHA then PLA
        let mut bus = test_bus(&[0x48, 0xA9, 0x00, 0x68]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x42;
        cpu.step(&mut bus); // PHA
        cpu.step(&mut bus); // LDA #$00
        assert_eq!(cpu.a, 0x00);
        cpu.step(&mut bus); // PLA

        assert_eq!(cpu.a, 0x42);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn pla_zero_flag() {
        let mut bus = test_bus(&[0x68]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0xFF;
        // スタックに0をpush
        cpu.push(&mut bus, 0x00);
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn plp() {
        let mut bus = test_bus(&[0x28]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.push(&mut bus, CARRY | NEGATIVE | BREAK);
        cpu.step(&mut bus);

        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
        // Bフラグは無視される
        assert_eq!(cpu.status & BREAK, 0);
        // Uフラグは常にセット
        assert_ne!(cpu.status & UNUSED, 0);
    }

    #[test]
    fn rol_accumulator_no_carry() {
        let mut bus = test_bus(&[0x2A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x25;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x4A);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn rol_accumulator_with_carry_in() {
        let mut bus = test_bus(&[0x2A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x25;
        cpu.status |= CARRY;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x4B);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn rol_accumulator_carry_out() {
        let mut bus = test_bus(&[0x2A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x80;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn rol_memory() {
        let mut bus = test_bus(&[0x26, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x10] = 0x85;
        cpu.status |= CARRY;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0x0B);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn ror_accumulator_no_carry() {
        let mut bus = test_bus(&[0x6A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x4A;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x25);
        assert_eq!(cpu.status & CARRY, 0);
    }

    #[test]
    fn ror_accumulator_with_carry_in() {
        let mut bus = test_bus(&[0x6A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x4A;
        cpu.status |= CARRY;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0xA5);
        assert_eq!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn ror_accumulator_carry_out() {
        let mut bus = test_bus(&[0x6A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x01;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & ZERO, 0);
    }

    #[test]
    fn ror_memory() {
        let mut bus = test_bus(&[0x66, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        bus.ram[0x10] = 0x03;
        cpu.status |= CARRY;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0x81);
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
        let mut bus = Bus::new(Cartridge::new_test(prg_rom));
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.status |= CARRY;

        cpu.step(&mut bus); // BRK
        assert_eq!(cpu.pc, 0xC000);

        cpu.step(&mut bus); // RTI
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
        let mut bus = Bus::new(Cartridge::new_test(prg_rom));
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);

        cpu.step(&mut bus); // JSR $8010
        assert_eq!(cpu.pc, 0x8010);

        cpu.step(&mut bus); // RTS
        assert_eq!(cpu.pc, 0x8003);
    }

    #[test]
    fn sbc_no_borrow() {
        // 0x50 - 0x10 (C=1, no borrow) = 0x40
        let mut bus = test_bus(&[0xE9, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x50;
        cpu.status |= CARRY;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x40);
        assert_ne!(cpu.status & CARRY, 0);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
        assert_eq!(cpu.status & OVERFLOW, 0);
    }

    #[test]
    fn sbc_with_borrow() {
        // 0x50 - 0x10 - 1(borrow, C=0) = 0x3F
        let mut bus = test_bus(&[0xE9, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x50;
        // C=0 (borrow)
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x3F);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn sbc_underflow() {
        // 0x00 - 0x01 (C=1) = 0xFF, C=0
        let mut bus = test_bus(&[0xE9, 0x01]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x00;
        cpu.status |= CARRY;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0xFF);
        assert_eq!(cpu.status & CARRY, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn sbc_overflow_positive() {
        // 0x50 - 0xB0 (C=1) → 正-負=負 → V=1
        let mut bus = test_bus(&[0xE9, 0xB0]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x50;
        cpu.status |= CARRY;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0xA0);
        assert_ne!(cpu.status & OVERFLOW, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn sbc_overflow_negative() {
        // 0xD0 - 0x70 (C=1) → 負-正=正 → V=1
        let mut bus = test_bus(&[0xE9, 0x70]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0xD0;
        cpu.status |= CARRY;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x60);
        assert_ne!(cpu.status & OVERFLOW, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn sbc_zero() {
        // 0x50 - 0x50 (C=1) = 0x00
        let mut bus = test_bus(&[0xE9, 0x50]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x50;
        cpu.status |= CARRY;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn sec() {
        let mut bus = test_bus(&[0x38]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        assert_eq!(cpu.status & CARRY, 0);
        cpu.step(&mut bus);

        assert_ne!(cpu.status & CARRY, 0);
    }

    #[test]
    fn sed() {
        let mut bus = test_bus(&[0xF8]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        assert_eq!(cpu.status & DECIMAL, 0);
        cpu.step(&mut bus);

        assert_ne!(cpu.status & DECIMAL, 0);
    }

    #[test]
    fn sei() {
        let mut bus = test_bus(&[0x58, 0x78]); // CLI then SEI
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.step(&mut bus); // CLI
        assert_eq!(cpu.status & INTERRUPT_DISABLE, 0);
        cpu.step(&mut bus); // SEI

        assert_ne!(cpu.status & INTERRUPT_DISABLE, 0);
    }

    #[test]
    fn sta_zero_page() {
        let mut bus = test_bus(&[0x85, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x42;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0x42);
    }

    #[test]
    fn sta_absolute() {
        let mut bus = test_bus(&[0x8D, 0x00, 0x02]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x99;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x0200], 0x99);
    }

    #[test]
    fn stx_zero_page() {
        let mut bus = test_bus(&[0x86, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x42;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0x42);
    }

    #[test]
    fn sty_zero_page() {
        let mut bus = test_bus(&[0x84, 0x10]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.y = 0x42;
        cpu.step(&mut bus);

        assert_eq!(bus.ram[0x10], 0x42);
    }

    #[test]
    fn tax() {
        let mut bus = test_bus(&[0xAA]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x42;
        cpu.step(&mut bus);

        assert_eq!(cpu.x, 0x42);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn tax_zero() {
        let mut bus = test_bus(&[0xAA]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x00;
        cpu.step(&mut bus);

        assert_eq!(cpu.x, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn tax_negative() {
        let mut bus = test_bus(&[0xAA]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x80;
        cpu.step(&mut bus);

        assert_eq!(cpu.x, 0x80);
        assert_eq!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn tay() {
        let mut bus = test_bus(&[0xA8]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x42;
        cpu.step(&mut bus);

        assert_eq!(cpu.y, 0x42);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn tay_zero() {
        let mut bus = test_bus(&[0xA8]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x00;
        cpu.step(&mut bus);

        assert_eq!(cpu.y, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn tay_negative() {
        let mut bus = test_bus(&[0xA8]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.a = 0x80;
        cpu.step(&mut bus);

        assert_eq!(cpu.y, 0x80);
        assert_eq!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn tsx() {
        let mut bus = test_bus(&[0xBA]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.sp = 0x42;
        cpu.step(&mut bus);

        assert_eq!(cpu.x, 0x42);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn tsx_zero() {
        let mut bus = test_bus(&[0xBA]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.sp = 0x00;
        cpu.step(&mut bus);

        assert_eq!(cpu.x, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn tsx_negative() {
        let mut bus = test_bus(&[0xBA]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.sp = 0xFD;
        cpu.step(&mut bus);

        assert_eq!(cpu.x, 0xFD);
        assert_eq!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn txa() {
        let mut bus = test_bus(&[0x8A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x42;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x42);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn txa_zero() {
        let mut bus = test_bus(&[0x8A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x00;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn txa_negative() {
        let mut bus = test_bus(&[0x8A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x80;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x80);
        assert_eq!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn txs() {
        let mut bus = test_bus(&[0x9A]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.x = 0x42;
        let status_before = cpu.status;
        cpu.step(&mut bus);

        assert_eq!(cpu.sp, 0x42);
        assert_eq!(cpu.status, status_before);
    }

    #[test]
    fn tya() {
        let mut bus = test_bus(&[0x98]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.y = 0x42;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x42);
        assert_eq!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn tya_zero() {
        let mut bus = test_bus(&[0x98]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.y = 0x00;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x00);
        assert_ne!(cpu.status & ZERO, 0);
        assert_eq!(cpu.status & NEGATIVE, 0);
    }

    #[test]
    fn tya_negative() {
        let mut bus = test_bus(&[0x98]);
        let mut cpu = Cpu::new();
        cpu.reset(&mut bus);
        cpu.y = 0x80;
        cpu.step(&mut bus);

        assert_eq!(cpu.a, 0x80);
        assert_eq!(cpu.status & ZERO, 0);
        assert_ne!(cpu.status & NEGATIVE, 0);
    }
}
