use AddressingMode::*;

#[derive(Clone, Copy, PartialEq)]
pub enum AddressingMode {
    Implicit,
    Accumulator,
    Immediate,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    Indirect,
    IndirectX,
    IndirectY,
    Relative,
}

#[derive(Clone, Copy)]
pub struct Instruction {
    pub name: &'static str,
    pub mode: AddressingMode,
    pub cycles: u8,
    pub page_cross_cycle: bool,
}

impl Instruction {
    const fn new(
        name: &'static str,
        mode: AddressingMode,
        cycles: u8,
        page_cross_cycle: bool,
    ) -> Self {
        Instruction {
            name,
            mode,
            cycles,
            page_cross_cycle,
        }
    }
}

pub const OPCODES: [Option<Instruction>; 256] = {
    let mut opcodes = [None; 256];

    opcodes[0x69] = Some(Instruction::new("ADC", Immediate, 2, false));
    opcodes[0x65] = Some(Instruction::new("ADC", ZeroPage, 3, false));
    opcodes[0x75] = Some(Instruction::new("ADC", ZeroPageX, 4, false));
    opcodes[0x6D] = Some(Instruction::new("ADC", Absolute, 4, false));
    opcodes[0x7D] = Some(Instruction::new("ADC", AbsoluteX, 4, true));
    opcodes[0x79] = Some(Instruction::new("ADC", AbsoluteY, 4, true));
    opcodes[0x61] = Some(Instruction::new("ADC", IndirectX, 6, false));
    opcodes[0x71] = Some(Instruction::new("ADC", IndirectY, 5, true));

    opcodes[0x0A] = Some(Instruction::new("ASL", Accumulator, 2, false));
    opcodes[0x06] = Some(Instruction::new("ASL", ZeroPage, 5, false));
    opcodes[0x16] = Some(Instruction::new("ASL", ZeroPageX, 6, false));
    opcodes[0x0E] = Some(Instruction::new("ASL", Absolute, 6, false));
    opcodes[0x1E] = Some(Instruction::new("ASL", AbsoluteX, 7, false));

    opcodes[0x90] = Some(Instruction::new("BCC", Relative, 2, false));
    opcodes[0xB0] = Some(Instruction::new("BCS", Relative, 2, false));
    opcodes[0xF0] = Some(Instruction::new("BEQ", Relative, 2, false));

    opcodes[0x30] = Some(Instruction::new("BMI", Relative, 2, false));
    opcodes[0xD0] = Some(Instruction::new("BNE", Relative, 2, false));
    opcodes[0x10] = Some(Instruction::new("BPL", Relative, 2, false));
    opcodes[0x00] = Some(Instruction::new("BRK", Implicit, 7, false));
    opcodes[0x50] = Some(Instruction::new("BVC", Relative, 2, false));
    opcodes[0x70] = Some(Instruction::new("BVS", Relative, 2, false));
    opcodes[0x18] = Some(Instruction::new("CLC", Implicit, 2, false));
    opcodes[0xD8] = Some(Instruction::new("CLD", Implicit, 2, false));
    opcodes[0x58] = Some(Instruction::new("CLI", Implicit, 2, false));
    opcodes[0xB8] = Some(Instruction::new("CLV", Implicit, 2, false));

    opcodes[0xC9] = Some(Instruction::new("CMP", Immediate, 2, false));
    opcodes[0xC5] = Some(Instruction::new("CMP", ZeroPage, 3, false));
    opcodes[0xD5] = Some(Instruction::new("CMP", ZeroPageX, 4, false));
    opcodes[0xCD] = Some(Instruction::new("CMP", Absolute, 4, false));
    opcodes[0xDD] = Some(Instruction::new("CMP", AbsoluteX, 4, true));
    opcodes[0xD9] = Some(Instruction::new("CMP", AbsoluteY, 4, true));
    opcodes[0xC1] = Some(Instruction::new("CMP", IndirectX, 6, false));
    opcodes[0xD1] = Some(Instruction::new("CMP", IndirectY, 5, true));

    opcodes[0xE0] = Some(Instruction::new("CPX", Immediate, 2, false));
    opcodes[0xE4] = Some(Instruction::new("CPX", ZeroPage, 3, false));
    opcodes[0xEC] = Some(Instruction::new("CPX", Absolute, 4, false));

    opcodes[0xC0] = Some(Instruction::new("CPY", Immediate, 2, false));
    opcodes[0xC4] = Some(Instruction::new("CPY", ZeroPage, 3, false));
    opcodes[0xCC] = Some(Instruction::new("CPY", Absolute, 4, false));

    opcodes[0xC6] = Some(Instruction::new("DEC", ZeroPage, 5, false));
    opcodes[0xD6] = Some(Instruction::new("DEC", ZeroPageX, 6, false));
    opcodes[0xCE] = Some(Instruction::new("DEC", Absolute, 6, false));
    opcodes[0xDE] = Some(Instruction::new("DEC", AbsoluteX, 7, false));
    opcodes[0xCA] = Some(Instruction::new("DEX", Implicit, 2, false));
    opcodes[0x88] = Some(Instruction::new("DEY", Implicit, 2, false));

    opcodes[0xE6] = Some(Instruction::new("INC", ZeroPage, 5, false));
    opcodes[0xF6] = Some(Instruction::new("INC", ZeroPageX, 6, false));
    opcodes[0xEE] = Some(Instruction::new("INC", Absolute, 6, false));
    opcodes[0xFE] = Some(Instruction::new("INC", AbsoluteX, 7, false));
    opcodes[0xE8] = Some(Instruction::new("INX", Implicit, 2, false));
    opcodes[0xC8] = Some(Instruction::new("INY", Implicit, 2, false));

    opcodes[0x4C] = Some(Instruction::new("JMP", Absolute, 3, false));
    opcodes[0x6C] = Some(Instruction::new("JMP", Indirect, 5, false));
    opcodes[0x20] = Some(Instruction::new("JSR", Absolute, 6, false));

    opcodes[0xA2] = Some(Instruction::new("LDX", Immediate, 2, false));
    opcodes[0xA6] = Some(Instruction::new("LDX", ZeroPage, 3, false));
    opcodes[0xB6] = Some(Instruction::new("LDX", ZeroPageY, 4, false));
    opcodes[0xAE] = Some(Instruction::new("LDX", Absolute, 4, false));
    opcodes[0xBE] = Some(Instruction::new("LDX", AbsoluteY, 4, true));

    opcodes[0xA0] = Some(Instruction::new("LDY", Immediate, 2, false));
    opcodes[0xA4] = Some(Instruction::new("LDY", ZeroPage, 3, false));
    opcodes[0xB4] = Some(Instruction::new("LDY", ZeroPageX, 4, false));
    opcodes[0xAC] = Some(Instruction::new("LDY", Absolute, 4, false));
    opcodes[0xBC] = Some(Instruction::new("LDY", AbsoluteX, 4, true));

    opcodes[0x4A] = Some(Instruction::new("LSR", Accumulator, 2, false));
    opcodes[0x46] = Some(Instruction::new("LSR", ZeroPage, 5, false));
    opcodes[0x56] = Some(Instruction::new("LSR", ZeroPageX, 6, false));
    opcodes[0x4E] = Some(Instruction::new("LSR", Absolute, 6, false));
    opcodes[0x5E] = Some(Instruction::new("LSR", AbsoluteX, 7, false));

    opcodes[0xEA] = Some(Instruction::new("NOP", Implicit, 2, false));

    opcodes[0x09] = Some(Instruction::new("ORA", Immediate, 2, false));
    opcodes[0x05] = Some(Instruction::new("ORA", ZeroPage, 3, false));
    opcodes[0x15] = Some(Instruction::new("ORA", ZeroPageX, 4, false));
    opcodes[0x0D] = Some(Instruction::new("ORA", Absolute, 4, false));
    opcodes[0x1D] = Some(Instruction::new("ORA", AbsoluteX, 4, true));
    opcodes[0x19] = Some(Instruction::new("ORA", AbsoluteY, 4, true));
    opcodes[0x01] = Some(Instruction::new("ORA", IndirectX, 6, false));
    opcodes[0x11] = Some(Instruction::new("ORA", IndirectY, 5, true));

    opcodes[0x48] = Some(Instruction::new("PHA", Implicit, 3, false));
    opcodes[0x08] = Some(Instruction::new("PHP", Implicit, 3, false));
    opcodes[0x68] = Some(Instruction::new("PLA", Implicit, 4, false));

    opcodes[0x49] = Some(Instruction::new("EOR", Immediate, 2, false));
    opcodes[0x45] = Some(Instruction::new("EOR", ZeroPage, 3, false));
    opcodes[0x55] = Some(Instruction::new("EOR", ZeroPageX, 4, false));
    opcodes[0x4D] = Some(Instruction::new("EOR", Absolute, 4, false));
    opcodes[0x5D] = Some(Instruction::new("EOR", AbsoluteX, 4, true));
    opcodes[0x59] = Some(Instruction::new("EOR", AbsoluteY, 4, true));
    opcodes[0x41] = Some(Instruction::new("EOR", IndirectX, 6, false));
    opcodes[0x51] = Some(Instruction::new("EOR", IndirectY, 5, true));

    opcodes[0x24] = Some(Instruction::new("BIT", ZeroPage, 3, false));
    opcodes[0x2C] = Some(Instruction::new("BIT", Absolute, 4, false));

    opcodes[0x29] = Some(Instruction::new("AND", Immediate, 2, false));
    opcodes[0x25] = Some(Instruction::new("AND", ZeroPage, 3, false));
    opcodes[0x35] = Some(Instruction::new("AND", ZeroPageX, 4, false));
    opcodes[0x2D] = Some(Instruction::new("AND", Absolute, 4, false));
    opcodes[0x3D] = Some(Instruction::new("AND", AbsoluteX, 4, true));
    opcodes[0x39] = Some(Instruction::new("AND", AbsoluteY, 4, true));
    opcodes[0x21] = Some(Instruction::new("AND", IndirectX, 6, false));
    opcodes[0x31] = Some(Instruction::new("AND", IndirectY, 5, true));

    opcodes[0xA9] = Some(Instruction::new("LDA", Immediate, 2, false));
    opcodes[0xA5] = Some(Instruction::new("LDA", ZeroPage, 3, false));
    opcodes[0xB5] = Some(Instruction::new("LDA", ZeroPageX, 4, false));
    opcodes[0xAD] = Some(Instruction::new("LDA", Absolute, 4, false));
    opcodes[0xBD] = Some(Instruction::new("LDA", AbsoluteX, 4, true));
    opcodes[0xB9] = Some(Instruction::new("LDA", AbsoluteY, 4, true));
    opcodes[0xA1] = Some(Instruction::new("LDA", IndirectX, 6, false));
    opcodes[0xB1] = Some(Instruction::new("LDA", IndirectY, 5, true));

    opcodes
};
