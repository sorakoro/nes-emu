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
