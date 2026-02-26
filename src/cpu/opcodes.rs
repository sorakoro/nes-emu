use AddressingMode::*;

#[derive(Clone, Copy)]
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
