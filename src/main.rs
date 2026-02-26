mod cart;
mod cpu;
mod ppu;

use crate::cart::Cartridge;
use crate::cpu::CPU;
use crate::ppu::PPU;
use std::{env, fs, path::Path};

fn main() {
    run().unwrap();
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().collect();
    let path = args.get(1).ok_or("ROM path not specified".to_string())?;
    let raw = load_rom(Path::new(path))?;

    let cart = Cartridge::new(&raw)?;
    let mut ppu = PPU::new(&cart);
    let mut cpu = CPU::new(&mut ppu, &cart);
    cpu.reset();

    loop {
        cpu.step();
    }
}

fn load_rom(path: &Path) -> Result<Vec<u8>, String> {
    fs::read(path).map_err(|_| format!("Failed to read ROM file").to_string())
}
