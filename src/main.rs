mod cart;
mod cpu;
mod ppu;

use crate::cart::Cartridge;
use crate::cpu::CPU;
use crate::ppu::{PPU, SCREEN_HEIGHT, SCREEN_WIDTH};
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::PixelFormatEnum;
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

    let sdl_context = sdl2::init()?;
    let video_subsystem = sdl_context.video()?;

    let scale = 3u32;
    let window = video_subsystem
        .window(
            "NES Emulator",
            SCREEN_WIDTH as u32 * scale,
            SCREEN_HEIGHT as u32 * scale,
        )
        .position_centered()
        .build()
        .map_err(|e| e.to_string())?;

    let mut canvas = window.into_canvas().build().map_err(|e| e.to_string())?;
    let texture_creator = canvas.texture_creator();
    let mut texture = texture_creator
        .create_texture_streaming(
            PixelFormatEnum::RGB24,
            SCREEN_WIDTH as u32,
            SCREEN_HEIGHT as u32,
        )
        .map_err(|e| e.to_string())?;

    let mut event_pump = sdl_context.event_pump()?;

    'running: loop {
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. }
                | Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => break 'running,
                _ => {}
            }
        }

        if cpu.step() {
            texture
                .update(None, &cpu.ppu.frame_buffer, SCREEN_WIDTH * 3)
                .map_err(|e| e.to_string())?;
            canvas.copy(&texture, None, None)?;
            canvas.present();
        }
    }

    Ok(())
}

fn load_rom(path: &Path) -> Result<Vec<u8>, String> {
    fs::read(path).map_err(|_| format!("Failed to read ROM file").to_string())
}
