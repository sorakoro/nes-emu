mod bus;
mod cart;
mod controller;
mod cpu;
mod ppu;

use crate::bus::Bus;
use crate::cart::Cartridge;
use crate::cpu::Cpu;
use crate::ppu::{SCREEN_HEIGHT, SCREEN_WIDTH};
use sdl2::event::Event;
use sdl2::keyboard::{Keycode, Scancode};
use sdl2::pixels::PixelFormatEnum;
use sdl2::rect::Rect;
use std::{env, fs, path::Path};

const OVERSCAN_TOP: u32 = 9;
const OVERSCAN_BOTTOM: u32 = 7;
const VISIBLE_HEIGHT: u32 = SCREEN_HEIGHT as u32 - OVERSCAN_TOP - OVERSCAN_BOTTOM;

fn main() {
    run().unwrap();
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().collect();
    let path = args.get(1).ok_or("ROM path not specified".to_string())?;
    let raw = load_rom(Path::new(path))?;

    let cartridge = Cartridge::new(&raw)?;
    let mut bus = Bus::new(cartridge);
    let mut cpu = Cpu::new();
    cpu.reset(&mut bus);

    let sdl_context = sdl2::init()?;
    let video_subsystem = sdl_context.video()?;

    let scale = 3u32;
    let window = video_subsystem
        .window(
            "NES Emulator",
            SCREEN_WIDTH as u32 * scale,
            VISIBLE_HEIGHT * scale,
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
        if cpu.step(&mut bus) {
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

            let keys = event_pump.keyboard_state();
            let mut buttons: u8 = 0;
            if keys.is_scancode_pressed(Scancode::Z) { buttons |= 1 << 0; } // A
            if keys.is_scancode_pressed(Scancode::X) { buttons |= 1 << 1; } // B
            if keys.is_scancode_pressed(Scancode::RShift) { buttons |= 1 << 2; } // Select
            if keys.is_scancode_pressed(Scancode::Return) { buttons |= 1 << 3; } // Start
            if keys.is_scancode_pressed(Scancode::Up) { buttons |= 1 << 4; }
            if keys.is_scancode_pressed(Scancode::Down) { buttons |= 1 << 5; }
            if keys.is_scancode_pressed(Scancode::Left) { buttons |= 1 << 6; }
            if keys.is_scancode_pressed(Scancode::Right) { buttons |= 1 << 7; }
            bus.controller.update(buttons);

            texture
                .update(None, &bus.ppu.frame_buffer, SCREEN_WIDTH * 3)
                .map_err(|e| e.to_string())?;
            let src = Rect::new(0, OVERSCAN_TOP as i32, SCREEN_WIDTH as u32, VISIBLE_HEIGHT);
            canvas.copy(&texture, src, None)?;
            canvas.present();
        }
    }

    Ok(())
}

fn load_rom(path: &Path) -> Result<Vec<u8>, String> {
    fs::read(path).map_err(|_| "Failed to read ROM file".to_string())
}
