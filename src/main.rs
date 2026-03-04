mod apu;
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
use std::time::{Duration, Instant};
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
    let rom_path = Path::new(path);
    let raw = load_rom(rom_path)?;

    let sav_path = rom_path.with_extension("sav");
    let mut cartridge = Cartridge::new(&raw)?;
    if cartridge.has_battery()
        && let Ok(sav) = fs::read(&sav_path)
    {
        cartridge.load_sav(&sav);
        eprintln!("Loaded save: {}", sav_path.display());
    }

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

    // Audio setup
    let audio_subsystem = sdl_context.audio()?;
    let desired_spec = sdl2::audio::AudioSpecDesired {
        freq: Some(44_100),
        channels: Some(1),
        samples: Some(1024),
    };
    let audio_queue =
        sdl2::audio::AudioQueue::<f32>::open_queue(&audio_subsystem, None, &desired_spec)?;
    audio_queue.resume();

    const FRAME_DURATION: Duration = Duration::from_nanos(16_639_267); // 1/60.0988
    let mut next_frame = Instant::now();
    let mut frame_count: u32 = 0;
    let mut fps_timer = Instant::now();

    'running: loop {
        if cpu.step(&mut bus) {
            for event in event_pump.poll_iter() {
                match event {
                    Event::Quit { .. }
                    | Event::KeyDown {
                        keycode: Some(Keycode::Escape),
                        ..
                    } => break 'running,
                    Event::KeyDown {
                        keycode: Some(Keycode::T),
                        ..
                    } => {
                        cpu.trace = !cpu.trace;
                        eprintln!("CPU trace: {}", if cpu.trace { "ON" } else { "OFF" });
                    }
                    _ => {}
                }
            }

            let keys = event_pump.keyboard_state();
            let mut buttons: u8 = 0;
            if keys.is_scancode_pressed(Scancode::Z) {
                buttons |= 1 << 0;
            } // A
            if keys.is_scancode_pressed(Scancode::X) {
                buttons |= 1 << 1;
            } // B
            if keys.is_scancode_pressed(Scancode::RShift) {
                buttons |= 1 << 2;
            } // Select
            if keys.is_scancode_pressed(Scancode::Return) {
                buttons |= 1 << 3;
            } // Start
            if keys.is_scancode_pressed(Scancode::Up) {
                buttons |= 1 << 4;
            }
            if keys.is_scancode_pressed(Scancode::Down) {
                buttons |= 1 << 5;
            }
            if keys.is_scancode_pressed(Scancode::Left) {
                buttons |= 1 << 6;
            }
            if keys.is_scancode_pressed(Scancode::Right) {
                buttons |= 1 << 7;
            }
            bus.controller.update(buttons);

            texture
                .update(None, &bus.ppu.frame_buffer, SCREEN_WIDTH * 3)
                .map_err(|e| e.to_string())?;
            let src = Rect::new(0, OVERSCAN_TOP as i32, SCREEN_WIDTH as u32, VISIBLE_HEIGHT);
            canvas.copy(&texture, src, None)?;
            canvas.present();

            // Queue audio samples
            let samples = bus.apu.drain_audio_buffer();
            if !samples.is_empty() {
                // Prevent queue from growing too large (max ~4 frames worth)
                let max_queued = 44_100 / 15 * 4; // ~4 frames of samples in bytes
                if audio_queue.size() < max_queued as u32 {
                    audio_queue
                        .queue_audio(&samples)
                        .map_err(|e| e.to_string())?;
                }
            }

            // FPS display
            frame_count += 1;
            if fps_timer.elapsed() >= Duration::from_secs(1) {
                let fps = frame_count as f64 / fps_timer.elapsed().as_secs_f64();
                let title = format!("NES Emulator - {:.1} fps", fps);
                canvas.window_mut().set_title(&title).unwrap_or(());
                frame_count = 0;
                fps_timer = Instant::now();
            }

            // Frame rate control: sleep + spin-wait hybrid
            next_frame += FRAME_DURATION;
            let now = Instant::now();
            if next_frame > now {
                let remaining = next_frame - now;
                if remaining > Duration::from_millis(1) {
                    std::thread::sleep(remaining - Duration::from_millis(1));
                }
                while Instant::now() < next_frame {
                    std::hint::spin_loop();
                }
            }
        }
    }

    if let Some(data) = bus.cartridge.save_data() {
        fs::write(&sav_path, data).map_err(|e| e.to_string())?;
        eprintln!("Saved: {}", sav_path.display());
    }

    Ok(())
}

fn load_rom(path: &Path) -> Result<Vec<u8>, String> {
    fs::read(path).map_err(|_| "Failed to read ROM file".to_string())
}
