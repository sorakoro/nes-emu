const LENGTH_TABLE: [u8; 32] = [
    10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14, 12, 16, 24, 18, 48, 20, 96,
    22, 192, 24, 72, 26, 16, 28, 32, 30,
];

const DUTY_TABLE: [[u8; 8]; 4] = [
    [0, 0, 0, 0, 0, 0, 0, 1], // 12.5%
    [0, 0, 0, 0, 0, 0, 1, 1], // 25%
    [0, 0, 0, 0, 1, 1, 1, 1], // 50%
    [1, 1, 1, 1, 1, 1, 0, 0], // 75% (inverted 25%)
];

const TRIANGLE_SEQUENCE: [u8; 32] = [
    15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
    12, 13, 14, 15,
];

const NOISE_PERIOD_TABLE: [u16; 16] = [
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068,
];

const DMC_RATE_TABLE: [u16; 16] = [
    428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54,
];

const CPU_FREQ: f64 = 1_789_773.0;
const SAMPLE_RATE: f64 = 44_100.0;

struct PulseChannel {
    // Registers
    duty: u8,
    length_halt: bool, // also envelope loop
    constant_volume: bool,
    volume_param: u8,

    sweep_enabled: bool,
    sweep_period: u8,
    sweep_negate: bool,
    sweep_shift: u8,
    sweep_reload: bool,
    sweep_divider: u8,

    timer_period: u16,
    length_counter: u8,

    // Internal state
    timer: u16,
    duty_pos: u8,

    envelope_start: bool,
    envelope_divider: u8,
    envelope_decay: u8,

    enabled: bool,
    is_pulse1: bool,
}

impl PulseChannel {
    fn new(is_pulse1: bool) -> Self {
        PulseChannel {
            duty: 0,
            length_halt: false,
            constant_volume: false,
            volume_param: 0,
            sweep_enabled: false,
            sweep_period: 0,
            sweep_negate: false,
            sweep_shift: 0,
            sweep_reload: false,
            sweep_divider: 0,
            timer_period: 0,
            length_counter: 0,
            timer: 0,
            duty_pos: 0,
            envelope_start: false,
            envelope_divider: 0,
            envelope_decay: 0,
            enabled: false,
            is_pulse1,
        }
    }

    fn write_reg(&mut self, reg: u8, value: u8) {
        match reg {
            0 => {
                self.duty = (value >> 6) & 0x03;
                self.length_halt = value & 0x20 != 0;
                self.constant_volume = value & 0x10 != 0;
                self.volume_param = value & 0x0F;
            }
            1 => {
                self.sweep_enabled = value & 0x80 != 0;
                self.sweep_period = (value >> 4) & 0x07;
                self.sweep_negate = value & 0x08 != 0;
                self.sweep_shift = value & 0x07;
                self.sweep_reload = true;
            }
            2 => {
                self.timer_period = (self.timer_period & 0xFF00) | value as u16;
            }
            3 => {
                self.timer_period = (self.timer_period & 0x00FF) | ((value as u16 & 0x07) << 8);
                if self.enabled {
                    self.length_counter = LENGTH_TABLE[(value >> 3) as usize];
                }
                self.duty_pos = 0;
                self.envelope_start = true;
            }
            _ => {}
        }
    }

    fn sweep_target_period(&self) -> u16 {
        let shift_result = self.timer_period >> self.sweep_shift;
        if self.sweep_negate {
            if self.is_pulse1 {
                // Pulse 1: ones' complement (subtract and then subtract 1 more)
                self.timer_period.wrapping_sub(shift_result).wrapping_sub(1)
            } else {
                // Pulse 2: two's complement (just subtract)
                self.timer_period.wrapping_sub(shift_result)
            }
        } else {
            self.timer_period.wrapping_add(shift_result)
        }
    }

    fn sweep_muting(&self) -> bool {
        self.timer_period < 8 || self.sweep_target_period() > 0x7FF
    }

    fn tick_timer(&mut self) {
        if self.timer == 0 {
            self.timer = self.timer_period;
            self.duty_pos = (self.duty_pos + 1) % 8;
        } else {
            self.timer -= 1;
        }
    }

    fn tick_envelope(&mut self) {
        if self.envelope_start {
            self.envelope_start = false;
            self.envelope_decay = 15;
            self.envelope_divider = self.volume_param;
        } else if self.envelope_divider == 0 {
            self.envelope_divider = self.volume_param;
            if self.envelope_decay > 0 {
                self.envelope_decay -= 1;
            } else if self.length_halt {
                self.envelope_decay = 15; // loop
            }
        } else {
            self.envelope_divider -= 1;
        }
    }

    fn tick_length_counter(&mut self) {
        if !self.length_halt && self.length_counter > 0 {
            self.length_counter -= 1;
        }
    }

    fn tick_sweep(&mut self) {
        let target = self.sweep_target_period();
        if self.sweep_divider == 0 && self.sweep_enabled && !self.sweep_muting() {
            self.timer_period = target;
        }
        if self.sweep_divider == 0 || self.sweep_reload {
            self.sweep_divider = self.sweep_period;
            self.sweep_reload = false;
        } else {
            self.sweep_divider -= 1;
        }
    }

    fn output(&self) -> u8 {
        if self.length_counter == 0 {
            return 0;
        }
        if self.sweep_muting() {
            return 0;
        }
        if DUTY_TABLE[self.duty as usize][self.duty_pos as usize] == 0 {
            return 0;
        }
        if self.constant_volume {
            self.volume_param
        } else {
            self.envelope_decay
        }
    }

    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        if !enabled {
            self.length_counter = 0;
        }
    }
}

struct TriangleChannel {
    control_flag: bool, // length counter halt / linear counter control
    linear_counter_reload: u8,
    linear_counter: u8,
    linear_counter_reload_flag: bool,
    timer_period: u16,
    timer: u16,
    length_counter: u8,
    sequencer_pos: u8,
    enabled: bool,
}

impl TriangleChannel {
    fn new() -> Self {
        TriangleChannel {
            control_flag: false,
            linear_counter_reload: 0,
            linear_counter: 0,
            linear_counter_reload_flag: false,
            timer_period: 0,
            timer: 0,
            length_counter: 0,
            sequencer_pos: 0,
            enabled: false,
        }
    }

    fn write_reg(&mut self, reg: u8, value: u8) {
        match reg {
            0 => {
                // $4008
                self.control_flag = value & 0x80 != 0;
                self.linear_counter_reload = value & 0x7F;
            }
            2 => {
                // $400A
                self.timer_period = (self.timer_period & 0xFF00) | value as u16;
            }
            3 => {
                // $400B
                self.timer_period = (self.timer_period & 0x00FF) | ((value as u16 & 0x07) << 8);
                if self.enabled {
                    self.length_counter = LENGTH_TABLE[(value >> 3) as usize];
                }
                self.linear_counter_reload_flag = true;
            }
            _ => {}
        }
    }

    fn tick_timer(&mut self) {
        if self.timer == 0 {
            self.timer = self.timer_period;
            if self.linear_counter > 0 && self.length_counter > 0 {
                self.sequencer_pos = (self.sequencer_pos + 1) % 32;
            }
        } else {
            self.timer -= 1;
        }
    }

    fn tick_linear_counter(&mut self) {
        if self.linear_counter_reload_flag {
            self.linear_counter = self.linear_counter_reload;
        } else if self.linear_counter > 0 {
            self.linear_counter -= 1;
        }
        if !self.control_flag {
            self.linear_counter_reload_flag = false;
        }
    }

    fn tick_length_counter(&mut self) {
        if !self.control_flag && self.length_counter > 0 {
            self.length_counter -= 1;
        }
    }

    fn output(&self) -> u8 {
        TRIANGLE_SEQUENCE[self.sequencer_pos as usize]
    }

    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        if !enabled {
            self.length_counter = 0;
        }
    }
}

struct NoiseChannel {
    length_halt: bool,
    constant_volume: bool,
    volume_param: u8,
    mode: bool,
    timer_period: u16,
    timer: u16,
    length_counter: u8,
    shift_register: u16,
    envelope_start: bool,
    envelope_divider: u8,
    envelope_decay: u8,
    enabled: bool,
}

impl NoiseChannel {
    fn new() -> Self {
        NoiseChannel {
            length_halt: false,
            constant_volume: false,
            volume_param: 0,
            mode: false,
            timer_period: 0,
            timer: 0,
            length_counter: 0,
            shift_register: 1,
            envelope_start: false,
            envelope_divider: 0,
            envelope_decay: 0,
            enabled: false,
        }
    }

    fn write_reg(&mut self, reg: u8, value: u8) {
        match reg {
            0 => {
                // $400C
                self.length_halt = value & 0x20 != 0;
                self.constant_volume = value & 0x10 != 0;
                self.volume_param = value & 0x0F;
            }
            2 => {
                // $400E
                self.mode = value & 0x80 != 0;
                self.timer_period = NOISE_PERIOD_TABLE[(value & 0x0F) as usize];
            }
            3 => {
                // $400F
                if self.enabled {
                    self.length_counter = LENGTH_TABLE[(value >> 3) as usize];
                }
                self.envelope_start = true;
            }
            _ => {}
        }
    }

    fn tick_timer(&mut self) {
        if self.timer == 0 {
            self.timer = self.timer_period;
            let bit = if self.mode { 6 } else { 1 };
            let feedback = (self.shift_register & 1) ^ ((self.shift_register >> bit) & 1);
            self.shift_register >>= 1;
            self.shift_register |= feedback << 14;
        } else {
            self.timer -= 1;
        }
    }

    fn tick_envelope(&mut self) {
        if self.envelope_start {
            self.envelope_start = false;
            self.envelope_decay = 15;
            self.envelope_divider = self.volume_param;
        } else if self.envelope_divider == 0 {
            self.envelope_divider = self.volume_param;
            if self.envelope_decay > 0 {
                self.envelope_decay -= 1;
            } else if self.length_halt {
                self.envelope_decay = 15;
            }
        } else {
            self.envelope_divider -= 1;
        }
    }

    fn tick_length_counter(&mut self) {
        if !self.length_halt && self.length_counter > 0 {
            self.length_counter -= 1;
        }
    }

    fn output(&self) -> u8 {
        if self.shift_register & 1 == 1 || self.length_counter == 0 {
            return 0;
        }
        if self.constant_volume {
            self.volume_param
        } else {
            self.envelope_decay
        }
    }

    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        if !enabled {
            self.length_counter = 0;
        }
    }
}

struct DmcChannel {
    irq_enabled: bool,
    loop_flag: bool,
    timer_period: u16,
    timer: u16,
    output_level: u8,
    shift_register: u8,
    bits_remaining: u8,
    silence_flag: bool,
    sample_buffer: Option<u8>,
    sample_addr_load: u16,
    sample_length_load: u16,
    sample_addr: u16,
    bytes_remaining: u16,
    sample_request: Option<u16>,
    irq_flag: bool,
    enabled: bool,
}

impl DmcChannel {
    fn new() -> Self {
        DmcChannel {
            irq_enabled: false,
            loop_flag: false,
            timer_period: DMC_RATE_TABLE[0],
            timer: DMC_RATE_TABLE[0],
            output_level: 0,
            shift_register: 0,
            bits_remaining: 8,
            silence_flag: true,
            sample_buffer: None,
            sample_addr_load: 0xC000,
            sample_length_load: 1,
            sample_addr: 0xC000,
            bytes_remaining: 0,
            sample_request: None,
            irq_flag: false,
            enabled: false,
        }
    }

    fn write_reg(&mut self, reg: u8, value: u8) {
        match reg {
            0 => {
                // $4010
                self.irq_enabled = value & 0x80 != 0;
                self.loop_flag = value & 0x40 != 0;
                self.timer_period = DMC_RATE_TABLE[(value & 0x0F) as usize];
                if !self.irq_enabled {
                    self.irq_flag = false;
                }
            }
            1 => {
                // $4011: direct load
                self.output_level = value & 0x7F;
            }
            2 => {
                // $4012
                self.sample_addr_load = 0xC000 + (value as u16 * 64);
            }
            3 => {
                // $4013
                self.sample_length_load = (value as u16 * 16) + 1;
            }
            _ => {}
        }
    }

    fn tick_timer(&mut self) {
        if self.timer == 0 {
            self.timer = self.timer_period;

            if !self.silence_flag {
                if self.shift_register & 1 == 1 {
                    if self.output_level <= 125 {
                        self.output_level += 2;
                    }
                } else if self.output_level >= 2 {
                    self.output_level -= 2;
                }
                self.shift_register >>= 1;
            }

            self.bits_remaining -= 1;
            if self.bits_remaining == 0 {
                self.bits_remaining = 8;
                if let Some(buf) = self.sample_buffer.take() {
                    self.shift_register = buf;
                    self.silence_flag = false;
                } else {
                    self.silence_flag = true;
                }
                // If buffer is now empty and there are more bytes, request one
                if self.sample_buffer.is_none() && self.bytes_remaining > 0 {
                    self.sample_request = Some(self.sample_addr);
                }
            }
        } else {
            self.timer -= 1;
        }
    }

    fn receive_sample(&mut self, byte: u8) {
        self.sample_buffer = Some(byte);
        self.sample_addr = self.sample_addr.wrapping_add(1);
        if self.sample_addr == 0x0000 {
            self.sample_addr = 0x8000;
        }
        self.bytes_remaining -= 1;
        if self.bytes_remaining == 0 {
            if self.loop_flag {
                self.restart();
            } else if self.irq_enabled {
                self.irq_flag = true;
            }
        }
    }

    fn restart(&mut self) {
        self.sample_addr = self.sample_addr_load;
        self.bytes_remaining = self.sample_length_load;
    }

    fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        if !enabled {
            self.bytes_remaining = 0;
            self.sample_request = None;
        } else if self.bytes_remaining == 0 {
            self.restart();
            if self.sample_buffer.is_none() {
                self.sample_request = Some(self.sample_addr);
            }
        }
    }

    fn output(&self) -> u8 {
        self.output_level
    }
}

pub struct Apu {
    pulse1: PulseChannel,
    pulse2: PulseChannel,
    triangle: TriangleChannel,
    noise: NoiseChannel,
    dmc: DmcChannel,

    frame_counter_mode: u8, // 0 = 4-step, 1 = 5-step
    frame_counter_cycle: u64,
    irq_inhibit: bool,

    timer_divider: bool,

    sample_cycle: f64,
    audio_buffer: Vec<f32>,
}

impl Apu {
    pub fn new() -> Self {
        Apu {
            pulse1: PulseChannel::new(true),
            pulse2: PulseChannel::new(false),
            triangle: TriangleChannel::new(),
            noise: NoiseChannel::new(),
            dmc: DmcChannel::new(),
            frame_counter_mode: 0,
            frame_counter_cycle: 0,
            irq_inhibit: false,
            timer_divider: false,
            sample_cycle: 0.0,
            audio_buffer: Vec::with_capacity(1024),
        }
    }

    pub fn tick(&mut self, cpu_cycles: u16) {
        for _ in 0..cpu_cycles {
            // Frame counter
            self.frame_counter_cycle += 1;
            self.clock_frame_counter();

            // Triangle and DMC timers tick every CPU cycle
            self.triangle.tick_timer();
            self.dmc.tick_timer();

            // Pulse timers tick every other CPU cycle
            self.timer_divider = !self.timer_divider;
            if self.timer_divider {
                self.pulse1.tick_timer();
                self.pulse2.tick_timer();
                self.noise.tick_timer();
            }

            // Downsample
            self.sample_cycle += SAMPLE_RATE;
            if self.sample_cycle >= CPU_FREQ {
                self.sample_cycle -= CPU_FREQ;
                self.audio_buffer.push(self.mix());
            }
        }
    }

    fn clock_frame_counter(&mut self) {
        let cycle = self.frame_counter_cycle;

        if self.frame_counter_mode == 0 {
            // 4-step sequence
            match cycle {
                3729 => self.quarter_frame(),
                7457 => self.half_frame(),
                11186 => self.quarter_frame(),
                14915 => {
                    self.half_frame();
                    self.frame_counter_cycle = 0;
                }
                _ => {}
            }
        } else {
            // 5-step sequence
            match cycle {
                3729 => self.quarter_frame(),
                7457 => self.half_frame(),
                11186 => self.quarter_frame(),
                18641 => {
                    self.half_frame();
                    self.frame_counter_cycle = 0;
                }
                _ => {}
            }
        }
    }

    fn quarter_frame(&mut self) {
        self.pulse1.tick_envelope();
        self.pulse2.tick_envelope();
        self.triangle.tick_linear_counter();
        self.noise.tick_envelope();
    }

    fn half_frame(&mut self) {
        self.quarter_frame();
        self.pulse1.tick_length_counter();
        self.pulse2.tick_length_counter();
        self.pulse1.tick_sweep();
        self.pulse2.tick_sweep();
        self.triangle.tick_length_counter();
        self.noise.tick_length_counter();
    }

    fn mix(&self) -> f32 {
        let p1 = self.pulse1.output() as f32;
        let p2 = self.pulse2.output() as f32;
        let pulse_sum = p1 + p2;
        let pulse_out = if pulse_sum == 0.0 {
            0.0
        } else {
            95.52 / (8128.0 / pulse_sum + 100.0)
        };

        let t = self.triangle.output() as f32;
        let n = self.noise.output() as f32;
        let d = self.dmc.output() as f32;
        let tnd_sum = t / 8227.0 + n / 12241.0 + d / 22638.0;
        let tnd_out = if tnd_sum == 0.0 {
            0.0
        } else {
            159.79 / (1.0 / tnd_sum + 100.0)
        };

        pulse_out + tnd_out
    }

    pub fn write_register(&mut self, addr: u16, value: u8) {
        match addr {
            0x4000..=0x4003 => self.pulse1.write_reg((addr & 0x03) as u8, value),
            0x4004..=0x4007 => self.pulse2.write_reg((addr & 0x03) as u8, value),
            0x4008 | 0x400A | 0x400B => {
                let reg = (addr - 0x4008) as u8;
                self.triangle.write_reg(reg, value);
            }
            0x400C | 0x400E | 0x400F => {
                let reg = (addr - 0x400C) as u8;
                self.noise.write_reg(reg, value);
            }
            0x4010..=0x4013 => {
                self.dmc.write_reg((addr - 0x4010) as u8, value);
            }
            0x4015 => {
                self.pulse1.set_enabled(value & 0x01 != 0);
                self.pulse2.set_enabled(value & 0x02 != 0);
                self.triangle.set_enabled(value & 0x04 != 0);
                self.noise.set_enabled(value & 0x08 != 0);
                self.dmc.set_enabled(value & 0x10 != 0);
                self.dmc.irq_flag = false;
            }
            0x4017 => {
                self.frame_counter_mode = (value >> 7) & 1;
                self.irq_inhibit = value & 0x40 != 0;
                self.frame_counter_cycle = 0;
                if self.frame_counter_mode == 1 {
                    self.half_frame();
                }
            }
            _ => {}
        }
    }

    pub fn read_status(&mut self) -> u8 {
        let mut status = 0u8;
        if self.pulse1.length_counter > 0 {
            status |= 0x01;
        }
        if self.pulse2.length_counter > 0 {
            status |= 0x02;
        }
        if self.triangle.length_counter > 0 {
            status |= 0x04;
        }
        if self.noise.length_counter > 0 {
            status |= 0x08;
        }
        if self.dmc.bytes_remaining > 0 {
            status |= 0x10;
        }
        if self.dmc.irq_flag {
            status |= 0x80;
        }
        self.dmc.irq_flag = false;
        status
    }

    pub fn dmc_sample_request(&mut self) -> Option<u16> {
        self.dmc.sample_request.take()
    }

    pub fn dmc_receive_sample(&mut self, byte: u8) {
        self.dmc.receive_sample(byte);
    }

    pub fn drain_audio_buffer(&mut self) -> Vec<f32> {
        std::mem::take(&mut self.audio_buffer)
    }
}
