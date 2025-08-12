use core::u16;

use embassy_rp::{
    adc::{Adc, Async, Channel},
    gpio::Output,
};
use embassy_sync::{
    blocking_mutex::{raw::ThreadModeRawMutex, ThreadModeMutex},
    channel::Receiver,
};
use embassy_time::Timer;

use crate::keys::NUM_KEYS;

pub const DEFAULT_HIGH: u32 = 1700;
pub const DEFAULT_LOW: u32 = 1400;
const DIF: f32 = (DEFAULT_HIGH - DEFAULT_LOW) as f32;
const DEFAULT_RELEASE_SCALE: f32 = 0.30;
const DEFAULT_ACTUATE_SCALE: f32 = 0.35;
const TOLERANCE_SCALE: f32 = 0.1;
const BUFFER_SIZE: usize = 1;

pub trait KeyState: Copy {
    const DEFAULT: Self;
    type Item;
    fn update_buf(&mut self, buf: Self::Item);

    fn get_buf(&self) -> Self::Item;

    fn is_pressed(&self) -> bool;

    fn is_analog(&self) -> bool;

    fn reset(&mut self);

    fn calibrate(&mut self, buf: Self::Item);

    fn setup(&mut self, buf: Self::Item) -> bool;
}

#[derive(Copy, Clone, Debug)]
pub struct DefaultSwitch {
    state: bool,
}

impl KeyState for DefaultSwitch {
    const DEFAULT: Self = Self { state: false };
    type Item = bool;
    fn update_buf(&mut self, buf: Self::Item) {
        self.state = buf;
    }

    fn is_pressed(&self) -> bool {
        self.state
    }

    fn is_analog(&self) -> bool {
        false
    }

    fn reset(&mut self) {
        self.state = false;
    }

    fn calibrate(&mut self, _: Self::Item) {}

    fn get_buf(&self) -> Self::Item {
        self.state
    }

    fn setup(&mut self, buf: Self::Item) -> bool {
        true
    }
}

// Makes hall effect switches act like a normal mechanical switch
#[derive(Copy, Clone, Default, Debug)]
pub struct DigitalPosition {
    buffer: [u16; BUFFER_SIZE as usize], // Take multiple readings to smooth out buffer
    buffer_pos: usize,
    release_point: u16,
    actuation_point: u16,
    lowest_point: u16,
    highest_point: u16,
    pressed: bool,
}

impl KeyState for DigitalPosition {
    type Item = u16;
    const DEFAULT: Self = Self {
        buffer: [0; BUFFER_SIZE as usize],
        buffer_pos: 0,
        release_point: (DEFAULT_HIGH - (DEFAULT_RELEASE_SCALE * DIF) as u32) as u16,
        actuation_point: (DEFAULT_HIGH - (DEFAULT_ACTUATE_SCALE * DIF) as u32) as u16,
        pressed: false,
        lowest_point: DEFAULT_LOW as u16,
        highest_point: DEFAULT_HIGH as u16,
    };

    // is_pressed is set like a normal mechanical switch, where if the buf
    // is higher than the release point, is_pressed is false, and if
    // the buf is lower than the acutation point, is_pressed is true
    fn update_buf(&mut self, pos: u16) {
        self.buffer[self.buffer_pos] = pos;
        self.buffer_pos = (self.buffer_pos + 1) % BUFFER_SIZE;
        let mut sum = 0;
        for buf in self.buffer {
            sum += buf;
        }
        let avg = sum / BUFFER_SIZE as u16;
        self.calibrate(avg);
        if avg <= self.actuation_point {
            self.pressed = true;
        } else if avg > self.release_point {
            self.pressed = false;
        }
    }

    fn is_pressed(&self) -> bool {
        self.pressed
    }

    fn get_buf(&self) -> u16 {
        let mut sum = 0;
        for buf in self.buffer {
            sum += buf as u16;
        }
        sum / BUFFER_SIZE as u16
    }

    // Keep calling this function with adc readings
    // until it returns true to calibrate keys
    fn setup(&mut self, reading: u16) -> bool {
        if self.buffer[0] == 0 || self.buffer_pos != 0 {
            self.buffer[self.buffer_pos] = reading;
            self.buffer_pos = (self.buffer_pos + 1) % BUFFER_SIZE as usize;
            false
        } else {
            let mut buf = 0;
            for num in self.buffer {
                buf += num;
            }
            let avg = buf / BUFFER_SIZE as u16;
            self.calibrate(avg);
            true
        }
    }

    fn calibrate(&mut self, buf: u16) {
        let mut changed = false;
        if self.highest_point < buf {
            self.highest_point = buf;
            changed = true;
        } else if self.lowest_point > buf {
            self.lowest_point = buf;
            changed = true;
        }

        if changed {
            let dif = (self.highest_point - self.lowest_point) as f32;
            self.release_point = self.highest_point - (DEFAULT_RELEASE_SCALE * dif) as u16;
            self.actuation_point = self.highest_point - (DEFAULT_ACTUATE_SCALE * dif) as u16;
        }
    }

    fn is_analog(&self) -> bool {
        true
    }

    fn reset(&mut self) {
        self.buffer.fill(self.highest_point);
        self.buffer_pos = 0;
        self.pressed = false;
    }
}

#[derive(Copy, Clone, Default, Debug)]
pub struct WootingPosition {
    buffer: [u16; BUFFER_SIZE as usize], // Take multiple readings to smooth out buffer
    buffer_pos: usize,
    release_point: u16,
    actuation_point: u16,
    lowest_point: u16,
    highest_point: u16,
    pressed: bool,
    last_pos: u16,
    wooting: bool,
    tolerance: u16,
}

impl KeyState for WootingPosition {
    type Item = u16;
    const DEFAULT: Self = Self {
        buffer: [0; BUFFER_SIZE as usize],
        last_pos: 0,
        buffer_pos: 0,
        release_point: (DEFAULT_HIGH - (DEFAULT_RELEASE_SCALE * DIF) as u32) as u16,
        actuation_point: (DEFAULT_HIGH - (DEFAULT_ACTUATE_SCALE * DIF) as u32) as u16,
        lowest_point: DEFAULT_LOW as u16,
        highest_point: DEFAULT_HIGH as u16,
        pressed: false,
        wooting: false,
        tolerance: (DIF * TOLERANCE_SCALE) as u16,
    };

    fn update_buf(&mut self, pos: u16) {
        self.buffer[self.buffer_pos as usize] = pos;
        self.buffer_pos = (self.buffer_pos + 1) % BUFFER_SIZE;
        let mut sum = 0;
        for buf in self.buffer {
            sum += buf;
        }
        let avg = sum / BUFFER_SIZE as u16;
        if avg > self.release_point {
            self.last_pos = avg;
            self.wooting = false;
            self.pressed = false;
            self.calibrate(avg);
        } else if avg < self.lowest_point {
            self.last_pos = avg;
            self.wooting = true;
            self.pressed = true;
            self.calibrate(avg);
        } else if avg < self.last_pos - self.tolerance
            || (avg <= self.actuation_point && !self.wooting)
        {
            self.last_pos = avg;
            self.wooting = true;
            self.pressed = true;
        } else if avg > self.last_pos + self.tolerance {
            self.last_pos = avg;
            self.pressed = false;
        }
    }

    fn calibrate(&mut self, buf: u16) {
        let mut changed = false;
        if self.highest_point < buf {
            self.highest_point = buf;
            changed = true;
        } else if self.lowest_point > buf {
            self.lowest_point = buf;
            changed = true;
        }

        if changed {
            let dif = (self.highest_point - self.lowest_point) as f32;
            self.release_point = self.highest_point - (DEFAULT_RELEASE_SCALE * dif) as u16;
            self.actuation_point = self.highest_point - (DEFAULT_ACTUATE_SCALE * dif) as u16;
            self.tolerance = (dif as f32 * TOLERANCE_SCALE) as u16;
        }
    }

    fn setup(&mut self, reading: u16) -> bool {
        if self.buffer[0] == 0 || self.buffer_pos != 0 {
            self.buffer[self.buffer_pos] = reading;
            self.buffer_pos = (self.buffer_pos + 1) % BUFFER_SIZE as usize;
            false
        } else {
            let mut buf = 0;
            for num in self.buffer {
                buf += num;
            }
            let avg = buf / BUFFER_SIZE as u16;
            self.calibrate(avg);
            true
        }
    }

    fn is_pressed(&self) -> bool {
        self.pressed
    }

    fn get_buf(&self) -> u16 {
        let mut sum = 0;
        for buf in self.buffer {
            sum += buf;
        }
        sum / BUFFER_SIZE as u16
    }

    fn is_analog(&self) -> bool {
        true
    }

    fn reset(&mut self) {
        self.buffer.fill(self.highest_point);
        self.pressed = false;
        self.buffer_pos = 0;
    }
}

#[derive(Copy, Clone)]
pub struct SlavePosition {
    state: u16,
    analog_reading: u16,
}
impl KeyState for SlavePosition {
    const DEFAULT: Self = Self {
        state: 0,
        analog_reading: u16::MAX,
    };
    type Item = u16;

    fn update_buf(&mut self, buf: Self::Item) {
        if buf > 1 {
            self.analog_reading = buf;
        } else {
            self.state = buf;
        }
    }

    fn get_buf(&self) -> Self::Item {
        self.analog_reading
    }

    fn is_pressed(&self) -> bool {
        self.state != 0
    }

    fn is_analog(&self) -> bool {
        true
    }

    fn reset(&mut self) {
        self.state = 0;
        self.analog_reading = u16::MAX;
    }

    fn calibrate(&mut self, _: Self::Item) {}

    fn setup(&mut self, _: Self::Item) -> bool {
        true
    }
}

#[derive(Copy, Clone)]
pub enum HeSwitch {
    Wooting(WootingPosition),
    Digital(DigitalPosition),
    Slave(SlavePosition),
}

impl KeyState for HeSwitch {
    const DEFAULT: Self = { Self::Wooting(WootingPosition::DEFAULT) };

    type Item = u16;

    fn update_buf(&mut self, buf: Self::Item) {
        match self {
            HeSwitch::Wooting(wp) => wp.update_buf(buf),
            HeSwitch::Digital(dp) => dp.update_buf(buf),
            HeSwitch::Slave(sp) => sp.update_buf(buf),
        }
    }

    fn get_buf(&self) -> Self::Item {
        match self {
            HeSwitch::Wooting(wp) => wp.get_buf(),
            HeSwitch::Digital(dp) => dp.get_buf(),
            HeSwitch::Slave(sp) => sp.get_buf(),
        }
    }

    fn is_pressed(&self) -> bool {
        match self {
            HeSwitch::Wooting(wp) => wp.is_pressed(),
            HeSwitch::Digital(dp) => dp.is_pressed(),
            HeSwitch::Slave(sp) => sp.is_pressed(),
        }
    }

    fn is_analog(&self) -> bool {
        true
    }

    fn reset(&mut self) {
        match self {
            HeSwitch::Wooting(wp) => wp.reset(),
            HeSwitch::Digital(dp) => dp.reset(),
            HeSwitch::Slave(sp) => sp.reset(),
        }
    }

    fn calibrate(&mut self, buf: Self::Item) {
        match self {
            HeSwitch::Wooting(wp) => wp.calibrate(buf),
            HeSwitch::Digital(dp) => dp.calibrate(buf),
            HeSwitch::Slave(sp) => sp.calibrate(buf),
        }
    }

    fn setup(&mut self, buf: Self::Item) -> bool {
        match self {
            HeSwitch::Wooting(wp) => wp.setup(buf),
            HeSwitch::Digital(dp) => dp.setup(buf),
            HeSwitch::Slave(sp) => sp.setup(buf),
        }
    }
}

pub trait KeySensors {
    type Item;
    fn update_positions<K: KeyState<Item = Self::Item>>(
        &mut self,
        positions: &mut [K],
    ) -> impl core::future::Future<Output = ()>;

    fn setup<K: KeyState<Item = Self::Item>>(
        &mut self,
        positions: &mut [K],
    ) -> impl core::future::Future<Output = ()>;
}

pub struct HallEffectSensors<'p, 'd, 'ch, const N: usize, const M: usize> {
    chans: [Channel<'p>; N],
    sel: [Output<'p>; M],
    adc: Adc<'d, Async>,
    slave_chan: Receiver<'ch, ThreadModeRawMutex, u32, 5>,
    order: [usize; NUM_KEYS / 2],
}

impl<'p, 'd, 'ch, const N: usize, const M: usize> HallEffectSensors<'p, 'd, 'ch, N, M> {
    pub fn new(
        chans: [Channel<'p>; N],
        sel: [Output<'p>; M],
        adc: Adc<'d, Async>,
        slave_chan: Receiver<'ch, ThreadModeRawMutex, u32, 5>,
        order: [usize; NUM_KEYS / 2],
    ) -> Self {
        Self {
            chans,
            sel,
            adc,
            slave_chan,
            order,
        }
    }
}

fn change_sel<'p>(pins: &mut [Output<'p>], sel: usize) {
    // For each pin, bit shift the sel with the respective index and mask that value to determine
    // if the pin should be high or low
    pins.iter_mut().enumerate().for_each(|(i, pin)| {
        if ((sel >> i) & 1) == 1 {
            pin.set_high();
        } else {
            pin.set_low();
        }
    });
}

impl<'p, 'd, 'ch, const N: usize, const M: usize> KeySensors
    for HallEffectSensors<'p, 'd, 'ch, N, M>
{
    type Item = u16;
    async fn update_positions<T: KeyState<Item = Self::Item>>(&mut self, positions: &mut [T]) {
        for (i, &pos) in self.order.iter().enumerate() {
            let chan = i % self.chans.len();
            if chan == 0 {
                let sel = i / self.chans.len();
                change_sel(&mut self.sel, sel);
                Timer::after_micros(1).await;
            }
            positions[pos].update_buf(self.adc.read(&mut self.chans[chan]).await.unwrap());
        }
        if let Ok(slave_rep) = self.slave_chan.try_receive() {
            let offset = NUM_KEYS / 2;
            for i in 0..(offset) {
                let val = (slave_rep >> i) & 1;
                positions[i + offset].update_buf(val as u16);
            }
        }
    }

    async fn setup<K: KeyState<Item = Self::Item>>(&mut self, positions: &mut [K]) {
        let mut setup = false;
        while !setup {
            setup = true;
            for (i, &pos) in self.order.iter().enumerate() {
                let chan = i % self.chans.len();
                if chan == 0 {
                    let sel = i / self.chans.len();
                    change_sel(&mut self.sel, sel);
                }
                let res = positions[pos].setup(self.adc.read(&mut self.chans[chan]).await.unwrap());
                // If any key isn't setup, the && will cause setup to be false leading to setup
                // being false after the loop
                setup = setup && res;
            }
        }
    }
}
