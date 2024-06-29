const MINIMUM_DISTANCE_SCALE_UP: f32 = 0.10;
const MINIMUM_DISTANCE_SCALE_DOWN: f32 = 0.15;
const BUFFER_SIZE: u16 = 1;
const TOLERANCE_SCALE: f32 = 0.075;

#[derive(Copy, Clone)]
pub struct Key {
    pub buffer: [u32; BUFFER_SIZE as usize],
    pos: u32,
    buffer_pos: u16,
    min_distance_up: u32,
    min_distance_down: u32,
    tolerance: u32,
    pub keycode: u8,
    is_pressed: bool,
    wooting: bool,
}

impl Key {
    pub const fn default() -> Self {
        Self {
            buffer: [0; BUFFER_SIZE as usize],
            pos: 0,
            min_distance_up: 0,
            min_distance_down: 0,
            tolerance: 0,
            buffer_pos: 0,
            keycode: 0x00,
            is_pressed: false,
            wooting: false,
        }
    }
    pub(crate) fn new(keycode: u8, lowest_point: f32, highest_point: f32) -> Self {
        Self {
            buffer: [0; BUFFER_SIZE as usize],
            pos: (lowest_point + ((highest_point - lowest_point) * MINIMUM_DISTANCE_SCALE_UP))
                as u32,
            min_distance_up: (lowest_point
                + (highest_point - lowest_point) * MINIMUM_DISTANCE_SCALE_UP)
                as u32,
            min_distance_down: (lowest_point
                + (highest_point - lowest_point) * MINIMUM_DISTANCE_SCALE_DOWN)
                as u32,
            tolerance: ((highest_point - lowest_point) * TOLERANCE_SCALE) as u32,
            buffer_pos: 0,
            keycode,
            is_pressed: false,
            wooting: false,
        }
    }

    pub fn update_buf(&mut self, pos: u16) {
        self.buffer[self.buffer_pos as usize] = pos as u32;
        self.buffer_pos = (self.buffer_pos + 1) % BUFFER_SIZE;
        let mut sum = 0;
        for buf in self.buffer {
            sum += buf;
        }
        let avg = sum / BUFFER_SIZE as u32;
        if avg > self.min_distance_up {
            self.pos = avg;
            self.wooting = false;
            self.is_pressed = false;
        } else if avg < self.pos - self.tolerance
            || (avg <= self.min_distance_down && !self.wooting)
        {
            self.pos = avg;
            self.wooting = true;
            self.is_pressed = true;
        } else if avg > self.pos + self.tolerance {
            self.pos = avg;
            self.is_pressed = false;
        }
    }

    pub fn get_key(&self) -> u8 {
        if self.is_pressed {
            self.keycode
        } else {
            0x00
        }
    }

    pub fn get_buf(&self) -> u16 {
        let mut sum = 0;
        for buf in self.buffer {
            sum += buf as u16;
        }
        sum / BUFFER_SIZE
    }
}
