const MINIMUM_DISTANCE_SCALE_UP: f32 = 0.90;
const MINIMUM_DISTANCE_SCALE_DOWN: f32 = 0.85;
const BUFFER_SIZE: u16 = 2;
const TOLERANCE_SCALE: f32 = 0.075;

pub const NUM_LAYERS: usize = 3;

#[derive(Copy, Clone)]
pub enum KeyType {
    Letter,
    Modifier,
    Layer,
}

#[derive(Copy, Clone)]
pub struct Key {
    pub buffer: [u32; BUFFER_SIZE as usize],
    position: u32,
    buffer_pos: u16,
    min_distance_up: u32,
    min_distance_down: u32,
    tolerance: u32,
    pub bit_pos: [u8; NUM_LAYERS], // Num represents pos to toggle in nkro report. Index indicates layer
    is_pressed: bool,
    wooting: bool,
    pub key_type: KeyType,
    pub current_layer: i8, // When a key is held, it will hold it's layer even when the a different
                           // layer key is pressed. This variable will keep track of that layer
}

impl Key {
    pub const fn default() -> Self {
        Self {
            buffer: [0; BUFFER_SIZE as usize],
            position: 0,
            min_distance_up: 0,
            min_distance_down: 0,
            tolerance: 0,
            buffer_pos: 0,
            bit_pos: [0x00; NUM_LAYERS],
            is_pressed: false,
            wooting: false,
            key_type: KeyType::Letter,
            current_layer: -1,
        }
    }
    pub(crate) fn new(
        bit_pos: [u8; NUM_LAYERS],
        key_type: KeyType,
        lowest_point: f32,
        highest_point: f32,
    ) -> Self {
        Self {
            buffer: [0; BUFFER_SIZE as usize],
            position: (lowest_point + ((highest_point - lowest_point) * MINIMUM_DISTANCE_SCALE_UP))
                as u32,
            min_distance_up: (lowest_point
                + (highest_point - lowest_point) * MINIMUM_DISTANCE_SCALE_UP)
                as u32,
            min_distance_down: (lowest_point
                + (highest_point - lowest_point) * MINIMUM_DISTANCE_SCALE_DOWN)
                as u32,
            tolerance: ((highest_point - lowest_point) * TOLERANCE_SCALE) as u32,
            buffer_pos: 0,
            bit_pos,
            is_pressed: false,
            wooting: false,
            key_type,
            current_layer: -1,
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
            self.position = avg;
            self.wooting = false;
            self.is_pressed = false;
        } else if avg < self.position - self.tolerance
            || (avg <= self.min_distance_down && !self.wooting)
        {
            self.position = avg;
            self.wooting = true;
            self.is_pressed = true;
        } else if avg > self.position + self.tolerance {
            self.position = avg;
            self.is_pressed = false;
        }
    }

    pub fn is_pressed(&self) -> bool {
        self.is_pressed
    }

    pub fn get_buf(&self) -> u16 {
        let mut sum = 0;
        for buf in self.buffer {
            sum += buf as u16;
        }
        sum / BUFFER_SIZE
    }
}
