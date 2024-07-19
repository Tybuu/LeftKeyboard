const MINIMUM_DISTANCE_SCALE_UP: f32 = 0.65;
const MINIMUM_DISTANCE_SCALE_DOWN: f32 = 0.60;
const BUFFER_SIZE: u16 = 2;
const TOLERANCE_SCALE: f32 = 0.075;
const DOUBLE_TIME_BUFFER: u16 = 250;

pub const NUM_LAYERS: usize = 3;

#[derive(Copy, Clone)]
pub enum ScanCodeType {
    Letter,
    Modifier,
    Layer,
}

#[derive(Copy, Clone)]
pub enum KeyType {
    Normal,
    DoubleToHold,
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
    pub scan_code_type: ScanCodeType,
    pub current_layer: i8, // When a key is held, it will hold it's layer even when the a different
    // layer key is pressed. This variable will keep track of that layer
    pub key_type: KeyType,
    was_pressed: bool,
    time: u16,
    double_state: bool,
    pub hold: bool,
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
            scan_code_type: ScanCodeType::Letter,
            current_layer: -1,
            key_type: KeyType::Normal,
            was_pressed: false,
            time: 0,
            double_state: false,
            hold: false,
        }
    }
    pub(crate) fn new(
        bit_pos: [u8; NUM_LAYERS],
        scan_code_type: ScanCodeType,
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
            scan_code_type,
            key_type,
            current_layer: -1,
            was_pressed: false,
            time: 0,
            double_state: false,
            hold: false,
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

    pub fn is_pressed(&mut self) -> bool {
        match self.key_type {
            KeyType::Normal => self.is_pressed,
            KeyType::DoubleToHold => self.double_is_pressed(),
        }
    }

    pub fn get_buf(&self) -> u16 {
        let mut sum = 0;
        for buf in self.buffer {
            sum += buf as u16;
        }
        sum / BUFFER_SIZE
    }

    pub fn double_is_pressed(&mut self) -> bool {
        if self.hold {
            true
        } else if self.time > DOUBLE_TIME_BUFFER {
            if !self.is_pressed {
                self.double_state = false;
                self.time = 0;
                self.was_pressed = false;
                false
            } else {
                true
            }
        } else if self.double_state {
            if self.is_pressed {
                self.hold = true;
                self.double_state = false;
                self.time = 0;
                self.was_pressed = false;
                true
            } else {
                self.time += 1;
                false
            }
        } else {
            if self.was_pressed && !self.is_pressed {
                self.double_state = true;
                self.time = 1;
                self.was_pressed = false;
                false
            } else {
                self.was_pressed = self.is_pressed;
                self.is_pressed
            }
        }
    }
}
