const MINIMUM_DISTANCE_SCALE_UP: f32 = 0.65;
const MINIMUM_DISTANCE_SCALE_DOWN: f32 = 0.60;
const BUFFER_SIZE: u16 = 2;
const TOLERANCE_SCALE: f32 = 0.075;

pub const NUM_LAYERS: usize = 3;

// struct Position {
//     pub buffer: [u32; BUFFER_SIZE as usize],
//     position: u32,
//     buffer_pos: u16,
//     min_distance_up: u32,
//     min_distance_down: u32,
//     tolerance: u32,
//     pub bit_pos: [u8; NUM_LAYERS], // Num represents pos to toggle in nkro report. Index indicates layer
//     is_pressed: bool,
//     wooting: bool,
// }
//
// impl Position {
//     pub const fn default() -> Self {
//         Self {
//             buffer: [0; BUFFER_SIZE as usize],
//             position: 0,
//             min_distance_up: 0,
//             min_distance_down: 0,
//             tolerance: 0,
//             buffer_pos: 0,
//             bit_pos: [0x00; NUM_LAYERS],
//             is_pressed: false,
//             wooting: false,
//         }
//     }
//     pub fn new(bit_pos: [u8; NUM_LAYERS], lowest_point: f32, highest_point: f32) -> Self {
//         Self {
//             buffer: [0; BUFFER_SIZE as usize],
//             position: (lowest_point + ((highest_point - lowest_point) * MINIMUM_DISTANCE_SCALE_UP))
//                 as u32,
//             min_distance_up: (lowest_point
//                 + (highest_point - lowest_point) * MINIMUM_DISTANCE_SCALE_UP)
//                 as u32,
//             min_distance_down: (lowest_point
//                 + (highest_point - lowest_point) * MINIMUM_DISTANCE_SCALE_DOWN)
//                 as u32,
//             tolerance: ((highest_point - lowest_point) * TOLERANCE_SCALE) as u32,
//             buffer_pos: 0,
//             bit_pos,
//             is_pressed: false,
//             wooting: false,
//         }
//     }
//
//     pub fn update_buf(&mut self, pos: u16) {
//         self.buffer[self.buffer_pos as usize] = pos as u32;
//         self.buffer_pos = (self.buffer_pos + 1) % BUFFER_SIZE;
//         let mut sum = 0;
//         for buf in self.buffer {
//             sum += buf;
//         }
//         let avg = sum / BUFFER_SIZE as u32;
//         if avg > self.min_distance_up {
//             self.position = avg;
//             self.wooting = false;
//             self.is_pressed = false;
//         } else if avg < self.position - self.tolerance
//             || (avg <= self.min_distance_down && !self.wooting)
//         {
//             self.position = avg;
//             self.wooting = true;
//             self.is_pressed = true;
//         } else if avg > self.position + self.tolerance {
//             self.position = avg;
//             self.is_pressed = false;
//         }
//     }
//
//     pub fn is_pressed(&self) -> bool {
//         self.is_pressed
//     }
//
//     pub fn get_buf(&self) -> u16 {
//         let mut sum = 0;
//         for buf in self.buffer {
//             sum += buf as u16;
//         }
//         sum / BUFFER_SIZE
//     }
// }

#[derive(Copy, Clone)]
pub enum ScanCodeType {
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
    pub scan_code_type: [ScanCodeType; NUM_LAYERS],
    pub current_layer: i8, // When a key is held, it will hold it's layer even when the a different
    // layer key is pressed. This variable will keep track of that layer
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
            scan_code_type: [ScanCodeType::Letter; NUM_LAYERS],
            current_layer: -1,
            was_pressed: false,
            time: 0,
            double_state: false,
            hold: false,
        }
    }
    pub fn new(
        bit_pos: [u8; NUM_LAYERS],
        scan_code_type: [ScanCodeType; NUM_LAYERS],
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

    pub fn set_key(&mut self, scan_code: u8, layer: usize, scan_type: ScanCodeType) {
        self.bit_pos[layer] = scan_code;
        self.scan_code_type[layer] = scan_type;
    }

    // pub fn double_is_pressed(&mut self) -> bool {
    //     if self.hold {
    //         true
    //     } else if self.time > DOUBLE_TIME_BUFFER {
    //         if !self.is_pressed {
    //             self.double_state = false;
    //             self.time = 0;
    //             self.was_pressed = false;
    //             false
    //         } else {
    //             true
    //         }
    //     } else if self.double_state {
    //         if self.is_pressed {
    //             self.hold = true;
    //             self.double_state = false;
    //             self.time = 0;
    //             self.was_pressed = false;
    //             true
    //         } else {
    //             self.time += 1;
    //             false
    //         }
    //     } else {
    //         if self.was_pressed && !self.is_pressed {
    //             self.double_state = true;
    //             self.time = 1;
    //             self.was_pressed = false;
    //             false
    //         } else {
    //             self.was_pressed = self.is_pressed;
    //             self.is_pressed
    //         }
    //     }
    // }
}
