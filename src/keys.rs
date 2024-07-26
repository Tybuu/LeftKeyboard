use rp2040_hal::Timer;

use crate::codes::KeyboardCodes;

const MINIMUM_DISTANCE_SCALE_UP: f32 = 0.80;
const MINIMUM_DISTANCE_SCALE_DOWN: f32 = 0.75;
const BUFFER_SIZE: u16 = 2;
const TOLERANCE_SCALE: f32 = 0.075;

const HOLD_TIME: u64 = 300 * 1000;
const PRESS_TIME: u64 = 2000;
pub const NUM_LAYERS: usize = 10;

static mut TIMER: Option<Timer> = None;

pub fn intialize_timer(timer: Timer) {
    unsafe { TIMER = Some(timer) }
}
#[derive(Copy, Clone)]
struct DigitalPosition {
    buffer: [u32; BUFFER_SIZE as usize],
    buffer_pos: u16,
    min_distance_up: u32,
    min_distance_down: u32,
    is_pressed: bool,
}

impl DigitalPosition {
    pub const fn default() -> Self {
        Self {
            buffer: [0; BUFFER_SIZE as usize],
            min_distance_up: 0,
            min_distance_down: 0,
            buffer_pos: 0,
            is_pressed: false,
        }
    }
    pub fn new(lowest_point: f32, highest_point: f32) -> Self {
        Self {
            buffer: [0; BUFFER_SIZE as usize],
            min_distance_up: (lowest_point
                + (highest_point - lowest_point) * MINIMUM_DISTANCE_SCALE_UP)
                as u32,
            min_distance_down: (lowest_point
                + (highest_point - lowest_point) * MINIMUM_DISTANCE_SCALE_DOWN)
                as u32,
            buffer_pos: 0,
            is_pressed: false,
        }
    }

    fn update_buf(&mut self, pos: u16) {
        self.buffer[self.buffer_pos as usize] = pos as u32;
        self.buffer_pos = (self.buffer_pos + 1) % BUFFER_SIZE;
        let mut sum = 0;
        for buf in self.buffer {
            sum += buf;
        }
        let avg = sum / BUFFER_SIZE as u32;
        if self.is_pressed && avg > self.min_distance_up {
            self.is_pressed = false;
        } else if !self.is_pressed && avg < self.min_distance_down {
            self.is_pressed = true;
        }
    }

    fn is_pressed(&self) -> bool {
        self.is_pressed
    }

    fn get_buf(&self) -> u16 {
        let mut sum = 0;
        for buf in self.buffer {
            sum += buf as u16;
        }
        sum / BUFFER_SIZE
    }
}

#[derive(Copy, Clone)]
struct WootingPosition {
    pub buffer: [u32; BUFFER_SIZE as usize],
    position: u32,
    buffer_pos: u16,
    min_distance_up: u32,
    min_distance_down: u32,
    tolerance: u32,
    is_pressed: bool,
    wooting: bool,
}

impl WootingPosition {
    pub const fn default() -> Self {
        Self {
            buffer: [0; BUFFER_SIZE as usize],
            position: 0,
            min_distance_up: 0,
            min_distance_down: 0,
            tolerance: 0,
            buffer_pos: 0,
            is_pressed: false,
            wooting: false,
        }
    }
    pub fn new(lowest_point: f32, highest_point: f32) -> Self {
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
            is_pressed: false,
            wooting: false,
        }
    }

    fn update_buf(&mut self, pos: u16) {
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

    fn is_pressed(&self) -> bool {
        self.is_pressed
    }

    fn get_buf(&self) -> u16 {
        let mut sum = 0;
        for buf in self.buffer {
            sum += buf as u16;
        }
        sum / BUFFER_SIZE
    }
}

#[derive(Copy, Clone)]
enum Position {
    Digital(DigitalPosition),
    Wooting(WootingPosition),
    Slave(u8),
}

impl Position {
    pub fn is_pressed(&self) -> bool {
        match self {
            Position::Digital(pos) => pos.is_pressed(),
            Position::Wooting(pos) => pos.is_pressed(),
            Position::Slave(pos) => *pos == 1,
        }
    }

    pub fn update_buf(&mut self, buf: u16) {
        match self {
            Position::Digital(pos) => pos.update_buf(buf),
            Position::Wooting(pos) => pos.update_buf(buf),
            Position::Slave(_) => *self = Position::Slave(buf as u8),
        }
    }

    pub fn get_buf(&self) -> u16 {
        match self {
            Position::Digital(pos) => pos.get_buf(),
            Position::Wooting(pos) => pos.get_buf(),
            Position::Slave(pos) => *pos as u16,
        }
    }
}

#[derive(Copy, Clone, PartialEq)]
pub enum KeyType {
    Letter,
    Modifier,
    Layer,
}

#[derive(Copy, Clone)]
pub struct ScanCode {
    pub key_type: KeyType,
    pub pos: u8,
    pub toggle: bool, // For layer keys. If true, pressing this key toggles the layer stored in pos
}

impl ScanCode {
    const fn default() -> Self {
        Self {
            key_type: KeyType::Letter,
            pos: 0,
            toggle: false,
        }
    }
}

#[derive(Copy, Clone)]
pub enum TapType {
    Normal,
    Hold, // Hold keys will have two scan codes, one for the tap and the other for hold
}

#[derive(Copy, Clone)]
pub struct KeyInfo {
    pub tap_type: TapType,
    tap_code: ScanCode,
    hold_code: ScanCode,
    time: u64,
    min_hold_time: u64,
    tap_time: u64,
    was_held: bool,
    release: TapType,
}

impl KeyInfo {
    const fn default() -> Self {
        Self {
            tap_type: TapType::Normal,
            tap_code: ScanCode::default(),
            hold_code: ScanCode::default(),
            time: 0,
            min_hold_time: HOLD_TIME,
            tap_time: 0,
            was_held: false,
            release: TapType::Normal,
        }
    }
}

pub enum ScanResult {
    Pressed(ScanCode),
    Released(ScanCode),
    Holding,
}

#[derive(Copy, Clone)]
pub struct Key {
    pos: Position,
    key_info: [KeyInfo; NUM_LAYERS],
    pub current_layer: i8,
    start: bool,
    pub roll: bool,
    other_side: bool,
}

impl Key {
    pub fn default() -> Self {
        Self {
            pos: Position::Wooting(WootingPosition::new(1400.0, 2000.0)),
            key_info: [KeyInfo::default(); NUM_LAYERS],
            current_layer: -1,
            start: false,
            roll: false,
            other_side: false,
        }
    }

    pub fn set_normal(&mut self, code: KeyboardCodes, toggle: bool, layer: usize) {
        match code.get_code_type() {
            KeyType::Layer => {
                self.key_info[layer].tap_type = TapType::Normal;
                self.key_info[layer].tap_code = ScanCode {
                    key_type: KeyType::Layer,
                    pos: (code as u8 - KeyboardCodes::Layer0 as u8),
                    toggle,
                };
            }
            KeyType::Modifier => {
                self.key_info[layer].tap_type = TapType::Normal;
                self.key_info[layer].tap_code = ScanCode {
                    key_type: KeyType::Modifier,
                    pos: (code as u8 - KeyboardCodes::KeyboardLeftControl as u8),
                    toggle,
                };
            }
            KeyType::Letter => {
                self.key_info[layer].tap_type = TapType::Normal;
                self.key_info[layer].tap_code = ScanCode {
                    key_type: KeyType::Letter,
                    pos: code as u8,
                    toggle,
                };
            }
        };
    }

    // pub fn set_normal_all(&mut self, code: [KeyboardCodes; NUM_LAYERS], toggle: bool) {
    //     for i in 0..NUM_LAYERS {
    //         self.key_info[i].tap_type = TapType::Normal;
    //         match code[i].get_code_type() {
    //             KeyType::Layer => {
    //                 self.key_info[i].tap_code = ScanCode {
    //                     key_type: KeyType::Layer,
    //                     pos: (code[i] as u8 - KeyboardCodes::Layer0 as u8),
    //                     toggle,
    //                 };
    //             }
    //             KeyType::Modifier => {
    //                 self.key_info[i].tap_code = ScanCode {
    //                     key_type: KeyType::Modifier,
    //                     pos: (code[i] as u8 - KeyboardCodes::KeyboardLeftControl as u8),
    //                     toggle,
    //                 };
    //             }
    //             KeyType::Letter => {
    //                 self.key_info[i].tap_code = ScanCode {
    //                     key_type: KeyType::Letter,
    //                     pos: code[i] as u8,
    //                     toggle,
    //                 };
    //             }
    //         };
    //     }
    // }

    pub fn set_hold(
        &mut self,
        tap_code: KeyboardCodes,
        hold_code: KeyboardCodes,
        tap_toggle: bool,
        hold_toggle: bool,
        layer: usize,
    ) {
        self.key_info[layer].tap_type = TapType::Hold;
        match tap_code.get_code_type() {
            KeyType::Layer => {
                self.key_info[layer].tap_code = ScanCode {
                    key_type: KeyType::Layer,
                    pos: (tap_code as u8 - KeyboardCodes::Layer0 as u8),
                    toggle: tap_toggle,
                };
            }
            KeyType::Modifier => {
                self.key_info[layer].tap_code = ScanCode {
                    key_type: KeyType::Modifier,
                    pos: (tap_code as u8 - KeyboardCodes::KeyboardLeftControl as u8),
                    toggle: tap_toggle,
                };
            }
            KeyType::Letter => {
                self.key_info[layer].tap_code = ScanCode {
                    key_type: KeyType::Letter,
                    pos: tap_code as u8,
                    toggle: tap_toggle,
                };
            }
        };
        match hold_code.get_code_type() {
            KeyType::Layer => {
                self.key_info[layer].hold_code = ScanCode {
                    key_type: KeyType::Layer,
                    pos: (hold_code as u8 - KeyboardCodes::Layer0 as u8),
                    toggle: hold_toggle,
                };
            }
            KeyType::Modifier => {
                self.key_info[layer].hold_code = ScanCode {
                    key_type: KeyType::Modifier,
                    pos: (hold_code as u8 - KeyboardCodes::KeyboardLeftControl as u8),
                    toggle: hold_toggle,
                };
            }
            KeyType::Letter => {
                self.key_info[layer].hold_code = ScanCode {
                    key_type: KeyType::Letter,
                    pos: hold_code as u8,
                    toggle: hold_toggle,
                };
            }
        };
    }

    pub fn set_digital(&mut self) {
        self.pos = Position::Digital(DigitalPosition::new(1400.0, 2000.0));
    }

    pub fn set_wooting(&mut self) {
        self.pos = Position::Wooting(WootingPosition::new(1400.0, 2000.0));
    }

    pub fn set_slave(&mut self) {
        self.pos = Position::Slave(0);
    }

    pub fn update_buf(&mut self, buf: u16) {
        self.pos.update_buf(buf);
    }

    // pub fn is_pressed(&self, layer: usize) -> ScanResult {
    //     if self.pos.is_pressed() {
    //         ScanResult::Pressed(self.scan_codes[layer])
    //     } else {
    //         ScanResult::Released(self.scan_codes[layer])
    //     }
    // }

    pub fn is_pressed(&mut self, layer: usize) -> ScanResult {
        match self.key_info[layer].tap_type {
            TapType::Normal => {
                if self.pos.is_pressed() {
                    ScanResult::Pressed(self.key_info[layer].tap_code)
                } else {
                    ScanResult::Released(self.key_info[layer].tap_code)
                }
            }
            TapType::Hold => unsafe {
                if self.pos.is_pressed() {
                    if self.key_info[layer].time == 0 {
                        self.key_info[layer].time = TIMER.unwrap().get_counter().ticks();
                        self.key_info[layer].release = TapType::Normal;
                        self.start = true;
                        ScanResult::Holding
                    } else if self.key_info[layer].was_held {
                        ScanResult::Pressed(self.key_info[layer].hold_code)
                    } else if TIMER.unwrap().get_counter().ticks() - self.key_info[layer].time
                        > self.key_info[layer].min_hold_time
                        || self.other_side
                    {
                        self.key_info[layer].was_held = true;
                        self.key_info[layer].release = TapType::Hold;
                        ScanResult::Pressed(self.key_info[layer].hold_code)
                    } else {
                        ScanResult::Holding
                    }
                } else {
                    match self.key_info[layer].release {
                        TapType::Hold => {
                            self.key_info[layer].time = 0;
                            self.key_info[layer].tap_time = 0;
                            self.key_info[layer].was_held = false;
                            ScanResult::Released(self.key_info[layer].hold_code)
                        }
                        TapType::Normal => {
                            if self.start {
                                self.start = false;
                                self.key_info[layer].was_held = false;
                                self.key_info[layer].time = 0;
                                self.key_info[layer].tap_time =
                                    TIMER.unwrap().get_counter().ticks();
                                ScanResult::Pressed(self.key_info[layer].tap_code)
                            } else if self.key_info[layer].tap_time != 0
                                && TIMER.unwrap().get_counter().ticks()
                                    - self.key_info[layer].tap_time
                                    < PRESS_TIME
                            {
                                ScanResult::Pressed(self.key_info[layer].tap_code)
                            } else {
                                ScanResult::Released(self.key_info[layer].tap_code)
                            }
                        }
                    }
                }
            },
        }
    }

    pub fn set_other(&mut self, state: bool) {
        self.other_side = state;
    }
    pub fn set_tapping_term(&mut self, term: u64, layer: usize) {
        self.key_info[layer].min_hold_time = term * 1000;
    }

    pub fn get_buf(&self) -> u16 {
        self.pos.get_buf()
    }
}
