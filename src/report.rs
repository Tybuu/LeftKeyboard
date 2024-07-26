use crate::{
    hid::KeyboardReportNKRO,
    keys::{Key, KeyType, ScanResult, TapType},
};

fn set_bit(num: u8, bit: u8, pos: u8) -> u8 {
    let mask = 1 << pos;
    if bit == 1 {
        num | mask
    } else {
        num & !mask
    }
}

pub struct Report {
    report: KeyboardReportNKRO,
    current_layer: usize,
    reset_layer: usize,
}

impl Report {
    pub const fn default() -> Self {
        Self {
            report: KeyboardReportNKRO::default(),
            current_layer: 0,
            reset_layer: 0,
        }
    }

    pub fn generate_report(&mut self, keys: &mut [Key]) -> Option<KeyboardReportNKRO> {
        let mut new_layer: i8 = -1;
        let mut reset = false; // if true, change the reset layer
        let mut changed = false;
        let mut key_pressed = false;
        for key in &mut *keys {
            match key.is_pressed(self.current_layer) {
                ScanResult::Pressed(code) => match code.key_type {
                    KeyType::Letter => {
                        key_pressed = true;
                    }
                    _ => {}
                },
                _ => {}
            }
        }
        for key in &mut *keys {
            if key.roll {
                key.set_other(key_pressed);
            }
        }
        // If multiple layer keys are pressed, the higest value will get priority
        for key in &mut *keys {
            let mut current_layer = self.current_layer;

            if key.current_layer != -1 {
                current_layer = key.current_layer as usize;
            }
            match key.is_pressed(current_layer) {
                ScanResult::Pressed(code) => {
                    if code.key_type == KeyType::Layer && code.pos as i8 > new_layer {
                        new_layer = code.pos as i8;
                        reset = code.toggle;
                        key.current_layer = current_layer as i8;
                    }
                }
                _ => {}
            }
        }
        if new_layer == -1 {
            new_layer = self.reset_layer as i8;
        }

        if self.current_layer as i8 != new_layer {
            self.current_layer = new_layer as usize;
            if reset {
                self.reset_layer = new_layer as usize;
            }
        }
        for key in &mut *keys {
            let mut current_layer = self.current_layer;
            if key.current_layer != -1 {
                current_layer = key.current_layer as usize;
            }
            match key.is_pressed(current_layer) {
                ScanResult::Pressed(code) => match code.key_type {
                    KeyType::Modifier => {
                        let b_idx = code.pos % 8;
                        let res = set_bit(self.report.modifier, 1, b_idx);
                        if res != self.report.modifier {
                            self.report.modifier = res;
                            changed = true;
                        }
                        key.current_layer = current_layer as i8;
                    }
                    KeyType::Letter => {
                        let n_idx = (code.pos / 8) as usize;
                        let b_idx = code.pos % 8;
                        let res = set_bit(self.report.nkro_keycodes[n_idx], 1, b_idx);
                        if res != self.report.nkro_keycodes[n_idx] {
                            self.report.nkro_keycodes[n_idx] = res;
                            changed = true;
                        }
                        key.current_layer = current_layer as i8;
                    }
                    KeyType::Layer => {}
                },
                ScanResult::Released(code) => match code.key_type {
                    KeyType::Modifier => {
                        let b_idx = code.pos % 8;
                        let res = set_bit(self.report.modifier, 0, b_idx);
                        if res != self.report.modifier {
                            self.report.modifier = res;
                            changed = true;
                        }
                        key.current_layer = -1;
                    }
                    KeyType::Letter => {
                        let n_idx = (code.pos / 8) as usize;
                        let b_idx = code.pos % 8;
                        let res = set_bit(self.report.nkro_keycodes[n_idx], 0, b_idx);
                        if res != self.report.nkro_keycodes[n_idx] {
                            self.report.nkro_keycodes[n_idx] = res;
                            changed = true;
                        }
                        key.current_layer = -1;
                    }
                    KeyType::Layer => {
                        key.current_layer = -1;
                    }
                },
                ScanResult::Holding => {
                    if key.current_layer == -1 {
                        key.current_layer = self.current_layer as i8;
                    }
                }
            }
        }
        if changed {
            Some(self.report)
        } else {
            None
        }
    }
}
