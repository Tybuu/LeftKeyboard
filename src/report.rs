use crate::{
    hid::{BufferReport, KeyboardReportNKRO},
    keys::{Key, ScanCodeType},
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
    inner_layer: usize,
    current_layer: usize,
    inner_modifier: u8,
}

impl Report {
    pub const fn default() -> Self {
        Self {
            report: KeyboardReportNKRO::default(),
            inner_layer: 0,
            inner_modifier: 0,
            current_layer: 0,
        }
    }

    pub fn generate_report(
        &mut self,
        keys: &mut [Key],
        external_layer: usize,
        external_modifier: u8,
    ) -> Option<(KeyboardReportNKRO, BufferReport)> {
        let layer = self.current_layer;
        let mut changed = false;
        let mut layer_pressed = false;
        keys.iter_mut()
            .for_each(|key| match key.scan_code_type[layer] {
                ScanCodeType::Layer => {
                    if key.is_pressed() {
                        self.inner_layer = key.bit_pos[layer] as usize;
                        self.current_layer = key.bit_pos[layer] as usize;
                        layer_pressed = true;
                    }
                }
                _ => {}
            });
        if (external_layer > 0 && !layer_pressed) || external_layer > self.inner_layer {
            self.current_layer = external_layer;
        } else if !layer_pressed {
            self.inner_layer = 0;
            self.current_layer = 0;
        }

        if layer != self.current_layer {
            changed = true;
        }

        keys.iter_mut().for_each(|key| {
            let val = if key.is_pressed() { 1 } else { 0 };
            let mut current_layer = self.current_layer;
            if key.current_layer != -1 {
                current_layer = key.current_layer as usize;
            }
            match key.scan_code_type[current_layer] {
                ScanCodeType::Letter => {
                    let n_idx = (key.bit_pos[current_layer] / 8) as usize;
                    let b_idx = key.bit_pos[current_layer] % 8;
                    let res = set_bit(self.report.nkro_keycodes[n_idx], val, b_idx);
                    if res != self.report.nkro_keycodes[n_idx] {
                        changed = true;
                        self.report.nkro_keycodes[n_idx] = res;
                    }
                }
                ScanCodeType::Modifier => {
                    let b_idx = key.bit_pos[current_layer] % 8;
                    self.inner_modifier = set_bit(self.inner_modifier, val, b_idx);
                }
                _ => {}
            }
            if val == 1 {
                key.current_layer = current_layer as i8;
            } else {
                key.current_layer = -1;
            }
        });
        let res = self.inner_modifier | external_modifier;
        if res != self.report.modifier {
            changed = true;
            self.report.modifier = res;
        }
        if changed {
            let mut buffer = BufferReport::default();
            buffer.layer_key = self.inner_layer as u8;
            buffer.inner_modifier = self.inner_modifier;
            Some((self.report, buffer))
        } else {
            None
        }
    }
}
