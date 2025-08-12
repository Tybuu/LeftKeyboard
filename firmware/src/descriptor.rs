use core::fmt;

use heapless::Vec;
use usbd_hid::descriptor::gen_hid_descriptor;
use usbd_hid::descriptor::{
    generator_prelude::{Serialize, SerializeTuple, SerializedDescriptor, Serializer},
    AsInputReport,
};

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = KEYBOARD) = {
        (usage_page = KEYBOARD, usage_min = 0xE0, usage_max = 0xE7) = {
            #[packed_bits 8]
            #[item_settings data,variable,absolute]
            modifier=input;
        };
        (usage_page = KEYBOARD, usage_min = 0x00, usage_max = 0xDE) = {
            #[packed_bits 222]
            #[item_settings data,variable,absolute]
            nkro_keycodes=input;
        };
    }
)]
#[allow(dead_code)]
#[derive(PartialEq, Eq)]
pub struct KeyboardReportNKRO {
    pub modifier: u8,
    pub nkro_keycodes: [u8; 28],
}

impl KeyboardReportNKRO {
    pub const fn default() -> Self {
        Self {
            modifier: 0,
            nkro_keycodes: [0; 28],
        }
    }
}

impl fmt::Display for KeyboardReportNKRO {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Modifier: {}\n Codes: {:?}",
            self.modifier, self.nkro_keycodes
        )
    }
}

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = MOUSE) = {
        (collection = PHYSICAL, usage = POINTER) = {
            (usage_page = BUTTON, usage_min = BUTTON_1, usage_max = BUTTON_8) = {
                #[packed_bits 8] #[item_settings data,variable,absolute] buttons=input;
            };
            (usage_page = GENERIC_DESKTOP,) = {
                (usage = X,) = {
                    #[item_settings data,variable,relative] x=input;
                };
                (usage = Y,) = {
                    #[item_settings data,variable,relative] y=input;
                };
                (usage = WHEEL,) = {
                    #[item_settings data,variable,relative] wheel=input;
                };
            };
            (usage_page = CONSUMER,) = {
                (usage = AC_PAN,) = {
                    #[item_settings data,variable,relative] pan=input;
                };
            };
        };
    }
)]
#[allow(dead_code)]
#[derive(PartialEq, Eq, Default)]
pub struct MouseReport {
    pub buttons: u8,
    pub x: i8,
    pub y: i8,
    pub wheel: i8, // Scroll down (negative) or up (positive) this many units
    pub pan: i8,   // Scroll left (negative) or right (positive) this many units
}

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = 0xFF69, usage = 0x01) = {
        input=input;
        output=output;
    }
)]
// The max for a single array is 32 elements
#[allow(dead_code)]
#[derive(Default)]
pub struct BufferReport {
    pub input: [u8; 32],
    pub output: [u8; 32],
}

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = 0xFF68, usage = 0x01) = {
        input=input;
        output=output;
    }
)]
// The max for a single array is 32 elements
#[allow(dead_code)]
#[derive(Default)]
pub struct SlaveReport {
    pub input: [u8; 32],
    pub output: [u8; 32],
}

// #[gen_hid_descriptor(
//     (collection = APPLICATION, usage_page = 0xFF68, usage = 0x01) = {
//         key_states=input;
//     }
// )]
// #[derive(PartialEq, Eq)]
// pub struct SlaveKeyReport {
//     pub key_states: [u8; 3],
// }
//
// impl SlaveKeyReport {
//     pub const fn default() -> Self {
//         Self {
//             key_states: [0u8; 3],
//         }
//     }
//
//     pub fn generate_report<const S: usize>(
//         &mut self,
//         keys: &mut Keys<S>,
//     ) -> Option<SlaveKeyReport> {
//         let mut pressed = Vec::<_, S>::new();
//         keys.is_pressed(&mut pressed);
//         let mut new_report = SlaveKeyReport::default();
//         for i in pressed {
//             let a_idx = (i / 8) as usize;
//             let b_idx = i % 8;
//             new_report.key_states[a_idx] |= 1 << b_idx;
//         }
//         if new_report != *self {
//             *self = new_report;
//             Some(*self)
//         } else {
//             None
//         }
//     }
// }
