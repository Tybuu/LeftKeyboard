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
        (usage_page = KEYBOARD, usage_min = 0x00, usage_max = 0x53) = {
            #[packed_bits 83]
            #[item_settings data,variable,absolute]
            nkro_keycodes=input;
        };
    }
)]
#[allow(dead_code)]
pub struct KeyboardReportNKRO {
    pub modifier: u8,
    pub nkro_keycodes: [u8; 11],
}

impl KeyboardReportNKRO {
    pub const fn default() -> Self {
        Self {
            modifier: 0,
            nkro_keycodes: [0; 11],
        }
    }
}

pub enum ModifierPosition {
    LeftCtrl = 0,
    LeftShift = 1,
    LeftAlt = 2,
    LeftGui = 3,
    RightCtrl = 4,
    RightShift = 5,
    RightAlt = 6,
    RightGui = 7,
}

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = VENDOR_DEFINED_START, usage = 0x01) = {
        key_report0=input;
        key_report1=input;
        inner_modifier=input;
        layer_key=input;
        key_pressed=input;
        output_buffer=output;
    }
)]
// The max for a single array is 32 elements
#[allow(dead_code)]
pub struct BufferReport {
    pub key_report0: [u8; 32],
    pub key_report1: [u8; 10],
    pub inner_modifier: u8,
    pub layer_key: u8,
    pub key_pressed: u8,
    pub output_buffer: [u8; 32],
}

impl BufferReport {
    pub const fn default() -> Self {
        Self {
            key_report0: [0u8; 32],
            key_report1: [0u8; 10],
            inner_modifier: 0u8,
            layer_key: 0u8,
            key_pressed: 0u8,
            output_buffer: [0u8; 32],
        }
    }
}
