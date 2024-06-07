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
        (usage_min = 0x00, usage_max = 0xFF) = {
            #[item_settings constant,variable,absolute]
            reserved=input;
        };
        (usage_page = LEDS, usage_min = 0x01, usage_max = 0x05) = {
            #[packed_bits 5]
            #[item_settings data,variable,absolute]
            leds=output;
        };
        (usage_page = KEYBOARD, usage_min = 0x00, usage_max = 0xFF) = {
            #[item_settings data,array,absolute]
            keycodes=input;
        };
        (usage_page = KEYBOARD, usage_min = 0x04, usage_max = 0x65) = {
            #[packed_bits 101]
            #[item_settings data,variable,absolute]
            nkro_keycodes=input;
        };
    }
)]
#[allow(dead_code)]
pub struct KeyboardReportNKRO {
    pub modifier: u8,
    pub reserved: u8,
    pub leds: u8,
    pub keycodes: [u8; 6],
    pub nkro_keycodes: [u8; 13],
}

impl KeyboardReportNKRO {
    pub const fn default() -> Self {
        Self {
            modifier: 0,
            reserved: 0,
            leds: 0,
            keycodes: [0; 6],
            nkro_keycodes: [0; 13],
        }
    }
}
