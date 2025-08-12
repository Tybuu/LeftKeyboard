use core::mem;

use num_enum::TryFromPrimitive;
use sequential_storage::map::{store_item, SerializationError, Value};

use crate::scan_codes::KeyCodes;

/// Wrapper around ScanCode to allow different fuctionalites when pressed
/// such as sending multiple keys
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ScanCodeBehavior {
    Single(KeyCodes),
    Double(KeyCodes, KeyCodes),
    Triple(KeyCodes, KeyCodes, KeyCodes),
    // Return a different key code depending on the other indexed key press status
    CombinedKey {
        other_index: usize,
        normal_code: KeyCodes,
        combined_code: KeyCodes,
    },
    ChangeConfig(u8),
}

impl ScanCodeBehavior {
    pub const fn default() -> Self {
        Self::Single(KeyCodes::Undefined)
    }
}

#[derive(Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum HidScanCodeType {
    Single = 0,
    Double = 1,
    Triple = 2,
    CombinedKey = 3,
    ChangeConfig = 4,
}
impl HidScanCodeType {
    pub fn get_len(&self) -> usize {
        match self {
            Self::Single => SINGLE_SERIAL_LENGTH,
            Self::Double => DOUBLE_SERIAL_LENGTH,
            Self::Triple => TRIPLE_SERIAL_LENGTH,
            Self::CombinedKey => COMBINED_KEY_SERIAL_LENGTH,
            Self::ChangeConfig => CHANGE_CONFIG_SERIAL_LENGTH,
        }
    }
}

const fn max_len(arr: &[usize]) -> usize {
    let mut max = 0;
    let mut i = 0;
    while i < arr.len() {
        if arr[i] > max {
            max = arr[i];
        }
        i += 1;
    }
    max
}

pub const MAX_SERIAL_LENGTH: usize = max_len(&[
    SINGLE_SERIAL_LENGTH,
    DOUBLE_SERIAL_LENGTH,
    TRIPLE_SERIAL_LENGTH,
    COMBINED_KEY_SERIAL_LENGTH,
    CHANGE_CONFIG_SERIAL_LENGTH,
]);

const SINGLE_SERIAL_LENGTH: usize = 2;
const DOUBLE_SERIAL_LENGTH: usize = 3;
const TRIPLE_SERIAL_LENGTH: usize = 4;
const COMBINED_KEY_SERIAL_LENGTH: usize = 4;
const CHANGE_CONFIG_SERIAL_LENGTH: usize = 2;

impl ScanCodeBehavior {
    pub fn into_buffer_len(&self) -> usize {
        match self {
            ScanCodeBehavior::Single(_) => SINGLE_SERIAL_LENGTH,
            ScanCodeBehavior::Double(_, _) => DOUBLE_SERIAL_LENGTH,
            ScanCodeBehavior::Triple(_, _, _) => TRIPLE_SERIAL_LENGTH,
            ScanCodeBehavior::CombinedKey { .. } => COMBINED_KEY_SERIAL_LENGTH,
            ScanCodeBehavior::ChangeConfig(_) => CHANGE_CONFIG_SERIAL_LENGTH,
        }
    }

    /// Searalizes into buffer
    pub fn into_buffer(
        &self,
        buffer: &mut [u8],
    ) -> Result<(), sequential_storage::map::SerializationError> {
        if buffer.len() < self.into_buffer_len() {
            Err(sequential_storage::map::SerializationError::BufferTooSmall)
        } else {
            match *self {
                ScanCodeBehavior::Single(code) => {
                    buffer[0] = HidScanCodeType::Single as u8;
                    buffer[1] = code as u8;
                }

                ScanCodeBehavior::Double(code0, code1) => {
                    buffer[0] = HidScanCodeType::Double as u8;
                    buffer[1] = code0 as u8;
                    buffer[2] = code1 as u8;
                }
                ScanCodeBehavior::Triple(code0, code1, code2) => {
                    buffer[0] = HidScanCodeType::Triple as u8;
                    buffer[1] = code0 as u8;
                    buffer[2] = code1 as u8;
                    buffer[3] = code2 as u8;
                }
                ScanCodeBehavior::CombinedKey {
                    other_index,
                    normal_code,
                    combined_code,
                } => {
                    buffer[0] = HidScanCodeType::CombinedKey as u8;
                    buffer[1] = normal_code as u8;
                    buffer[2] = combined_code as u8;
                    buffer[3] = other_index as u8;
                }
                ScanCodeBehavior::ChangeConfig(config_num) => {
                    buffer[0] = HidScanCodeType::ChangeConfig as u8;
                    buffer[1] = config_num;
                }
            }
            Ok(())
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct ScanCodeLayerStorage<const N: usize> {
    pub codes: [ScanCodeBehavior; N],
}

impl<const N: usize> ScanCodeLayerStorage<N> {
    pub const fn default() -> Self {
        Self {
            codes: [ScanCodeBehavior::Single(KeyCodes::Undefined); N],
        }
    }
}

impl<'a, const N: usize> Value<'a> for ScanCodeLayerStorage<N> {
    fn serialize_into(
        &self,
        buffer: &mut [u8],
    ) -> Result<usize, sequential_storage::map::SerializationError> {
        let storage_size: usize = self.codes.map(|x| x.into_buffer_len()).iter().sum();
        if buffer.len() < storage_size {
            Err(sequential_storage::map::SerializationError::BufferTooSmall)
        } else {
            let mut i = 0;
            for code in self.codes {
                let code_len = code.into_buffer_len();
                code.into_buffer(&mut buffer[i..(i + code_len)])?;
                i += code_len;
            }
            Ok(storage_size)
        }
    }

    fn deserialize_from(
        buffer: &'a [u8],
    ) -> Result<Self, sequential_storage::map::SerializationError>
    where
        Self: Sized,
    {
        let mut codes = Self::default();
        let mut buf_i = 0;
        let mut code_i = 0;
        while buf_i < buffer.len() {
            let code = ScanCodeBehavior::deserialize_from(&buffer[buf_i..])?;
            codes.codes[code_i] = code;
            buf_i += code.into_buffer_len();
            code_i += 1;
        }
        Ok(codes)
    }
}

impl<'a> Value<'a> for ScanCodeBehavior {
    fn serialize_into(
        &self,
        buffer: &mut [u8],
    ) -> Result<usize, sequential_storage::map::SerializationError> {
        if buffer.len() < self.into_buffer_len() {
            Err(sequential_storage::map::SerializationError::BufferTooSmall)
        } else {
            self.into_buffer(buffer)?;
            Ok(self.into_buffer_len())
        }
    }

    fn deserialize_from(
        buffer: &'a [u8],
    ) -> Result<Self, sequential_storage::map::SerializationError>
    where
        Self: Sized,
    {
        let hid_type = HidScanCodeType::try_from(buffer[0])
            .map_err(|_| sequential_storage::map::SerializationError::InvalidFormat)?;
        match hid_type {
            HidScanCodeType::Single => {
                if buffer.len() < SINGLE_SERIAL_LENGTH {
                    Err(sequential_storage::map::SerializationError::BufferTooSmall)
                } else {
                    let code = buffer[1].into();
                    Ok(ScanCodeBehavior::Single(code))
                }
            }
            HidScanCodeType::Double => {
                if buffer.len() < DOUBLE_SERIAL_LENGTH {
                    Err(sequential_storage::map::SerializationError::BufferTooSmall)
                } else {
                    let code0 = buffer[1].into();
                    let code1 = buffer[2].into();
                    Ok(ScanCodeBehavior::Double(code0, code1))
                }
            }
            HidScanCodeType::Triple => {
                if buffer.len() < TRIPLE_SERIAL_LENGTH {
                    Err(sequential_storage::map::SerializationError::BufferTooSmall)
                } else {
                    let code0 = buffer[1].into();
                    let code1 = buffer[2].into();
                    let code2 = buffer[3].into();
                    Ok(ScanCodeBehavior::Triple(code0, code1, code2))
                }
            }
            HidScanCodeType::CombinedKey => {
                if buffer.len() < COMBINED_KEY_SERIAL_LENGTH {
                    Err(sequential_storage::map::SerializationError::BufferTooSmall)
                } else {
                    let normal_code = buffer[1].into();
                    let combined_code = buffer[2].into();
                    let other_index = buffer[3] as usize;
                    Ok(ScanCodeBehavior::CombinedKey {
                        other_index,
                        normal_code,
                        combined_code,
                    })
                }
            }
            HidScanCodeType::ChangeConfig => {
                if buffer.len() < CHANGE_CONFIG_SERIAL_LENGTH {
                    Err(sequential_storage::map::SerializationError::BufferTooSmall)
                } else {
                    Ok(ScanCodeBehavior::ChangeConfig(buffer[1]))
                }
            }
        }
    }
}
