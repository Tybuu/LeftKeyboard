#![no_std]
#![no_main]

mod keys;
mod report;

use adafruit_kb2040 as bsp;
use bsp::{
    entry,
    hal::{
        self,
        adc::AdcPin,
        gpio::{
            bank0::{Gpio0, Gpio1, Gpio2},
            FunctionSio, Pin, PullType, SioOutput,
        },
        Adc, Timer,
    },
    pac::{self, interrupt::USBCTRL_IRQ, RESETS},
};
use core::iter::once;
use cortex_m::asm;
use cortex_m::prelude::_embedded_hal_adc_OneShot;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use hal::pio::PIOExt;
use keys::{Key, KeyType, ScanCodeType};
use panic_probe as _;
use report::{BufferReport, KeyboardReportNKRO};

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
// use sparkfun_pro_micro_rp2040 as bsp;

use adafruit_kb2040::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac::interrupt,
    sio::Sio,
    watchdog::Watchdog,
};

use smart_leds::{brightness, colors};
use smart_leds_trait::{SmartLedsWrite, RGB8};
use usb_device::{
    class_prelude::{UsbBusAllocator, UsbClass},
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_hid::{
    descriptor::{KeyboardReport, KeyboardUsage, SerializedDescriptor},
    hid_class::{
        HIDClass, HidClassSettings, HidCountryCode, HidProtocol, HidProtocolMode, HidSubClass,
        ProtocolModeConfig,
    },
};
use ws2812_pio::Ws2812;

use crate::report::ModifierPosition;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

static mut USB_HID2: Option<HIDClass<hal::usb::UsbBus>> = None;

static mut REPORT: KeyboardReportNKRO = KeyboardReportNKRO::default();

static mut BUFFER: BufferReport = BufferReport::default();

static mut KEYS: [Key; 21] = [Key::default(); 21];

static mut MOD_REPORT: u8 = 0u8;

static mut LAYER_KEY: u8 = 0u8;

static mut KEY_PRESSED: u8 = 0;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        pins.neopixel.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        USB_BUS = Some(usb_bus);
    }
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    let mut usb_settings = HidClassSettings::default();
    usb_settings.subclass = HidSubClass::NoSubClass;
    usb_settings.protocol = HidProtocol::Keyboard;
    usb_settings.config = ProtocolModeConfig::ForceReport;
    usb_settings.locale = HidCountryCode::US;

    let usb_hid = HIDClass::new_with_settings(bus_ref, KeyboardReportNKRO::desc(), 1, usb_settings);
    unsafe {
        USB_HID = Some(usb_hid);
    }

    let usb_hid2 = HIDClass::new(bus_ref, BufferReport::desc(), 1);

    unsafe {
        USB_HID2 = Some(usb_hid2);
    }

    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x0a55, 0x0a55))
        .strings(&[StringDescriptors::default()
            .manufacturer("tybeast")
            .product("Left Tybeast Ones")])
        .unwrap()
        .device_class(3) // from: https://www.usb.org/defined-class-codes
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let mut sel0 = pins.d2.into_push_pull_output();
    let mut sel1 = pins.rx.into_push_pull_output();
    let mut sel2 = pins.tx.into_push_pull_output();

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    // Pins names are matched to the names in the schematic (i trolled)
    let mut a3 = AdcPin::new(pins.a0).unwrap();
    let mut a2 = AdcPin::new(pins.a1).unwrap();
    let mut a1 = AdcPin::new(pins.a2).unwrap();
    let mut a0 = AdcPin::new(pins.a3).unwrap();

    // let mut i2c = I2C::i2c1(
    //     pac.I2C1,
    //     pins.d10.reconfigure(),
    //     pins.mosi.reconfigure(),
    //     400.kHz(),
    //     &mut pac.RESETS,
    //     clocks.system_clock.freq(),
    // );

    let mut keys = [Key::new(
        [0x00; keys::NUM_LAYERS],
        ScanCodeType::Letter,
        KeyType::Normal,
        1400.0,
        2000.0,
    ); 21];

    // TODO:: Use EEPROM to initalize the key parameters
    //
    // Initialize Keys

    // Rows go from top to bottom
    // First Row left to right
    keys[7].bit_pos[0] = KeyboardUsage::KeyboardEscape as u8;
    keys[7].bit_pos[1] = KeyboardUsage::KeyboardEscape as u8;
    keys[14].bit_pos[0] = KeyboardUsage::KeyboardQq as u8;
    keys[14].bit_pos[1] = KeyboardUsage::KeyboardBacktickTilde as u8;
    keys[14].bit_pos[2] = KeyboardUsage::KeyboardF1 as u8;
    keys[2].bit_pos[0] = KeyboardUsage::KeyboardWw as u8;
    keys[2].bit_pos[2] = KeyboardUsage::KeyboardF2 as u8;
    keys[18].bit_pos[0] = KeyboardUsage::KeyboardEe as u8;
    keys[18].bit_pos[2] = KeyboardUsage::KeyboardF3 as u8;
    keys[5].bit_pos[0] = KeyboardUsage::KeyboardRr as u8;
    keys[5].bit_pos[2] = KeyboardUsage::KeyboardF4 as u8;
    keys[0].bit_pos[0] = KeyboardUsage::KeyboardTt as u8;
    keys[0].bit_pos[2] = KeyboardUsage::KeyboardF5 as u8;

    // Middle Row
    keys[3].bit_pos[0] = KeyboardUsage::KeyboardCapsLock as u8;
    keys[3].bit_pos[1] = KeyboardUsage::KeyboardCapsLock as u8;
    keys[11].bit_pos[0] = KeyboardUsage::KeyboardAa as u8;
    keys[11].bit_pos[1] = KeyboardUsage::Keyboard1Exclamation as u8;
    keys[6].bit_pos[0] = KeyboardUsage::KeyboardSs as u8;
    keys[6].bit_pos[1] = KeyboardUsage::Keyboard2At as u8;
    keys[1].bit_pos[0] = KeyboardUsage::KeyboardDd as u8;
    keys[1].bit_pos[1] = KeyboardUsage::Keyboard3Hash as u8;
    keys[9].bit_pos[0] = KeyboardUsage::KeyboardFf as u8;
    keys[9].bit_pos[1] = KeyboardUsage::Keyboard4Dollar as u8;
    keys[4].bit_pos[0] = KeyboardUsage::KeyboardGg as u8;
    keys[4].bit_pos[1] = KeyboardUsage::Keyboard5Percent as u8;

    // Bottom Row
    keys[15].scan_code_type = ScanCodeType::Modifier;
    keys[15].bit_pos[0] = ModifierPosition::LeftShift as u8;
    keys[15].bit_pos[1] = ModifierPosition::LeftShift as u8;
    keys[19].bit_pos[0] = KeyboardUsage::KeyboardZz as u8;
    keys[10].bit_pos[0] = KeyboardUsage::KeyboardXx as u8;
    keys[13].bit_pos[0] = KeyboardUsage::KeyboardCc as u8;
    keys[17].bit_pos[0] = KeyboardUsage::KeyboardVv as u8;
    keys[8].bit_pos[0] = KeyboardUsage::KeyboardBb as u8;

    // Thumb Row
    keys[12].scan_code_type = ScanCodeType::Modifier;
    keys[12].key_type = KeyType::DoubleToHold;
    keys[12].bit_pos[0] = ModifierPosition::LeftGui as u8;
    keys[12].bit_pos[1] = ModifierPosition::LeftGui as u8;
    keys[16].scan_code_type = ScanCodeType::Layer;
    keys[20].bit_pos[0] = KeyboardUsage::KeyboardSpacebar as u8;
    keys[20].bit_pos[1] = KeyboardUsage::KeyboardSpacebar as u8;

    ws.write(brightness(once(colors::CYAN), 48)).unwrap();
    loop {
        for i in 0..6 {
            change_sel(&mut sel0, &mut sel1, &mut sel2, i);
            delay.delay_us(10);
            if i != 5 {
                keys[usize::from(4 * i as u16)].update_buf(adc.read(&mut a0).unwrap_or(69));
                keys[usize::from(4 * i as u16 + 1)].update_buf(adc.read(&mut a1).unwrap_or(69));
                keys[usize::from(4 * i as u16 + 2)].update_buf(adc.read(&mut a2).unwrap_or(69));
                keys[usize::from(4 * i as u16 + 3)].update_buf(adc.read(&mut a3).unwrap_or(69));
            } else {
                keys[20].update_buf(adc.read(&mut a0).unwrap_or(69));
            }
        }
        let report = generate_report(&mut keys);
        critical_section::with(|_| unsafe {
            (
                REPORT.modifier,
                BUFFER.layer_key,
                BUFFER.key_pressed,
                REPORT.nkro_keycodes,
            ) = report;
            BUFFER.inner_modifier = REPORT.modifier;
            REPORT.modifier |= MOD_REPORT;
            for i in 0..16 {
                let bytes = keys[usize::from(i as u16)].get_buf().to_be_bytes();
                BUFFER.key_report0[usize::from(i as u16 * 2)] = bytes[0];
                BUFFER.key_report0[usize::from(i as u16 * 2 + 1)] = bytes[1];
            }

            for i in 16..21 {
                let bytes = keys[usize::from(i as u16)].get_buf().to_be_bytes();
                BUFFER.key_report1[usize::from((i as u16 - 16) * 2)] = bytes[0];
                BUFFER.key_report1[usize::from((i as u16 - 16) * 2 + 1)] = bytes[1];
            }
        });
        asm::wfi();
    }
}

fn change_sel<P: PullType>(
    sel0: &mut Pin<Gpio2, FunctionSio<SioOutput>, P>,
    sel1: &mut Pin<Gpio1, FunctionSio<SioOutput>, P>,
    sel2: &mut Pin<Gpio0, FunctionSio<SioOutput>, P>,
    num: u8,
) {
    match num {
        0 => {
            sel0.set_low().unwrap();
            sel1.set_low().unwrap();
            sel2.set_low().unwrap();
        }
        1 => {
            sel0.set_high().unwrap();
            sel1.set_low().unwrap();
            sel2.set_low().unwrap();
        }
        2 => {
            sel0.set_low().unwrap();
            sel1.set_high().unwrap();
            sel2.set_low().unwrap();
        }
        3 => {
            sel0.set_high().unwrap();
            sel1.set_high().unwrap();
            sel2.set_low().unwrap();
        }
        4 => {
            sel0.set_low().unwrap();
            sel1.set_low().unwrap();
            sel2.set_high().unwrap();
        }
        5 => {
            sel0.set_high().unwrap();
            sel1.set_low().unwrap();
            sel2.set_high().unwrap();
        }
        _ => {
            sel0.set_low().unwrap();
            sel1.set_low().unwrap();
            sel2.set_low().unwrap();
        }
    }
}

// Returns the values that can be used in the keyboard report. First value in the tuple
// is the modifier keys, second being the layer key, the third being if a key is pressed, while the last is the nkro keycodes
fn generate_report(keys: &mut [Key]) -> (u8, u8, u8, [u8; 11]) {
    let mut mod_report = 0u8;
    let mut nkro_report = [0u8; 11];
    let mut layer_key = 0u8;

    let mut key_pressed = false;

    // Find which layer we need to use by looking at the status of the layer keys
    let mut layer: usize = 0;
    critical_section::with(|_| unsafe {
        if LAYER_KEY != 0 {
            layer = 2;
        }
    });
    if keys[16].is_pressed() {
        layer = 1;
        layer_key = 1;
    }

    for i in 0..keys.len() {
        if keys[i].is_pressed() {
            let mut current_layer = layer;
            if keys[i].current_layer != -1 {
                current_layer = keys[i].current_layer as usize;
            }
            match keys[i].scan_code_type {
                ScanCodeType::Letter => {
                    let index = keys[i].bit_pos[current_layer] / 8;
                    let mask = 1 << (keys[i].bit_pos[current_layer] % 8);
                    key_pressed = true;
                    nkro_report[usize::from(index)] |= mask;
                }
                ScanCodeType::Modifier => {
                    let mask = 1 << (keys[i].bit_pos[current_layer] % 8);
                    mod_report |= mask;
                }
                ScanCodeType::Layer => {}
            }
            keys[i].current_layer = current_layer as i8;
        } else {
            keys[i].current_layer = -1;
        }
    }
    critical_section::with(|_| unsafe {
        if key_pressed || KEY_PRESSED == 1 {
            for key in keys {
                match key.key_type {
                    KeyType::Normal => {}
                    KeyType::DoubleToHold => {
                        key.hold = false;
                    }
                };
            }
        }
    });
    (mod_report, layer_key, key_pressed as u8, nkro_report)
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt()]
unsafe fn USBCTRL_IRQ() {
    if USB_DEVICE
        .as_mut()
        .unwrap()
        .poll(&mut [USB_HID.as_mut().unwrap(), USB_HID2.as_mut().unwrap()])
    {
        USB_HID.as_mut().unwrap().poll();
        USB_HID2.as_mut().unwrap().poll();
    }
    let mut buf = [0u8; 64];
    USB_HID2.as_mut().unwrap().pull_raw_output(&mut buf).ok();
    if buf[0] == 5 {
        MOD_REPORT = buf[1];
        LAYER_KEY = buf[2];
        KEY_PRESSED = buf[3];
    }
    REPORT.modifier |= MOD_REPORT;
    USB_HID.as_mut().map(|hid| hid.push_input(&REPORT));
    USB_HID.as_mut().unwrap().pull_raw_output(&mut [0; 64]).ok();
    USB_HID2.as_mut().map(|hid| hid.push_input(&BUFFER));
}

// End of file
