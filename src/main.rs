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
use keys::{Key, KeyType};
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

static mut SHIFT: bool = false;

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
            .product("crap keypad")])
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

    let mut sel0 = pins.tx.into_push_pull_output();
    let mut sel1 = pins.rx.into_push_pull_output();
    let mut sel2 = pins.d2.into_push_pull_output();

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

    let mut keys = [Key::new([0x00; 2], KeyType::Letter, 1400.0, 2000.0); 21];

    // TODO:: Use EEPROM to initalize the key parameters
    //
    // Initialize Keys
    keys[0].bit_pos[0] = KeyboardUsage::KeyboardHh as u8;
    keys[1].bit_pos[0] = KeyboardUsage::KeyboardKk as u8;
    keys[2].bit_pos[0] = KeyboardUsage::KeyboardOo as u8;
    keys[4].bit_pos[0] = KeyboardUsage::KeyboardYy as u8;
    keys[5].bit_pos[0] = KeyboardUsage::KeyboardUu as u8;
    keys[6].bit_pos[0] = KeyboardUsage::KeyboardLl as u8;
    keys[9].bit_pos[0] = KeyboardUsage::KeyboardJj as u8;
    keys[14].bit_pos[0] = KeyboardUsage::KeyboardPp as u8;
    keys[18].bit_pos[0] = KeyboardUsage::KeyboardIi as u8;

    keys[20].key_type = KeyType::Modifier;
    keys[20].bit_pos[0] = ModifierPosition::LeftShift as u8;

    let mut n: u8 = 128;

    loop {
        ws.write(brightness(once(colors::CYAN), 48)).unwrap();
        n = n.wrapping_add(1);
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
        // let mut buf = [0u8; 22];
        // let mut read = false;
        // loop {
        //     match i2c.read(0x72u8, &mut buf) {
        //         Ok(_) => {
        //             if buf[0] == 0x7 {
        //                 read = true;
        //                 break;
        //             }
        //         }
        //         Err(_) => break,
        //     }
        // }
        let report = generate_report(&mut keys);
        critical_section::with(|_| unsafe {
            (REPORT.modifier, REPORT.nkro_keycodes) = report;
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
            BUFFER.key_report1[10] = SHIFT as u8;
        });
        asm::wfi();
    }
}

fn change_sel<P: PullType>(
    sel0: &mut Pin<Gpio0, FunctionSio<SioOutput>, P>,
    sel1: &mut Pin<Gpio1, FunctionSio<SioOutput>, P>,
    sel2: &mut Pin<Gpio2, FunctionSio<SioOutput>, P>,
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
// is the modifier keys while the second is the nkro keycodes
fn generate_report(keys: &mut [Key]) -> (u8, [u8; 11]) {
    let mut mod_report = 0u8;
    let mut nkro_report = [0u8; 11];

    unsafe {
        if SHIFT {
            let mask = 1 << 1;
            mod_report |= mask;
        }
    }

    // Find which layer we need to use by looking at the status of the layer keys
    let mut layer: usize = 0;
    for i in 0..keys.len() {
        if keys[i].is_pressed() {
            if keys[i].current_layer == -1 {
                match keys[i].key_type {
                    KeyType::Letter => {
                        let index = keys[i].bit_pos[layer] / 8;
                        let mask = 1 << (keys[i].bit_pos[layer] % 8);
                        nkro_report[usize::from(index)] |= mask;
                    }
                    KeyType::Modifier => {
                        let mask = 1 << (keys[i].bit_pos[layer] % 8);
                        mod_report |= mask;
                    }
                    KeyType::Layer => {}
                }
                keys[i].current_layer = layer as i8;
            } else {
                let layer = usize::from(keys[i].current_layer as u8);
                match keys[i].key_type {
                    KeyType::Letter => {
                        let index = keys[i].bit_pos[layer] / 8;
                        let mask = 1 << (keys[i].bit_pos[layer] % 8);
                        nkro_report[usize::from(index)] |= mask;
                    }
                    KeyType::Modifier => {
                        let mask = 1 << (keys[i].bit_pos[layer] % 8);
                        mod_report |= mask;
                    }
                    KeyType::Layer => {}
                }
            }
        } else {
            keys[i].current_layer = -1;
        }
    }
    (mod_report, nkro_report)
}

fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
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
    USB_HID.as_mut().map(|hid| hid.push_input(&REPORT));
    USB_HID.as_mut().unwrap().pull_raw_output(&mut [0; 64]).ok();
    USB_HID2.as_mut().map(|hid| hid.push_input(&BUFFER));
    let mut buf = [0u8; 32];
    USB_HID2.as_mut().unwrap().pull_raw_output(&mut buf).ok();
    if buf[0] == 7 {
        SHIFT = !SHIFT;
    }
}

// End of file
