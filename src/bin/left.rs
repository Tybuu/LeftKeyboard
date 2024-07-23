#![no_std]
#![no_main]

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
use embedded_hal::digital::{InputPin, OutputPin};
use hal::{
    gpio::{
        DynFunction, DynPinId, DynSioConfig, FunctionSioInput, InOutPin, PinId, PinState, SioConfig,
    },
    pio::PIOExt,
};
use keyboard::{
    codes::KeyboardCodes,
    keys::{Key, NUM_LAYERS},
    report::Report,
};
use keyboard::{
    hid::{BufferReport, KeyboardReportNKRO},
    keys,
};
use panic_probe as _;

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

const NUM_KEYS: u32 = 21;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

static mut USB_HID2: Option<HIDClass<hal::usb::UsbBus>> = None;

static mut SLAVE_KEY_STATES: [u8; 3] = [0u8; 3];
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
        .device_class(0xef) // from: https://www.usb.org/defined-class-codes
        .device_sub_class(0x02)
        .device_protocol(0x01)
        .composite_with_iads()
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

    // let mut s_pins = SelPins::new(sel0, sel1, sel2);

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    // Pins names are matched to the names in the schematic (i trolled)
    let mut a3 = AdcPin::new(pins.a0).unwrap();
    let mut a2 = AdcPin::new(pins.a1).unwrap();
    let mut a1 = AdcPin::new(pins.a2).unwrap();
    let mut a0 = AdcPin::new(pins.a3).unwrap();

    let mut order: [u8; NUM_KEYS as usize] = [
        7, 14, 2, 18, 5, 0, 3, 11, 6, 1, 9, 4, 15, 19, 10, 13, 17, 8, 12, 16, 20,
    ];
    find_order(&mut order);
    let mut keys = [Key::default(); 42];

    // Initialize Keys for Left Keys
    // Rows go from top to bottom
    // First Row left to right
    // keys[0].set_normal(KeyboardCodes::KeyboardEscape, false, 0);
    keys[0].set_normal(KeyboardCodes::KeyboardTab, false, 0);
    keys[1].set_normal(KeyboardCodes::KeyboardQq, false, 0);
    keys[1].set_normal(KeyboardCodes::KeyboardBacktickTilde, false, 1);
    keys[1].set_normal(KeyboardCodes::KeyboardF1, false, 2);
    keys[2].set_normal(KeyboardCodes::KeyboardWw, false, 0);
    keys[2].set_normal(KeyboardCodes::KeyboardF2, false, 2);
    keys[3].set_normal(KeyboardCodes::KeyboardEe, false, 0);
    keys[3].set_normal(KeyboardCodes::KeyboardF3, false, 2);

    keys[4].set_normal(KeyboardCodes::KeyboardRr, false, 0);
    keys[4].set_normal(KeyboardCodes::KeyboardF4, false, 2);
    keys[5].set_normal(KeyboardCodes::KeyboardTt, false, 0);
    keys[5].set_normal(KeyboardCodes::KeyboardF5, false, 2);

    // Middle Row
    // keys[6].set_normal_all([KeyboardCodes::KeyboardLeftControl; NUM_LAYERS], false);
    keys[6].set_hold(
        KeyboardCodes::KeyboardEscape,
        KeyboardCodes::KeyboardLeftControl,
        false,
        false,
        0,
    );
    keys[7].set_normal(KeyboardCodes::KeyboardAa, false, 0);
    keys[7].set_normal(KeyboardCodes::Keyboard1Exclamation, false, 1);
    keys[8].set_normal(KeyboardCodes::KeyboardSs, false, 0);
    keys[8].set_normal(KeyboardCodes::Keyboard2At, false, 1);
    keys[9].set_normal(KeyboardCodes::KeyboardDd, false, 0);
    keys[9].set_normal(KeyboardCodes::Keyboard3Hash, false, 1);
    keys[10].set_normal(KeyboardCodes::KeyboardFf, false, 0);
    keys[10].set_normal(KeyboardCodes::Keyboard4Dollar, false, 1);
    keys[11].set_normal(KeyboardCodes::KeyboardGg, false, 0);
    keys[11].set_normal(KeyboardCodes::Keyboard5Percent, false, 1);

    // Bottom Row
    keys[12].set_normal_all([KeyboardCodes::KeyboardLeftShift; NUM_LAYERS], false);
    keys[13].set_normal(KeyboardCodes::KeyboardZz, false, 0);
    keys[14].set_normal(KeyboardCodes::KeyboardXx, false, 0);
    keys[15].set_normal(KeyboardCodes::KeyboardCc, false, 0);
    keys[16].set_normal(KeyboardCodes::KeyboardVv, false, 0);
    keys[17].set_normal(KeyboardCodes::KeyboardBb, false, 0);

    // Thumb Row
    keys[18].set_normal_all([KeyboardCodes::KeyboardLeftGUI; NUM_LAYERS], false);
    keys[19].set_normal_all([KeyboardCodes::Layer1; NUM_LAYERS], false);
    keys[20].set_normal_all([KeyboardCodes::KeyboardSpacebar; NUM_LAYERS], false);

    // Right Keyboard
    // Top Row
    //
    keys[21].set_normal(KeyboardCodes::KeyboardYy, false, 0);
    keys[21].set_normal(KeyboardCodes::KeyboardDashUnderscore, false, 1);
    keys[21].set_normal(KeyboardCodes::KeyboardF6, false, 2);
    keys[22].set_normal(KeyboardCodes::KeyboardUu, false, 0);
    keys[22].set_normal(KeyboardCodes::KeyboardEqualPlus, false, 1);
    keys[22].set_normal(KeyboardCodes::KeyboardF7, false, 2);
    keys[23].set_normal(KeyboardCodes::KeyboardIi, false, 0);
    keys[23].set_normal(KeyboardCodes::KeyboardOpenBracketBrace, false, 1);
    keys[23].set_normal(KeyboardCodes::KeyboardF8, false, 2);
    keys[24].set_normal(KeyboardCodes::KeyboardOo, false, 0);
    keys[24].set_normal(KeyboardCodes::KeyboardCloseBracketBrace, false, 1);
    keys[24].set_normal(KeyboardCodes::KeyboardF8, false, 2);
    keys[25].set_normal(KeyboardCodes::KeyboardPp, false, 0);
    keys[25].set_normal(KeyboardCodes::KeyboardBackslashBar, false, 1);
    keys[25].set_normal(KeyboardCodes::KeyboardF9, false, 2);
    keys[26].set_normal(KeyboardCodes::KeyboardBackspace, false, 0);
    keys[26].set_normal(KeyboardCodes::KeyboardBackspace, false, 1);
    keys[26].set_normal(KeyboardCodes::KeyboardF10, false, 2);

    // Middle Row
    keys[27].set_normal(KeyboardCodes::KeyboardHh, false, 0);
    keys[27].set_normal(KeyboardCodes::Keyboard6Caret, false, 1);
    keys[27].set_normal(KeyboardCodes::KeyboardLeftArrow, false, 2);
    keys[28].set_normal(KeyboardCodes::KeyboardJj, false, 0);
    keys[28].set_normal(KeyboardCodes::Keyboard7Ampersand, false, 1);
    keys[28].set_normal(KeyboardCodes::KeyboardDownArrow, false, 2);
    keys[29].set_normal(KeyboardCodes::KeyboardKk, false, 0);
    keys[29].set_normal(KeyboardCodes::Keyboard8Asterisk, false, 1);
    keys[29].set_normal(KeyboardCodes::KeyboardUpArrow, false, 2);
    keys[30].set_normal(KeyboardCodes::KeyboardLl, false, 0);
    keys[30].set_normal(KeyboardCodes::Keyboard9OpenParens, false, 1);
    keys[30].set_normal(KeyboardCodes::KeyboardRightArrow, false, 2);
    keys[31].set_normal(KeyboardCodes::KeyboardSemiColon, false, 0);
    keys[31].set_normal(KeyboardCodes::Keyboard0CloseParens, false, 1);
    keys[32].set_normal(KeyboardCodes::KeyboardSingleDoubleQuote, false, 0);

    // Bottom Row
    keys[33].set_normal(KeyboardCodes::KeyboardNn, false, 0);
    keys[34].set_normal(KeyboardCodes::KeyboardMm, false, 0);
    keys[35].set_normal(KeyboardCodes::KeyboardCommaLess, false, 0);
    keys[36].set_normal(KeyboardCodes::KeyboardPeriodGreater, false, 0);
    keys[37].set_normal(KeyboardCodes::KeyboardSlashQuestion, false, 0);
    keys[38].set_normal(KeyboardCodes::KeyboardRightAlt, false, 0);

    // Thumb Row
    keys[39].set_normal(KeyboardCodes::KeyboardEnter, false, 0);
    keys[39].set_normal(KeyboardCodes::KeyboardEnter, false, 1);
    keys[40].set_normal_all([KeyboardCodes::Layer2; NUM_LAYERS], false);
    // keys[41].set_normal_all([KeyboardCodes::Layer2; NUM_LAYERS], false);
    set_slave(&mut keys[21..42]);

    ws.write(brightness(once(colors::CYAN), 48)).unwrap();
    let mut report = Report::default();

    loop {
        let mut external_keys = [0u8; 3];
        critical_section::with(|_| unsafe {
            external_keys = SLAVE_KEY_STATES;
        });
        let mut pos = 0;
        // for i in 0..6 {
        //     change_sel(&mut sel0, &mut sel1, &mut sel2, i);
        //     delay.delay_us(10);
        //     if i != 5 {
        //         keys[usize::from(4 * i as u16)].update_buf(adc.read(&mut a0).unwrap_or(69));
        //         keys[usize::from(4 * i as u16 + 1)].update_buf(adc.read(&mut a1).unwrap_or(69));
        //         keys[usize::from(4 * i as u16 + 2)].update_buf(adc.read(&mut a2).unwrap_or(69));
        //         keys[usize::from(4 * i as u16 + 3)].update_buf(adc.read(&mut a3).unwrap_or(69));
        //     } else {
        //         keys[20].update_buf(adc.read(&mut a0).unwrap_or(69));
        //     }
        // }
        for i in order {
            let chan = pos % 4;
            if chan == 0 {
                change_sel(&mut sel0, &mut sel1, &mut sel2, pos / 4);
                delay.delay_us(10);
            }
            match chan {
                0 => keys[i as usize].update_buf(adc.read(&mut a0).unwrap()),
                1 => keys[i as usize].update_buf(adc.read(&mut a1).unwrap()),
                2 => keys[i as usize].update_buf(adc.read(&mut a2).unwrap()),
                3 => keys[i as usize].update_buf(adc.read(&mut a3).unwrap()),
                _ => keys[i as usize].update_buf(0),
            }
            pos += 1;
        }
        let len = keys.len();
        let e_keys = &mut keys[21..len];
        for i in 0..21 {
            let a_idx = (i / 8) as usize;
            let b_idx = i % 8;
            let val = (external_keys[a_idx] >> b_idx) & 1;
            e_keys[i as usize].update_buf(val as u16);
        }
        let hid_report = report.generate_report(&mut keys);
        let buf_rep = BufferReport::default();
        match hid_report {
            Some(key_rep) => {
                critical_section::with(|_| unsafe {
                    let _ = USB_HID.as_mut().map(|hid| hid.push_input(&key_rep));
                    let _ = USB_HID2.as_mut().map(|hid| hid.push_input(&(buf_rep)));
                });
            }
            None => {}
        }
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

fn find_order(ary: &mut [u8]) {
    let mut new_ary = [0u8; NUM_KEYS as usize];
    for i in 0..ary.len() {
        for j in 0..ary.len() {
            if ary[j as usize] == i as u8 {
                new_ary[i as usize] = j as u8;
            }
        }
    }
    ary.copy_from_slice(&new_ary);
}

fn set_slave(keys: &mut [Key]) {
    for key in keys {
        key.set_slave();
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
        USB_HID.as_mut().unwrap().pull_raw_output(&mut [0; 64]).ok();
        let mut buf = [0u8; 64];
        USB_HID2.as_mut().unwrap().pull_raw_output(&mut buf).ok();
        if buf[0] == 5 {
            SLAVE_KEY_STATES[0] = buf[1];
            SLAVE_KEY_STATES[1] = buf[2];
            SLAVE_KEY_STATES[2] = buf[3];
        }
    }
}

// End of file
