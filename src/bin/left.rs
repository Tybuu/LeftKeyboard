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
    hid::ModifierPosition,
    keys::{Key, ScanCodeType},
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

static mut REPORT: KeyboardReportNKRO = KeyboardReportNKRO::default();

static mut BUFFER: BufferReport = BufferReport::default();

static mut KEYS: [Key; 21] = [Key::default(); 21];

static mut MOD_REPORT: u8 = 0u8;

static mut EXTERNAL_LAYER: u8 = 0u8;

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
    let mut keys = [Key::new(
        [0x00; keys::NUM_LAYERS],
        [ScanCodeType::Letter; keys::NUM_LAYERS],
        1400.0,
        2000.0,
    ); 21];

    // Initialize Keys
    // Rows go from top to bottom
    // First Row left to right
    keys[0].bit_pos[0] = KeyboardUsage::KeyboardEscape as u8;
    keys[0].bit_pos[1] = KeyboardUsage::KeyboardTab as u8;
    keys[1].bit_pos[0] = KeyboardUsage::KeyboardQq as u8;
    keys[1].bit_pos[1] = KeyboardUsage::KeyboardBacktickTilde as u8;
    keys[1].bit_pos[2] = KeyboardUsage::KeyboardF1 as u8;
    keys[2].bit_pos[0] = KeyboardUsage::KeyboardWw as u8;
    keys[2].bit_pos[2] = KeyboardUsage::KeyboardF2 as u8;
    keys[3].bit_pos[0] = KeyboardUsage::KeyboardEe as u8;
    keys[3].bit_pos[2] = KeyboardUsage::KeyboardF3 as u8;
    keys[4].bit_pos[0] = KeyboardUsage::KeyboardRr as u8;
    keys[4].bit_pos[2] = KeyboardUsage::KeyboardF4 as u8;
    keys[5].bit_pos[0] = KeyboardUsage::KeyboardTt as u8;
    keys[5].bit_pos[2] = KeyboardUsage::KeyboardF5 as u8;

    // Middle Row
    keys[6].scan_code_type[0] = ScanCodeType::Modifier;
    keys[6].scan_code_type[1] = ScanCodeType::Modifier;
    keys[6].scan_code_type[2] = ScanCodeType::Modifier;
    keys[6].bit_pos[0] = ModifierPosition::LeftCtrl as u8;
    keys[6].bit_pos[1] = ModifierPosition::LeftCtrl as u8;
    keys[7].bit_pos[0] = KeyboardUsage::KeyboardAa as u8;
    keys[7].bit_pos[1] = KeyboardUsage::Keyboard1Exclamation as u8;
    keys[8].bit_pos[0] = KeyboardUsage::KeyboardSs as u8;
    keys[8].bit_pos[1] = KeyboardUsage::Keyboard2At as u8;
    keys[9].bit_pos[0] = KeyboardUsage::KeyboardDd as u8;
    keys[9].bit_pos[1] = KeyboardUsage::Keyboard3Hash as u8;
    keys[10].bit_pos[0] = KeyboardUsage::KeyboardFf as u8;
    keys[10].bit_pos[1] = KeyboardUsage::Keyboard4Dollar as u8;
    keys[11].bit_pos[0] = KeyboardUsage::KeyboardGg as u8;
    keys[11].bit_pos[1] = KeyboardUsage::Keyboard5Percent as u8;

    // Bottom Row
    keys[12].scan_code_type[0] = ScanCodeType::Modifier;
    keys[12].scan_code_type[1] = ScanCodeType::Modifier;
    keys[12].scan_code_type[2] = ScanCodeType::Modifier;
    keys[12].bit_pos[0] = ModifierPosition::LeftShift as u8;
    keys[12].bit_pos[1] = ModifierPosition::LeftShift as u8;
    keys[12].bit_pos[2] = ModifierPosition::LeftShift as u8;
    keys[13].bit_pos[0] = KeyboardUsage::KeyboardZz as u8;
    keys[14].bit_pos[0] = KeyboardUsage::KeyboardXx as u8;
    keys[15].bit_pos[0] = KeyboardUsage::KeyboardCc as u8;
    keys[16].bit_pos[0] = KeyboardUsage::KeyboardVv as u8;
    keys[17].bit_pos[0] = KeyboardUsage::KeyboardBb as u8;

    // Thumb Row
    keys[18].scan_code_type[0] = ScanCodeType::Modifier;
    keys[18].scan_code_type[1] = ScanCodeType::Modifier;
    keys[18].scan_code_type[2] = ScanCodeType::Modifier;
    keys[18].bit_pos[0] = ModifierPosition::LeftGui as u8;
    keys[18].bit_pos[1] = ModifierPosition::LeftGui as u8;
    keys[19].scan_code_type[0] = ScanCodeType::Layer;
    keys[19].scan_code_type[1] = ScanCodeType::Layer;
    keys[19].scan_code_type[2] = ScanCodeType::Layer;
    keys[19].bit_pos[0] = 1;
    keys[19].bit_pos[1] = 1;
    keys[19].bit_pos[2] = 1;
    keys[20].bit_pos[0] = KeyboardUsage::KeyboardSpacebar as u8;
    keys[20].bit_pos[1] = KeyboardUsage::KeyboardSpacebar as u8;

    ws.write(brightness(once(colors::CYAN), 48)).unwrap();
    let mut report = Report::default();

    loop {
        let mut external_layer = 0u8;
        let mut external_modifier = 0u8;
        critical_section::with(|_| unsafe {
            external_layer = EXTERNAL_LAYER;
            external_modifier = MOD_REPORT;
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
        let hid_report =
            report.generate_report(&mut keys, external_layer as usize, external_modifier);
        match hid_report {
            Some((key_rep, buf_rep)) => {
                critical_section::with(|_| unsafe {
                    let _ = USB_HID.as_mut().map(|hid| hid.push_input(&key_rep));
                    let _ = USB_HID2.as_mut().map(|hid| hid.push_input(&buf_rep));
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
            MOD_REPORT = buf[1];
            EXTERNAL_LAYER = buf[2];
        }
    }
}

// End of file
