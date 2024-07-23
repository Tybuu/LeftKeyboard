#![no_std]
#![no_main]

use adafruit_kb2040 as bsp;
use bsp::{
    entry,
    hal::{
        adc::AdcPin,
        gpio::{
            bank0::{Gpio0, Gpio1, Gpio2},
            FunctionSio, Pin, PullType, SioOutput,
        },
        Adc, Timer,
    },
    pac,
};
use core::iter::once;
use cortex_m::prelude::_embedded_hal_adc_OneShot;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use keyboard::{hid::SlaveKeyReport, keys::Key};
use panic_probe as _;
use rp2040_hal::{pio::PIOExt, usb::UsbBus};

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
use smart_leds_trait::SmartLedsWrite;
use usb_device::{
    class_prelude::{UsbBusAllocator, UsbClass},
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_hid::{
    descriptor::SerializedDescriptor,
    hid_class::{
        HIDClass, HidClassSettings, HidCountryCode, HidProtocol, HidSubClass, ProtocolModeConfig,
    },
};
use ws2812_pio::Ws2812;

const NUM_KEYS: u32 = 21;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<UsbBus>> = None;

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

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
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

    let usb_hid = HIDClass::new_with_settings(bus_ref, SlaveKeyReport::desc(), 1, usb_settings);
    unsafe {
        USB_HID = Some(usb_hid);
    }

    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x0727, 0x0727))
        .strings(&[StringDescriptors::default()
            .manufacturer("tybeast")
            .product("Right Tybeast Ones")])
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
        pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
    };

    let mut sel0 = pins.tx.into_push_pull_output();
    let mut sel1 = pins.rx.into_push_pull_output();
    let mut sel2 = pins.d2.into_push_pull_output();

    // let mut s_pins = SelPins::new(sel0, sel1, sel2);

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    // Pins names are matched to the names in the schematic (i trolled)
    let mut a3 = AdcPin::new(pins.a0).unwrap();
    let mut a2 = AdcPin::new(pins.a1).unwrap();
    let mut a1 = AdcPin::new(pins.a2).unwrap();
    let mut a0 = AdcPin::new(pins.a3).unwrap();

    let mut order: [u8; NUM_KEYS as usize] = [
        4, 5, 18, 2, 14, 7, 0, 9, 1, 6, 11, 3, 12, 17, 13, 10, 19, 15, 20, 16, 8,
    ];
    find_order(&mut order);

    let mut keys = [Key::default(); 21];

    ws.write(brightness(once(colors::CYAN), 48)).unwrap();
    let mut report = SlaveKeyReport::default();

    loop {
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
        let mut pos = 0;
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
        let hid_report = report.generate_report(&mut keys);
        match hid_report {
            Some(key_rep) => {
                critical_section::with(|_| unsafe {
                    let _ = USB_HID.as_mut().map(|hid| hid.push_input(&key_rep));
                });
            }
            None => {}
        }
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
        .poll(&mut [USB_HID.as_mut().unwrap()])
    {
        USB_HID.as_mut().unwrap().poll();
        USB_HID.as_mut().unwrap().pull_raw_output(&mut [0; 64]).ok();
    }
}

// End of file
