#![no_std]
#![no_main]

mod keys;
mod report;
use bsp::{
    entry,
    hal::{self, adc::AdcPin, Adc, Timer},
    pac::{self, interrupt::USBCTRL_IRQ, RESETS},
};
use cortex_m::{asm, interrupt::free as disable_interrupts};
use cortex_m::{
    delay::Delay,
    prelude::{_embedded_hal_adc_OneShot, _embedded_hal_blocking_delay_DelayMs},
};
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};
use panic_probe as _;
use report::KeyboardReportNKRO;
use rp_pico as bsp;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
// use sparkfun_pro_micro_rp2040 as bsp;

use rp_pico::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac::interrupt,
    sio::Sio,
    watchdog::Watchdog,
};

use usb_device::{
    class_prelude::{UsbBusAllocator, UsbClass},
    device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_hid::{
    descriptor::{KeyboardReport, SerializedDescriptor},
    hid_class::{
        HIDClass, HidClassSettings, HidCountryCode, HidProtocol, HidProtocolMode, HidSubClass,
        ProtocolModeConfig,
    },
};

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

static mut REPORT: KeyboardReportNKRO = KeyboardReportNKRO::default();

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

    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    // let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
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

    let usb_dev = UsbDeviceBuilder::new(&bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("tybeast")
            .product("dumb keypad")])
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

    let mut led_pin = pins.led.into_push_pull_output();
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let pin26 = AdcPin::new(pins.gpio26.into_floating_input()).unwrap();
    let pin27 = AdcPin::new(pins.gpio27.into_floating_input()).unwrap();
    led_pin.set_high().unwrap();
    let mut key0 = keys::Key::new(0x07, 2200.0, 3150.0);
    let mut key1 = keys::Key::new(0x09, 2250.0, 3150.0);
    loop {
        // let start = timer.get_counter().ticks();
        let mut adc_fifo = adc
            .build_fifo()
            .round_robin((&pin26, &pin27))
            .clock_divider(0, 0)
            .start();
        while adc_fifo.len() < 2 {}
        key0.update_buf(adc_fifo.read());
        key1.update_buf(adc_fifo.read());
        adc_fifo.stop();
        // let end = timer.get_counter().ticks();
        info!(
            "Key0: {}, Pressed: {} | Key1: {}, Pressed {}",
            key0.buffer,
            key0.get_key() != 0x00,
            key1.buffer,
            key1.get_key() != 0x00 // (end - start)
        );
        critical_section::with(|_| unsafe {
            REPORT.keycodes[0] = key0.get_key();
            REPORT.keycodes[1] = key1.get_key();
        });
        asm::wfi();
    }
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt()]
unsafe fn USBCTRL_IRQ() {
    USB_DEVICE
        .as_mut()
        .unwrap()
        .poll(&mut [USB_HID.as_mut().unwrap()]);
    USB_HID.as_mut().map(|hid| hid.push_input(&REPORT));
    USB_HID.as_mut().unwrap().poll();
}

// End of file
