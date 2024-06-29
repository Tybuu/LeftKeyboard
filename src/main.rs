#![no_std]
#![no_main]

mod keys;
mod report;

use cortex_m::{
    asm,
    delay::Delay,
    prelude::{_embedded_hal_adc_OneShot, _embedded_hal_blocking_delay_DelayMs},
};
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{
    digital::{InputPin, OutputPin, StatefulOutputPin},
    i2c::I2c,
};
use fugit::RateExtU32;
use hal::clocks::init_clocks_and_plls;
use hal::gpio::bank0::{Gpio0, Gpio1, Gpio2};
use hal::gpio::{FunctionSio, Pin, Pins, PullType, SioOutput};
use hal::{adc::AdcPin, Clock};
use hal::{
    pac::{self, interrupt},
    Adc, Sio, Timer, Watchdog,
};
use keys::Key;
use panic_probe as _;
use report::{BufferReport, KeyboardReportNKRO};
use rp2040_hal as hal;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
// use sparkfun_pro_micro_rp2040 as bsp;

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

static mut USB_HID2: Option<HIDClass<hal::usb::UsbBus>> = None;

static mut REPORT: KeyboardReportNKRO = KeyboardReportNKRO::default();

static mut BUFFER: BufferReport = BufferReport::default();

static mut NUM: u8 = 0;

static mut KEYS: [Key; 21] = [Key::default(); 21];

#[hal::entry]
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

    let pins = Pins::new(
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

    let mut sel0 = pins.gpio2.into_push_pull_output();
    let mut sel1 = pins.gpio1.into_push_pull_output();
    let mut sel2 = pins.gpio0.into_push_pull_output();

    sel0.set_low().unwrap();
    sel1.set_low().unwrap();
    sel2.set_low().unwrap();

    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut a0 = AdcPin::new(pins.gpio26).unwrap();
    let mut a1 = AdcPin::new(pins.gpio27).unwrap();
    let mut a2 = AdcPin::new(pins.gpio28).unwrap();
    let mut a3 = AdcPin::new(pins.gpio29).unwrap();

    let mut keys = [Key::new(0x00, 1400.0, 2000.0); 21];

    loop {
        let start = timer.get_counter().ticks();
        for i in 0..6 {
            change_sel(&mut sel0, &mut sel1, &mut sel2, i);
            delay.delay_us(10);
            if i != 5 {
                // let mut adc_fifo = adc
                //     .build_fifo()
                //     .round_robin((&a0, &a1, &a2, &a3))
                //     .clock_divider(0, 0)
                //     .start();
                // while adc_fifo.len() < 4 {}
                // keys[usize::from(4 * i as u16)].update_buf(adc_fifo.read());
                // keys[usize::from(4 * i as u16 + 1)].update_buf(adc_fifo.read());
                // keys[usize::from(4 * i as u16 + 2)].update_buf(adc_fifo.read());
                // keys[usize::from(4 * i as u16 + 3)].update_buf(adc_fifo.read());
                // adc_fifo.stop();
                let val: u16 = adc.read(&mut a0).unwrap();
                keys[usize::from(4 * i as u16)].update_buf(val);
                keys[usize::from(4 * i as u16 + 1)].update_buf(val);
                keys[usize::from(4 * i as u16 + 2)].update_buf(val);
                keys[usize::from(4 * i as u16 + 3)].update_buf(val);
            } else {
                // let mut adc_fifo = adc
                //     .build_fifo()
                //     .round_robin(&a3)
                //     .clock_divider(0, 0)
                //     .start();
                // while adc_fifo.len() < 1 {}
                // keys[20].update_buf(adc_fifo.read());
                // adc_fifo.stop();
                keys[20].update_buf(adc.read(&mut a0).unwrap());
            }
        }
        critical_section::with(|_| unsafe {
            for i in 0..16 {
                BUFFER.key_report0[usize::from(i as u16 * 2)] =
                    ((keys[usize::from(i as u16)].get_buf() & !(0b11111111 as u16)) >> 8) as u8;
                BUFFER.key_report0[usize::from(i as u16 * 2 + 1)] =
                    (keys[usize::from(i as u16)].get_buf() & 0b11111111 as u16) as u8;
            }

            for i in 16..21 {
                BUFFER.key_report1[usize::from((i as u16 - 16) * 2)] =
                    ((keys[usize::from(i as u16)].get_buf() & !(0b11111111 as u16)) >> 8) as u8;
                BUFFER.key_report1[usize::from((i as u16 - 16) * 2 + 1)] =
                    (keys[usize::from(i as u16)].get_buf() & 0b11111111 as u16) as u8;
            }
            BUFFER.key_report1[10] = a0.channel();
            BUFFER.key_report1[11] = a1.channel();
            BUFFER.key_report1[12] = a2.channel();
            BUFFER.key_report1[13] = a3.channel();
        });
        asm::wfi();
        let end = timer.get_counter().ticks();
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
}

// End of file
