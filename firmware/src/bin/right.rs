//! This example test the RP Pico on board LED.
//!
//! It does not work with the RP Pico W board. See wifi_blinky.rs.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::adc::{self, Adc, Channel, Config as AdcConfig};
use embassy_rp::gpio::{Pin, Pull};
use embassy_rp::{bind_interrupts, gpio, peripherals, usb};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use keyboard::descriptor::{BufferReport, KeyboardReportNKRO, MouseReport, SlaveKeyReport};
use keyboard::keys::Keys;

use embassy_rp::usb::Driver;
use embassy_usb::class::hid::{HidReaderWriter, HidWriter, State};
use embassy_usb::{Builder, Config, Handler};
use gpio::{Level, Output};
use keyboard::report::Report;
use tybeast_ones_he::descriptor::{BufferReport, SlaveKeyReport};
use usbd_hid::descriptor::SerializedDescriptor;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<peripherals::USB>;
    ADC_IRQ_FIFO => adc::InterruptHandler;
});

static MUX: Mutex<CriticalSectionRawMutex, [u8; 3]> = Mutex::new([0u8; 3]);

const SCROLL_TIME: u64 = 500;
const MOUSE_POINTER_TIME: u64 = 10;

const NUM_KEYS: usize = 21;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Device Started!");
    let p = embassy_rp::init(Default::default());
    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0x727, 0x727);
    config.manufacturer = Some("Tybeast Corp.");
    config.product = Some("Tybeast Ones (Right)");
    config.max_power = 500;
    config.max_packet_size_0 = 64;
    config.composite_with_iads = true;
    config.device_class = 0xef;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut device_handler = MyDeviceHandler::new();

    let mut key_state = State::new();
    let mut slave_state = State::new();
    let mut com_state = State::new();
    let mut mouse_state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    builder.handler(&mut device_handler);

    // Create classes on the builder.
    let key_config = embassy_usb::class::hid::Config {
        report_descriptor: SlaveKeyReport::desc(),
        request_handler: None,
        poll_ms: 1,
        max_packet_size: 32,
    };
    let com_config = embassy_usb::class::hid::Config {
        report_descriptor: BufferReport::desc(),
        request_handler: None,
        poll_ms: 60,
        max_packet_size: 64,
    };

    let mut key_writer = HidWriter::<_, 29>::new(&mut builder, &mut key_state, key_config);
    let com_hid = HidReaderWriter::<_, 32, 64>::new(&mut builder, &mut com_state, com_config);

    let (mut c_reader, mut c_writer) = com_hid.split();

    // Build the builder.
    let mut usb = builder.build();
    let usb_fut = usb.run();

    // Sel Pins
    let mut sel0 = Output::new(p.PIN_0, Level::Low);
    let mut sel1 = Output::new(p.PIN_1, Level::Low);
    let mut sel2 = Output::new(p.PIN_2, Level::Low);

    // Adc
    let mut adc = Adc::new(p.ADC, Irqs, AdcConfig::default());
    let mut a3 = Channel::new_pin(p.PIN_26, Pull::None);
    let mut a2 = Channel::new_pin(p.PIN_27, Pull::None);
    let mut a1 = Channel::new_pin(p.PIN_28, Pull::None);
    let mut a0 = Channel::new_pin(p.PIN_29, Pull::None);

    let mut order: [usize; NUM_KEYS] = [
        4, 5, 18, 2, 14, 7, 0, 9, 1, 6, 11, 3, 12, 17, 13, 10, 19, 15, 20, 16, 8,
    ];
    find_order(&mut order);

    let mut keys = Keys::<NUM_KEYS>::default();

    let mut setup = false;
    while !setup {
        let mut pos = 0;
        setup = true;
        for i in order {
            // Equivalent to pos % 4
            let chan = pos & 0b11;
            if chan == 0 {
                // equivalent to pos / 4
                change_sel(&mut sel0, &mut sel1, &mut sel2, pos >> 2);
            }
            let res = match chan {
                0 => keys.setup(i, adc.read(&mut a0).await.unwrap()),
                1 => keys.setup(i, adc.read(&mut a1).await.unwrap()),
                2 => keys.setup(i, adc.read(&mut a2).await.unwrap()),
                3 => keys.setup(i, adc.read(&mut a3).await.unwrap()),
                _ => false,
            };
            setup = setup && res;
            pos += 1;
        }
    }
    let mut report = SlaveKeyReport::default();

    // Main keyboard loop
    let usb_key_in = async {
        loop {
            let mut pos = 0;
            for i in order {
                let chan = pos % 4;
                if chan == 0 {
                    change_sel(&mut sel0, &mut sel1, &mut sel2, pos / 4);
                    Timer::after_micros(1).await;
                }
                match chan {
                    0 => keys.update_buf(i, adc.read(&mut a0).await.unwrap()),
                    1 => keys.update_buf(i, adc.read(&mut a1).await.unwrap()),
                    2 => keys.update_buf(i, adc.read(&mut a2).await.unwrap()),
                    3 => keys.update_buf(i, adc.read(&mut a3).await.unwrap()),
                    _ => {}
                }
                pos += 1;
            }
            let key_report = report.generate_report(&mut keys);
            match key_report {
                Some(report) => {
                    key_writer.write_serialize(&report).await.unwrap();
                }
                None => {}
            }
        }
    };

    join(usb_key_in, usb_fut).await;
}

struct MyDeviceHandler {
    configured: AtomicBool,
}

impl MyDeviceHandler {
    fn new() -> Self {
        MyDeviceHandler {
            configured: AtomicBool::new(false),
        }
    }
}

impl Handler for MyDeviceHandler {
    fn enabled(&mut self, enabled: bool) {
        self.configured.store(false, Ordering::Relaxed);
        if enabled {
            info!("Device enabled");
        } else {
            info!("Device disabled");
        }
    }

    fn reset(&mut self) {
        self.configured.store(false, Ordering::Relaxed);
        info!("Bus reset, the Vbus current limit is 100mA");
    }

    fn addressed(&mut self, addr: u8) {
        self.configured.store(false, Ordering::Relaxed);
        info!("USB address set to: {}", addr);
    }

    fn configured(&mut self, configured: bool) {
        self.configured.store(configured, Ordering::Relaxed);
        if configured {
            info!(
                "Device configured, it may now draw up to the configured current limit from Vbus."
            )
        } else {
            info!("Device is no longer configured, the Vbus current limit is 100mA.");
        }
    }
}

fn find_order(ary: &mut [usize]) {
    let mut new_ary = [0usize; 21 as usize];
    for i in 0..ary.len() {
        for j in 0..ary.len() {
            if ary[j as usize] == i {
                new_ary[i as usize] = j;
            }
        }
    }
    ary.copy_from_slice(&new_ary);
}

fn change_sel(sel0: &mut Output, sel1: &mut Output, sel2: &mut Output, num: u8) {
    match num {
        0 => {
            sel0.set_low();
            sel1.set_low();
            sel2.set_low();
        }
        1 => {
            sel0.set_high();
            sel1.set_low();
            sel2.set_low();
        }
        2 => {
            sel0.set_low();
            sel1.set_high();
            sel2.set_low();
        }
        3 => {
            sel0.set_high();
            sel1.set_high();
            sel2.set_low();
        }
        4 => {
            sel0.set_low();
            sel1.set_low();
            sel2.set_high();
        }
        5 => {
            sel0.set_high();
            sel1.set_low();
            sel2.set_high();
        }
        _ => {
            sel0.set_low();
            sel1.set_low();
            sel2.set_low();
        }
    }
}
