//! This example test the RP Pico on board LED.
//!
//! It does not work with the RP Pico W board. See wifi_blinky.rs.
#![no_std]
#![no_main]

use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_futures::join::{join, join3, join4};
use embassy_rp::adc::{self, Adc, Channel as AdcChannel, Config as AdcConfig};
use embassy_rp::flash::{Async, Flash};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::FLASH;
use embassy_rp::{bind_interrupts, peripherals, usb};

use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::class::hid::{HidReaderWriter, HidWriter, State};
use embassy_usb::{Builder, Config, Handler};
use sequential_storage::cache::NoCache;
use static_cell::StaticCell;
use tybeast_ones_he::com::Com;
use tybeast_ones_he::descriptor::{BufferReport, KeyboardReportNKRO, MouseReport, SlaveReport};
use tybeast_ones_he::keys::{Keys, NUM_KEYS};
use tybeast_ones_he::position::{HallEffectSensors, HeSwitch, KeySensors, KeyState, SlavePosition};
use tybeast_ones_he::report::Report;
use tybeast_ones_he::storage::Storage;
use usbd_hid::descriptor::SerializedDescriptor;
use {defmt_rtt as _, panic_probe as _};

const FLASH_START: u32 = 1024 * 1024;
const FLASH_END: u32 = FLASH_START + 4096 * 5;
const FLASH_SIZE: usize = 2 * 1024 * 1024;

static KEYS: Mutex<ThreadModeRawMutex, Keys<HeSwitch>> = Mutex::new(Keys::default());

static CACHE: StaticCell<NoCache> = StaticCell::new();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<peripherals::USB>;
    ADC_IRQ_FIFO => adc::InterruptHandler;
});

#[embassy_executor::task]
async fn storage_task(storage: Storage<Flash<'static, FLASH, Async, FLASH_SIZE>, NoCache>) {
    storage.run_storage().await;
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Device Started!");
    let p = embassy_rp::init(Default::default());
    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xa55, 0xa55);
    config.manufacturer = Some("Tybeast Corp.");
    config.product = Some("Tybeast Ones HE (Left)");
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
    let mut mouse_state = State::new();
    let mut com_state = State::new();

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
    // Create classes on the builder.
    let key_config = embassy_usb::class::hid::Config {
        report_descriptor: KeyboardReportNKRO::desc(),
        request_handler: None,
        poll_ms: 1,
        max_packet_size: 32,
    };
    let slave_config = embassy_usb::class::hid::Config {
        report_descriptor: SlaveReport::desc(),
        request_handler: None,
        poll_ms: 1,
        max_packet_size: 64,
    };
    let com_config = embassy_usb::class::hid::Config {
        report_descriptor: BufferReport::desc(),
        request_handler: None,
        poll_ms: 1,
        max_packet_size: 64,
    };
    let mouse_config = embassy_usb::class::hid::Config {
        report_descriptor: MouseReport::desc(),
        request_handler: None,
        poll_ms: 5,
        max_packet_size: 5,
    };

    let mut key_writer = HidWriter::<_, 29>::new(&mut builder, &mut key_state, key_config);
    let mut slave_hid =
        HidReaderWriter::<_, 10, 10>::new(&mut builder, &mut slave_state, slave_config);
    let (com_reader, com_writer) =
        HidReaderWriter::<_, 32, 32>::new(&mut builder, &mut com_state, com_config).split();
    let mut mouse_writer = HidWriter::<_, 5>::new(&mut builder, &mut mouse_state, mouse_config);

    // Build the builder.
    let mut usb = builder.build();
    let usb_fut = usb.run();

    let cache = CACHE.init_with(|| NoCache::new());
    let storage = Storage::init(
        Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH0),
        FLASH_START..FLASH_END,
        cache,
    )
    .await;
    _spawner.spawn(storage_task(storage)).unwrap();

    let slave_chan = Channel::new();
    // Sel Pins
    let sel0 = Output::new(p.PIN_2, Level::Low);
    let sel1 = Output::new(p.PIN_1, Level::Low);
    let sel2 = Output::new(p.PIN_0, Level::Low);

    // Adc
    let adc = Adc::new(p.ADC, Irqs, AdcConfig::default());
    let a3 = AdcChannel::new_pin(p.PIN_26, Pull::None);
    let a2 = AdcChannel::new_pin(p.PIN_27, Pull::None);
    let a1 = AdcChannel::new_pin(p.PIN_28, Pull::None);
    let a0 = AdcChannel::new_pin(p.PIN_29, Pull::None);

    let mut order: [usize; NUM_KEYS / 2] = [
        7, 14, 2, 18, 5, 0, 3, 11, 6, 1, 9, 4, 15, 19, 10, 13, 17, 8, 12, 16, 20,
    ];
    find_order(&mut order);
    let mut key_sensors = HallEffectSensors::new(
        [a0, a1, a2, a3],
        [sel0, sel1, sel2],
        adc,
        slave_chan.receiver(),
        order,
    );

    let mut keys = KEYS.lock().await;
    keys.load_keys_from_storage(0).await;
    keys.set_position_type_ranged(
        (NUM_KEYS / 2)..NUM_KEYS,
        HeSwitch::Slave(SlavePosition::DEFAULT),
    );
    keys.setup_positions(&mut key_sensors).await;

    drop(keys);

    let mut com = Com::new(&KEYS, com_reader, com_writer);

    let key_loop = async {
        let mut report = Report::new(key_sensors);
        loop {
            let (key_rep, mouse_rep);
            {
                let mut keys = KEYS.lock().await;
                (key_rep, mouse_rep) = report.generate_report(&mut keys).await;
                drop(keys);
            }
            let key_task = async {
                match key_rep {
                    Some(rep) => {
                        info!("Writing key report!");
                        key_writer.write_serialize(rep).await.unwrap();
                    }
                    None => {}
                }
            };
            let mouse_task = async {
                match mouse_rep {
                    Some(rep) => {
                        mouse_writer.write_serialize(rep).await.unwrap();
                    }
                    None => {}
                }
            };
            join(key_task, mouse_task).await;
            Timer::after_micros(200).await;
        }
    };

    let slave_loop = async {
        loop {
            let mut buf = [0u8; 32];
            slave_hid.read(&mut buf).await.unwrap();
            if buf[0] == 5 {
                let slave_rep = u32::from_le_bytes([buf[1], buf[2], buf[3], 0]);
                slave_chan.send(slave_rep).await;
            }
        }
    };
    join4(usb_fut, com.com_loop(), key_loop, slave_loop).await;
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
    let mut new_ary = [0usize; NUM_KEYS / 2];
    for i in 0..ary.len() {
        for j in 0..ary.len() {
            if ary[j as usize] == i {
                new_ary[i as usize] = j;
            }
        }
    }
    ary.copy_from_slice(&new_ary);
}
