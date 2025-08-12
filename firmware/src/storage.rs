use core::ops::{DerefMut, Range};

use defmt::{error, info, Format};
use embassy_futures::join::join;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex, signal::Signal,
};
use embassy_time::Timer;
use embedded_storage_async::nor_flash::NorFlash;
use sequential_storage::{
    cache::KeyCacheImpl,
    erase_all,
    map::{fetch_item, store_item, Value},
};

use crate::{
    codes::ScanCodeLayerStorage,
    keys::{NUM_KEYS, NUM_LAYERS},
};

pub static STORAGE_WRITE_CHANNEL: Channel<CriticalSectionRawMutex, (StorageKey, StorageItem), 10> =
    Channel::new();
pub static STORAGE_REQUEST_READ_LOCK: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());
pub static STORAGE_SIGNAL_READ: Signal<CriticalSectionRawMutex, StorageKey> = Signal::new();
pub static STORAGE_SIGNAL_ITEM: Signal<CriticalSectionRawMutex, Option<StorageItem>> =
    Signal::new();

pub static STORAGE_READ_CHANNEL: Channel<
    CriticalSectionRawMutex,
    (
        StorageKey,
        &'static mut Signal<CriticalSectionRawMutex, Option<StorageItem>>,
    ),
    10,
> = Channel::new();

type InternalStorageKey = u16;

#[derive(Debug, Clone, Copy, Format)]
pub enum StorageKey {
    StorageCheck,
    KeyScanCode { config_num: usize, layer: usize },
}

impl StorageKey {
    fn to_key(&self) -> InternalStorageKey {
        const SCAN_CODE_OFFSET: InternalStorageKey = 100;
        match self {
            StorageKey::StorageCheck => 0 as InternalStorageKey,
            StorageKey::KeyScanCode { config_num, layer } => {
                SCAN_CODE_OFFSET
                    + ((NUM_LAYERS * *config_num) as InternalStorageKey)
                    + *layer as InternalStorageKey
            }
        }
    }
}

pub struct Storage<S: NorFlash, K: KeyCacheImpl<InternalStorageKey> + 'static> {
    flash_range: Range<u32>,
    flash: Mutex<CriticalSectionRawMutex, (S, &'static mut K)>,
}

#[derive(Debug, Clone)]
pub enum StorageItem {
    Key(ScanCodeLayerStorage<NUM_KEYS>),
}

impl<S: NorFlash, K: KeyCacheImpl<InternalStorageKey> + 'static> Storage<S, K> {
    /// Returns Storage Struct. This method will clear
    /// the flash range if not intialized.
    pub async fn init(mut flash: S, flash_range: Range<u32>, cache: &'static mut K) -> Self {
        info!("Init Stage");
        let mut data_buffer = [0; 128];

        Timer::after_millis(10).await;
        // Check if the key value pair (0x0, 0x69) is in the map
        // If the pair is not in the map, it indicates that the
        // storage isn't initialized
        match fetch_item::<InternalStorageKey, u32, _>(
            &mut flash,
            flash_range.clone(),
            cache,
            &mut data_buffer,
            &StorageKey::StorageCheck.to_key(),
        )
        .await
        {
            Ok(res) => match res {
                Some(val) => {
                    if val != 0x69 {
                        erase_all(&mut flash, flash_range.clone()).await.unwrap();
                        store_item(
                            &mut flash,
                            flash_range.clone(),
                            cache,
                            &mut data_buffer,
                            &StorageKey::StorageCheck.to_key(),
                            &0x69u32,
                        )
                        .await
                        .unwrap();
                        info!("Key Exists, invalid value");
                    } else {
                        info!("Valid Storage");
                    }
                }
                None => {
                    erase_all(&mut flash, flash_range.clone()).await.unwrap();
                    store_item(
                        &mut flash,
                        flash_range.clone(),
                        cache,
                        &mut data_buffer,
                        &StorageKey::StorageCheck.to_key(),
                        &0x69u32,
                    )
                    .await
                    .unwrap();
                    info!("Key Doesn't exist");
                }
            },
            Err(_) => {
                info!("Error occured");
            }
        };
        Self {
            flash: Mutex::new((flash, cache)),
            flash_range,
        }
    }

    pub async fn store_item<'a, V: Value<'a>>(&self, key: InternalStorageKey, value: &V) {
        let mut buffer = [0; 256];
        let (flash, cache) = &mut *(self.flash.lock().await);
        match store_item(
            flash,
            self.flash_range.clone(),
            cache.deref_mut(),
            &mut buffer,
            &key,
            value,
        )
        .await
        {
            Ok(_) => info!("Item Stored succesfully"),
            Err(_) => error!("Failed to store item"),
        }
    }

    /// This method allows non-async methods to write to the storage in a async matter with
    /// channels. Method is not needed if all your functions can be run in async
    pub async fn run_storage(&self) {
        let write_loop = async {
            loop {
                let (key, value) = STORAGE_WRITE_CHANNEL.receive().await;
                info!("Writing key: {} | {}", key, key.to_key());
                let key_index = key.to_key();
                match value {
                    StorageItem::Key(code) => self.store_item(key_index, &code).await,
                };
            }
        };

        let read_loop = async {
            loop {
                let key = STORAGE_SIGNAL_READ.wait().await;
                let key_index = key.to_key();
                let mut buf = [0u8; 256];
                match key {
                    StorageKey::StorageCheck => {
                        STORAGE_SIGNAL_ITEM.signal(None);
                    }
                    StorageKey::KeyScanCode { .. } => {
                        match self
                            .get_item::<ScanCodeLayerStorage<NUM_KEYS>>(key_index, &mut buf)
                            .await
                            .unwrap()
                        {
                            Some(val) => {
                                STORAGE_SIGNAL_ITEM.signal(Some(StorageItem::Key(val)));
                            }
                            None => {
                                STORAGE_SIGNAL_ITEM.signal(None);
                            }
                        }
                    }
                }
            }
        };
        join(write_loop, read_loop).await;
    }

    pub async fn get_item<'a, V: Value<'a>>(
        &self,
        key: InternalStorageKey,
        buffer: &'a mut [u8],
    ) -> Result<Option<V>, sequential_storage::Error<S::Error>> {
        let (flash, cache) = &mut *(self.flash.lock().await);
        fetch_item(
            flash,
            self.flash_range.clone(),
            cache.deref_mut(),
            buffer,
            &key,
        )
        .await
    }

    pub async fn clear(&self) {
        let (flash, _) = &mut *(self.flash.lock().await);
        erase_all(flash, self.flash_range.clone()).await.unwrap();
    }
}

pub async fn get_item(key: StorageKey) -> Option<StorageItem> {
    info!("Requested {} | {}", key, key.to_key());
    let _lock = STORAGE_REQUEST_READ_LOCK.lock().await;
    STORAGE_SIGNAL_READ.signal(key);
    STORAGE_SIGNAL_ITEM.wait().await
}

pub async fn store_val(key: StorageKey, item: &StorageItem) {
    STORAGE_WRITE_CHANNEL.send((key, item.clone())).await;
}
