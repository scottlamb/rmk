use postcard::experimental::max_size::MaxSize;
use rmk_types::connection::ConnectionStatus;
use serde::{Deserialize, Serialize};

#[cfg(feature = "_ble")]
use crate::event::BatteryStatusEvent;
use crate::event::{KeyboardEvent, PointingEvent};
#[cfg(not(feature = "_ble"))]
use crate::event::TrackpadEvent;

#[cfg(feature = "_ble")]
pub mod ble;
pub mod central;
/// Common abstraction layer of split driver
pub(crate) mod driver;
pub mod peripheral;
#[cfg(feature = "rp2040")]
pub mod rp;
#[cfg(not(feature = "_ble"))]
pub mod serial;

/// Maximum size of a split message
pub const SPLIT_MESSAGE_MAX_SIZE: usize = SplitMessage::POSTCARD_MAX_SIZE + 4;

/// Message used from central & peripheral communication
#[repr(u8)]
// `Copy` is dropped: `Trackpad(TrackpadEvent)` carries a `heapless::Vec`
// inside `TrackpadFingers`, which isn't Copy. All call sites take
// `&SplitMessage` or move ownership explicitly.
#[derive(Serialize, Deserialize, Debug, Clone, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum SplitMessage {
    /// Keyboard event, from peripheral to central
    Key(KeyboardEvent),
    /// Pointing device event, from peripheral to central
    Pointing(PointingEvent),
    /// Multi-touch trackpad scan-cycle event, from peripheral to central.
    /// Carries per-finger absolute positions for the central's
    /// `TrackpadHidProcessor` to translate into HID reports.
    ///
    /// Gated on `not(_ble)`: the BLE split path's GATT service generates
    /// a `[u8; SPLIT_MESSAGE_MAX_SIZE]::default()` call that only works
    /// for arrays up to 32 bytes, and a 5-finger trackpad event pushes
    /// the message past that. BLE-paired trackpads are a follow-up.
    #[cfg(not(feature = "_ble"))]
    Trackpad(TrackpadEvent),
    /// Led state, on/off, from central to peripheral
    LedState(bool),
    /// `ConnectionStatus` snapshot of the central.
    /// Synced central → peripheral on every change.
    ConnectionStatus(ConnectionStatus),
    /// BLE Address, used in syncing address between central and peripheral
    Address([u8; 6]),
    /// Clear the saved peer info
    ClearPeer,
    /// Lock state led indicator from central to peripheral
    KeyboardIndicator(u8),
    /// Layer number from central to peripheral
    Layer(u8),
    /// WPM from central to peripheral
    #[cfg(feature = "display")]
    Wpm(u16),
    /// Modifier state from central to peripheral
    #[cfg(feature = "display")]
    Modifier(u8),
    /// Sleep state from central to peripheral
    #[cfg(feature = "display")]
    SleepState(bool),
    /// Battery status, from peripheral to central
    #[cfg(feature = "_ble")]
    BatteryStatus(BatteryStatusEvent),
}
