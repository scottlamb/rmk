use postcard::experimental::max_size::MaxSize;
use rmk_types::connection::ConnectionStatus;
use serde::{Deserialize, Serialize};

#[cfg(feature = "_ble")]
use crate::event::BatteryStatusEvent;
use crate::event::{KeyboardEvent, PointingEvent};
#[cfg(not(feature = "_ble"))]
use crate::event::TrackpadEvent;

/// Maximum number of LEDs in a single `SplitMessage::LightingFrame`.
/// Covers all currently-supported keyboards with comfortable slack;
/// raise if you have a larger strip on a single half.
#[cfg(feature = "rgb_lighting")]
pub const SPLIT_LIGHTING_MAX_LEDS: usize = 32;

/// Maximum bytes per lighting frame (RGB triples).
#[cfg(feature = "rgb_lighting")]
pub const SPLIT_LIGHTING_MAX_BYTES: usize = SPLIT_LIGHTING_MAX_LEDS * 3;

/// Newtype wrapper for the `SplitMessage::LightingFrame` payload.
/// Necessary because `#[derive(MaxSize)]` doesn't understand
/// `heapless::Vec`; a manual `MaxSize` impl on this wrapper threads
/// the size through the enum's derive. Same pattern as
/// `TrackpadFingers` in `event/input.rs`.
#[cfg(feature = "rgb_lighting")]
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LightingFrameBytes(pub heapless::Vec<u8, SPLIT_LIGHTING_MAX_BYTES>);

#[cfg(feature = "rgb_lighting")]
impl MaxSize for LightingFrameBytes {
    // Postcard length-prefix is a varint; for SPLIT_LIGHTING_MAX_BYTES
    // up to 16383 the prefix is at most 2 bytes, plus the payload.
    const POSTCARD_MAX_SIZE: usize = 2 + SPLIT_LIGHTING_MAX_BYTES;
}

#[cfg(feature = "rgb_lighting")]
impl core::ops::Deref for LightingFrameBytes {
    type Target = heapless::Vec<u8, SPLIT_LIGHTING_MAX_BYTES>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[cfg(feature = "rgb_lighting")]
impl core::ops::DerefMut for LightingFrameBytes {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

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
// `Copy` is dropped: both `Trackpad(TrackpadEvent)` and
// `LightingFrame(LightingFrameBytes)` carry a `heapless::Vec` (in
// `TrackpadFingers` / `LightingFrameBytes`), which isn't Copy. All call
// sites take `&SplitMessage` or move/clone explicitly.
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
    /// Per-LED RGB frame for the peripheral's strip, central to peripheral.
    /// Bytes are semantic RGB triples (`[r0, g0, b0, r1, g1, b1, …]`);
    /// the peripheral's WS2812 driver handles wire-format ordering
    /// (typically GRB). Sent on every render the central does for
    /// the peripheral's renderer — infrequent, since renders are
    /// event-driven (layer / modifier / LED indicator changes).
    ///
    /// Gated on `not(_ble)` because the BLE split path's GATT service
    /// generates a `[u8; SPLIT_MESSAGE_MAX_SIZE]::default()` call, which
    /// only works for arrays up to 32 bytes. The lighting payload pushes
    /// the message past that. BLE-paired RGB peripherals are a follow-up.
    #[cfg(all(feature = "rgb_lighting", not(feature = "_ble")))]
    LightingFrame(LightingFrameBytes),
}
