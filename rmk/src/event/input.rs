//! Input events for RMK
//!
//! This module contains all input-related events:
//! - Keyboard events (key press/release, rotary encoder)
//! - Modifier events
//! - Pointing device events (mouse, trackball, etc.)

use postcard::experimental::max_size::MaxSize;
use rmk_macro::event;
use rmk_types::modifier::ModifierCombination;
use serde::{Deserialize, Serialize};

use crate::input_device::rotary_encoder::Direction;

// ============================================================================
// Keyboard Events
// ============================================================================

/// `KeyboardEvent` is the event whose `KeyAction` is stored in the keymap.
///
/// `KeyboardEvent` is different from events from pointing devices,
/// events from pointing devices are processed directly by the corresponding processors,
/// while `KeyboardEvent` is processed by the keyboard with the keymap.
#[event(
    channel_size = crate::KEYBOARD_EVENT_CHANNEL_SIZE,
    pubs = crate::KEYBOARD_EVENT_PUB_SIZE,
    subs = crate::KEYBOARD_EVENT_SUB_SIZE
)]
#[derive(Serialize, Deserialize, Clone, Copy, Debug, MaxSize, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KeyboardEvent {
    pub pressed: bool,
    pub pos: KeyboardEventPos,
}

impl KeyboardEvent {
    pub fn key(row: u8, col: u8, pressed: bool) -> Self {
        Self {
            pressed,
            pos: KeyboardEventPos::Key(KeyPos { row, col }),
        }
    }

    pub fn rotary_encoder(id: u8, direction: Direction, pressed: bool) -> Self {
        Self {
            pressed,
            pos: KeyboardEventPos::RotaryEncoder(RotaryEncoderPos { id, direction }),
        }
    }
}

/// The position of the keyboard event.
///
/// The position can be either a key (row, col), or a rotary encoder (id, direction)
#[derive(Serialize, Deserialize, Clone, Copy, Debug, MaxSize, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum KeyboardEventPos {
    Key(KeyPos),
    RotaryEncoder(RotaryEncoderPos),
}

impl KeyboardEventPos {
    pub(crate) fn key_pos(col: u8, row: u8) -> Self {
        Self::Key(KeyPos { row, col })
    }

    pub(crate) fn rotary_encoder_pos(id: u8, direction: Direction) -> Self {
        Self::RotaryEncoder(RotaryEncoderPos { id, direction })
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, MaxSize, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KeyPos {
    pub row: u8,
    pub col: u8,
}

/// Event for rotary encoder
#[derive(Serialize, Deserialize, Clone, Copy, Debug, MaxSize, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RotaryEncoderPos {
    /// The id of the rotary encoder
    pub id: u8,
    /// The direction of the rotary encoder
    pub direction: Direction,
}

// ============================================================================
// Modifier Events
// ============================================================================

/// Modifier keys combination changed event
#[event(channel_size = crate::MODIFIER_EVENT_CHANNEL_SIZE, pubs = crate::MODIFIER_EVENT_PUB_SIZE, subs = crate::MODIFIER_EVENT_SUB_SIZE)]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ModifierEvent {
    pub modifier: ModifierCombination,
}

/// Mouse-button bitmap changed event. Published by the keyboard whenever
/// keymap-driven mouse buttons (`MouseBtn1..8`) press or release. Bit 0 is
/// `MouseBtn1` / left, etc. Mirrors the `buttons` field of the standard
/// mouse HID report.
///
/// Subscribers (e.g. [`crate::input_device::trackpad_hid::TrackpadHidProcessor`])
/// use it to surface button transitions even when no other input is in
/// flight — without it, a `MouseBtn1` press whose destination is a
/// trackpad's HID interface would only land on the next chip cycle, which
/// for an event-mode IQS5xx with no finger present is "never".
#[event(
    channel_size = crate::MOUSE_BUTTONS_EVENT_CHANNEL_SIZE,
    pubs = crate::MOUSE_BUTTONS_EVENT_PUB_SIZE,
    subs = crate::MOUSE_BUTTONS_EVENT_SUB_SIZE
)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MouseButtonsEvent {
    pub buttons: u8,
}

// ============================================================================
// Pointing Device Events
// ============================================================================

#[event(
    channel_size = crate::POINTING_EVENT_CHANNEL_SIZE,
    pubs = crate::POINTING_EVENT_PUB_SIZE,
    subs = crate::POINTING_EVENT_SUB_SIZE
)]
#[derive(Serialize, Deserialize, Clone, Debug, Copy, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PointingEvent(pub [AxisEvent; 3]);

#[derive(Serialize, Deserialize, Clone, Debug, Copy, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AxisEvent {
    /// The axis event value type, relative or absolute
    pub typ: AxisValType,
    /// The axis name
    pub axis: Axis,
    /// Value of the axis event
    pub value: i16,
}

#[derive(Serialize, Deserialize, Clone, Debug, Copy, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AxisValType {
    /// The axis value is relative
    Rel,
    /// The axis value is absolute
    Abs,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Axis {
    X,
    Y,
    Z,
    H,
    V,
    // .. More is allowed
}

/// Set the CPI (Resolution) of the pointing device
/// TODO: Make the channel size configurable
#[event(channel_size = 8, pubs = 2, subs = 2)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PointingSetCpiEvent {
    pub device_id: u8,
    pub cpi: u16,
}

// ============================================================================
// Trackpad Events
// ============================================================================

/// Maximum number of simultaneous finger contacts in a `TrackpadEvent`. Five
/// covers the IQS5xx family; trackpads supporting more would need this raised
/// (and the split message budget reconsidered).
pub const TRACKPAD_MAX_FINGERS: usize = 5;

/// One finger's absolute state within a `TrackpadEvent`.
#[derive(Serialize, Default, Deserialize, Clone, Debug, Copy, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TrackpadFinger {
    /// Stable per-touch identity: source-supplied index that survives from
    /// touch-down to lift, so consumers can map each finger to the same
    /// contact across frames without doing frame-to-frame proximity matching.
    /// For the IQS5xx this is the chip's 0..=4 slot index — that controller
    /// keeps each finger pinned to its slot (other slots may sentinel out
    /// independently), which means a finger that touches down in slot 2 stays
    /// at id=2 until it lifts. Drivers without per-finger persistence should
    /// still synthesise stable values here (e.g. by tracking previous-frame
    /// matches themselves).
    pub id: u8,
    /// Absolute X position in chip-resolution units.
    pub x: u16,
    /// Absolute Y position in chip-resolution units.
    pub y: u16,
    /// Touch strength (capacitance-derived). Higher = firmer contact.
    pub touch_strength: u16,
    /// Touch area, in chip-channel units. Larger = bigger blob (e.g. palm).
    pub area: u8,
    /// True iff this contact is on the surface and being tracked. Mirrors
    /// PTP's `tip_switch` bit. A `tip=false` record carries the contact's
    /// last known position so a downstream consumer can drop the contact
    /// from gesture state.
    ///
    /// Drivers for chips that natively expose a per-finger lift bit emit one
    /// `tip=false` record on lift and then drop the id from subsequent
    /// events. Drivers without that signal (e.g. IQS5xx in its single
    /// strength=0/area=0 transitional cycle) only need to ensure the lift
    /// is observable; a consumer that also tracks id-disappearance is
    /// robust to either form.
    pub tip: bool,
    /// True if the source considers this contact a real finger versus a
    /// rejected blob (palm, water, etc.). Maps directly to PTP's
    /// `confidence` bit. Drivers that do palm rejection internally and
    /// surface only accepted contacts (e.g. IQS5xx) always set this true.
    pub confidence: bool,
}

/// Newtype wrapping `heapless::Vec<TrackpadFinger, TRACKPAD_MAX_FINGERS>`
/// so we can supply postcard's `MaxSize` manually. The blanket `MaxSize` impl
/// postcard ships is for `heapless` 0.7, but this crate uses 0.9 — different
/// type, no impl. `Deref`/`DerefMut` mean callers see the inner `Vec`'s API
/// unchanged (`.len()`, `.iter()`, `.push()`, indexing, etc.).
#[derive(Serialize, Default, Deserialize, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TrackpadFingers(pub heapless::Vec<TrackpadFinger, TRACKPAD_MAX_FINGERS>);

impl MaxSize for TrackpadFingers {
    // 1-byte length varint (TRACKPAD_MAX_FINGERS == 5 fits in one byte) plus
    // up to TRACKPAD_MAX_FINGERS finger records.
    const POSTCARD_MAX_SIZE: usize = 1 + TRACKPAD_MAX_FINGERS * <TrackpadFinger as MaxSize>::POSTCARD_MAX_SIZE;
}

impl core::ops::Deref for TrackpadFingers {
    type Target = heapless::Vec<TrackpadFinger, TRACKPAD_MAX_FINGERS>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl core::ops::DerefMut for TrackpadFingers {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// Raw output of one trackpad scan cycle, before software gesture
/// interpretation. Drivers that read multi-touch frames publish this; a
/// downstream processor turns it into `PointingEvent` (cursor / scroll) or
/// a multi-finger PTP HID report.
#[event(
    channel_size = crate::TRACKPAD_EVENT_CHANNEL_SIZE,
    pubs = crate::TRACKPAD_EVENT_PUB_SIZE,
    subs = crate::TRACKPAD_EVENT_SUB_SIZE
)]
#[derive(Serialize, Default, Deserialize, Clone, Debug, MaxSize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TrackpadEvent {
    /// Per-finger absolute state. `fingers.len()` is the active finger count.
    pub fingers: TrackpadFingers,
}
