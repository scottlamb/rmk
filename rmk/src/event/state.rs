//! Keyboard state events

use rmk_macro::event;
use rmk_types::led_indicator::LedIndicator;

/// Active layer changed event
#[event(channel_size = crate::LAYER_CHANGE_EVENT_CHANNEL_SIZE, pubs = crate::LAYER_CHANGE_EVENT_PUB_SIZE, subs = crate::LAYER_CHANGE_EVENT_SUB_SIZE)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LayerChangeEvent(pub u8);

impl LayerChangeEvent {
    pub fn new(layer: u8) -> Self {
        Self(layer)
    }
}

impl_payload_wrapper!(LayerChangeEvent, u8);

/// WPM updated event
#[event(channel_size = crate::WPM_UPDATE_EVENT_CHANNEL_SIZE, pubs = crate::WPM_UPDATE_EVENT_PUB_SIZE, subs = crate::WPM_UPDATE_EVENT_SUB_SIZE)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WpmUpdateEvent(pub u16);

impl WpmUpdateEvent {
    pub fn new(wpm: u16) -> Self {
        Self(wpm)
    }
}

impl_payload_wrapper!(WpmUpdateEvent, u16);

/// LED indicator state changed event
#[event(channel_size = crate::LED_INDICATOR_EVENT_CHANNEL_SIZE, pubs = crate::LED_INDICATOR_EVENT_PUB_SIZE, subs = crate::LED_INDICATOR_EVENT_SUB_SIZE)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LedIndicatorEvent(pub LedIndicator);

impl LedIndicatorEvent {
    pub fn new(indicator: LedIndicator) -> Self {
        Self(indicator)
    }
}

impl_payload_wrapper!(LedIndicatorEvent, LedIndicator);

/// Sleep state changed event
#[event(channel_size = crate::SLEEP_STATE_EVENT_CHANNEL_SIZE, pubs = crate::SLEEP_STATE_EVENT_PUB_SIZE, subs = crate::SLEEP_STATE_EVENT_SUB_SIZE)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SleepStateEvent(pub bool);

impl SleepStateEvent {
    pub fn new(sleeping: bool) -> Self {
        Self(sleeping)
    }
}

impl_payload_wrapper!(SleepStateEvent, bool);

/// Caps Word activation changed event. Fires when the keyboard's
/// internal caps-word state machine activates, deactivates, or times
/// out at the next keypress. Subscribers (RGB lighting overlays that
/// highlight Shift keys) get a state signal that's independent of the
/// host caps-lock LED indicator (caps-word is firmware-only — it
/// shifts subsequent letters by injecting Shift into the resolved
/// modifiers, without ever toggling host caps-lock).
#[event(channel_size = crate::CAPS_WORD_EVENT_CHANNEL_SIZE, pubs = crate::CAPS_WORD_EVENT_PUB_SIZE, subs = crate::CAPS_WORD_EVENT_SUB_SIZE)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CapsWordEvent(pub bool);

impl CapsWordEvent {
    pub fn new(active: bool) -> Self {
        Self(active)
    }
}

impl_payload_wrapper!(CapsWordEvent, bool);
