//! Cross-split lighting transport.
//!
//! On a split keyboard, the simplest way to keep both halves visually
//! coherent is to render the whole keyboard's frame on the central
//! (where the runtime keymap, layer state, and modifier state live)
//! and ship the peripheral's portion as bytes. This avoids replicating
//! keymap data and layer-state-tracking machinery on the peripheral,
//! and means the peripheral automatically picks up runtime keymap
//! changes (Vial dynamic remaps, layer toggles, etc.) without any
//! extra protocol work.
//!
//! The mechanism: the central runs a second `LightingProcessor` whose
//! driver is [`SplitShipDriver`], which packs each rendered frame
//! into a `heapless::Vec<u8>` and pushes it to [`LIGHTING_FRAME_TX`].
//! The split TX loop consumes this channel and emits a
//! `SplitMessage::LightingFrame` over the wire. On the peripheral, the
//! split RX loop receives the message and pushes it to
//! [`LIGHTING_FRAME_RX`]; [`run_split_lighting_receiver`] consumes
//! that channel and writes to the peripheral's local WS2812 driver.
//!
//! Bandwidth: 29 LEDs × 3 bytes = 87 bytes per frame plus postcard
//! framing ≈ 100 bytes; at 115200 bps PIO serial that's ~9 ms on the
//! wire. Frames are sent only on render events (layer / modifier /
//! LED indicator changes), not per video frame, so the average rate
//! is whatever the user actually does — well under any noticeable
//! impact on the split link's other traffic (key events, pointer).

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use super::driver::{LightingDriver, Rgb};
use crate::split::{LightingFrameBytes, SPLIT_LIGHTING_MAX_LEDS};

/// Central → split TX. Capacity 1 — more queued frames waste memory
/// since the split TX loop will simply send the latest. The driver's
/// `write()` blocks until the previous frame has been picked up,
/// providing natural back-pressure when the link is busy.
pub static LIGHTING_FRAME_TX: Channel<CriticalSectionRawMutex, LightingFrameBytes, 1> = Channel::new();

/// Split RX → peripheral receiver task.
pub static LIGHTING_FRAME_RX: Channel<CriticalSectionRawMutex, LightingFrameBytes, 1> = Channel::new();

/// Lighting driver that, instead of writing to local hardware, packs
/// the rendered frame and ships it across the split link via
/// [`LIGHTING_FRAME_TX`]. Used by the central's second
/// `LightingProcessor` (the one configured for the peripheral half's
/// LED layout).
///
/// `N` is the number of LEDs on the peripheral. Must satisfy
/// `N <= SPLIT_LIGHTING_MAX_LEDS` (32); larger strips need either a
/// raised `SPLIT_LIGHTING_MAX_LEDS` or a chunked transport, neither
/// of which is implemented yet.
pub struct SplitShipDriver<const N: usize> {
    buf: [Rgb; N],
}

impl<const N: usize> SplitShipDriver<N> {
    pub const fn new() -> Self {
        // Forced compile-time check via an associated const that
        // depends on N. Without this, an N too large would silently
        // truncate at runtime in `write()`.
        let _ = Self::ASSERT_N_FITS;
        Self {
            buf: [Rgb::new(0, 0, 0); N],
        }
    }

    const ASSERT_N_FITS: () = assert!(
        N <= SPLIT_LIGHTING_MAX_LEDS,
        "SplitShipDriver N exceeds SPLIT_LIGHTING_MAX_LEDS \
         (raise SPLIT_LIGHTING_MAX_LEDS in rmk/src/split/mod.rs to fit your strip)"
    );
}

impl<const N: usize> Default for SplitShipDriver<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> LightingDriver for SplitShipDriver<N> {
    fn count(&self) -> usize {
        N
    }

    fn frame_buffer(&mut self) -> &mut [Rgb] {
        &mut self.buf
    }

    async fn write(&mut self) {
        let mut bytes = LightingFrameBytes::default();
        for color in &self.buf {
            // Semantic RGB on the wire; the peripheral's WS2812 driver
            // handles wire-format ordering (typically GRB) when it
            // pushes to hardware.
            let _ = bytes.0.push(color.r);
            let _ = bytes.0.push(color.g);
            let _ = bytes.0.push(color.b);
        }
        LIGHTING_FRAME_TX.send(bytes).await;
    }
}

/// Peripheral-side task. Reads frames from [`LIGHTING_FRAME_RX`] (where
/// the split RX loop in `crate::split::peripheral` deposits them) and
/// pushes them to the local WS2812 driver. Run alongside the rest of
/// the peripheral's tasks (matrix, trackpad, USB log, etc.).
///
/// The peripheral never needs a `LightingProcessor` of its own — this
/// task is the entire lighting subsystem on that side.
pub async fn run_split_lighting_receiver<D: LightingDriver>(mut driver: D) -> ! {
    loop {
        let bytes = LIGHTING_FRAME_RX.receive().await;
        let buf = driver.frame_buffer();
        let count = (bytes.len() / 3).min(buf.len());
        for i in 0..count {
            buf[i] = Rgb::new(bytes[i * 3], bytes[i * 3 + 1], bytes[i * 3 + 2]);
        }
        driver.write().await;
    }
}
