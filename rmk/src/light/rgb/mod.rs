//! Per-key RGB lighting.
//!
//! Mirrors the `display` module's split: a [`LightingDriver`] owns the
//! hardware (PIO state machine + DMA on the RP2040, etc.) and a
//! [`LightingRenderer`] computes a frame from keyboard state. The
//! [`LightingProcessor`] glues them together by subscribing to the
//! same event channels the OLED display uses.
//!
//! # Design notes specific to WS2812-on-no-decoupling boards
//!
//! Measurements on the SoflePLUS2 (29 LEDs, no main-PCB bulk caps; see
//! the project memory for details) showed that on this hardware:
//!
//! * **Peak instantaneous current is the dominant cause of rail noise**,
//!   not frame-to-frame deltas. WS2812 chips PWM at ~400 Hz with their
//!   phases approximately aligned, so the rail sees the sum of all lit
//!   LEDs' peaks repeated 400 times per second.
//! * Per-frame net-delta caps and per-channel slew-rate limits do not
//!   meaningfully change ripple in this regime — they were dropped from
//!   the design.
//! * The two effective levers are: (1) brightness / peak-current ceiling,
//!   and (2) frame rate — fewer latches per second means fewer events
//!   even though the underlying PWM is unchanged.
//! * Skip-on-unchanged is essentially free and saves the data burst on
//!   static state, so it's enabled implicitly by the event-driven loop:
//!   the renderer is only called when state changes.
//!
//! # Feature flags
//!
//! * `rgb_lighting` — base traits and processor (this module).
//! * `ws2812_pio_rp` — RP2040 PIO-based WS2812 driver.

pub mod driver;
pub mod keymap_source;
pub mod processor;
pub mod renderer;

#[cfg(feature = "split")]
pub mod split_link;

#[cfg(feature = "ws2812_pio_rp")]
pub mod ws2812_pio_rp;

pub use driver::{LightingDriver, Rgb};
pub use keymap_source::{KeymapSource, StaticKeymapView};
pub use processor::LightingProcessor;
pub use renderer::{
    AnchorOverlay, CapsWordOverlay, KeyWashOverlay, LayerKeyRenderer, LayerPaletteRenderer, LayerTriggerOverlay,
    LightingContext, LightingRenderer, ModifierKeyOverlay, SolidRenderer,
};
#[cfg(feature = "split")]
pub use split_link::{LIGHTING_FRAME_RX, LIGHTING_FRAME_TX, SplitShipDriver, run_split_lighting_receiver};
