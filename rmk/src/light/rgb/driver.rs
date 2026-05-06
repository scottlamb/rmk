//! Lighting driver trait and color type.

pub use smart_leds::RGB8 as Rgb;

/// Hardware backend that drives a strip of RGB LEDs.
///
/// The driver owns its frame buffer so the renderer can fill it
/// in place without an extra copy. The processor calls
/// [`frame_buffer`](Self::frame_buffer) to get a writable slice, hands
/// it to the renderer, then calls [`write`](Self::write) to push it
/// to the hardware.
pub trait LightingDriver {
    /// Number of LEDs this driver controls.
    fn count(&self) -> usize;

    /// Returns the driver's writable frame buffer. The slice is exactly
    /// `count()` long.
    fn frame_buffer(&mut self) -> &mut [Rgb];

    /// Push the current frame buffer to the hardware. May not return
    /// until the data has been clocked out (e.g. WS2812 DMA completion).
    async fn write(&mut self);
}
