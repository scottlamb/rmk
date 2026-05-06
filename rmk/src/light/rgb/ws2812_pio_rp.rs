//! WS2812 driver for RP2040, wrapping `embassy_rp::pio_programs::ws2812`.
//!
//! Behind the `ws2812_pio_rp` feature.

use embassy_hal_internal::Peri;
use embassy_rp::dma;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::pio::{Common, Instance, PioPin, StateMachine};
pub use embassy_rp::pio_programs::ws2812::PioWs2812Program;
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812};

use super::driver::{LightingDriver, Rgb};

/// RP2040 PIO-based WS2812 driver.
///
/// Wraps [`PioWs2812`] from `embassy_rp::pio_programs::ws2812`. The
/// const generic `N` is the number of LEDs on this strip.
///
/// The frame buffer lives inside this struct so the renderer can fill
/// it in place without an intermediate copy. Construction takes the
/// same arguments as the underlying `PioWs2812::new`.
pub struct Ws2812PioRp<'d, P, const S: usize, const N: usize>
where
    P: Instance,
{
    inner: PioWs2812<'d, P, S, N, Grb>,
    buf: [Rgb; N],
}

impl<'d, P, const S: usize, const N: usize> Ws2812PioRp<'d, P, S, N>
where
    P: Instance,
{
    /// Construct a new driver. Defaults to GRB color order, which is
    /// what stock WS2812B uses.
    pub fn new<D: dma::ChannelInstance>(
        pio: &mut Common<'d, P>,
        sm: StateMachine<'d, P, S>,
        dma_ch: Peri<'d, D>,
        irq: impl Binding<D::Interrupt, dma::InterruptHandler<D>> + 'd,
        pin: Peri<'d, impl PioPin>,
        program: &PioWs2812Program<'d, P>,
    ) -> Self {
        Self {
            inner: PioWs2812::new(pio, sm, dma_ch, irq, pin, program),
            buf: [Rgb::new(0, 0, 0); N],
        }
    }
}

impl<P, const S: usize, const N: usize> LightingDriver for Ws2812PioRp<'_, P, S, N>
where
    P: Instance,
{
    fn count(&self) -> usize {
        N
    }

    fn frame_buffer(&mut self) -> &mut [Rgb] {
        &mut self.buf
    }

    async fn write(&mut self) {
        self.inner.write(&self.buf).await;
    }
}
