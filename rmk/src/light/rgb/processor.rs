//! [`LightingProcessor`] — subscribes to keyboard state events and
//! drives a [`LightingDriver`] via a [`LightingRenderer`].

use rmk_macro::processor;

use super::driver::LightingDriver;
use super::renderer::{LightingContext, LightingRenderer};
use crate::core_traits::Runnable;
use crate::event::{CapsWordEvent, EventSubscriber, LayerChangeEvent, LedIndicatorEvent, ModifierEvent};
use crate::processor::Processor;

/// Drives an RGB strip from keyboard state.
///
/// Subscribes to [`LayerChangeEvent`], [`ModifierEvent`], and
/// [`LedIndicatorEvent`]. Each event updates the internal
/// [`LightingContext`] and triggers a render-then-write. Static
/// state (no incoming events) produces no writes — which means the
/// underlying WS2812 chips run their PWM cycles undisturbed by
/// latch-induced phase realignment, an important property on hardware
/// without bulk decoupling (see the module-level doc for measurement
/// notes).
///
/// # Generics
///
/// * `D` — driver, must implement [`LightingDriver`].
/// * `R` — renderer, must implement [`LightingRenderer`].
///
/// # Example
///
/// ```rust,ignore
/// let driver = Ws2812PioRp::<_, 0, 29>::new(/* ... */);
/// let renderer = LayerPaletteRenderer::<8>::new(palette, Rgb::new(0, 0, 0));
/// let processor = LightingProcessor::new(driver, renderer);
/// // Run alongside the rest of the firmware via run_all! / join.
/// ```
#[processor(subscribe = [LayerChangeEvent, ModifierEvent, LedIndicatorEvent, CapsWordEvent])]
#[::rmk::macros::runnable_generated]
pub struct LightingProcessor<D, R>
where
    D: LightingDriver,
    R: LightingRenderer,
{
    driver: D,
    renderer: R,
    ctx: LightingContext,
}

impl<D, R> LightingProcessor<D, R>
where
    D: LightingDriver,
    R: LightingRenderer,
{
    pub fn new(driver: D, renderer: R) -> Self {
        Self {
            driver,
            renderer,
            ctx: LightingContext::default(),
        }
    }

    pub fn renderer_mut(&mut self) -> &mut R {
        &mut self.renderer
    }

    /// Render into the driver's frame buffer and push it to the
    /// hardware. Called once on startup and whenever a subscribed
    /// event arrives.
    async fn render_and_write(&mut self) {
        // Split-borrow the driver's frame buffer through a method call,
        // pass it to the renderer along with the immutable ctx, then
        // release the borrow before calling write().
        {
            let frame = self.driver.frame_buffer();
            self.renderer.render(&self.ctx, frame);
        }
        self.driver.write().await;
    }

    async fn on_layer_change_event(&mut self, event: LayerChangeEvent) {
        self.ctx.layer = event.0;
        self.render_and_write().await;
    }

    async fn on_modifier_event(&mut self, event: ModifierEvent) {
        self.ctx.modifiers = event.modifier;
        self.render_and_write().await;
    }

    async fn on_led_indicator_event(&mut self, event: LedIndicatorEvent) {
        self.ctx.caps_lock = event.0.caps_lock();
        self.ctx.num_lock = event.0.num_lock();
        self.ctx.scroll_lock = event.0.scroll_lock();
        self.render_and_write().await;
    }

    async fn on_caps_word_event(&mut self, event: CapsWordEvent) {
        self.ctx.caps_word = event.0;
        self.render_and_write().await;
    }
}

impl<D, R> Runnable for LightingProcessor<D, R>
where
    D: LightingDriver,
    R: LightingRenderer,
{
    async fn run(&mut self) -> ! {
        let mut sub = <Self as Processor>::subscriber();

        // Initial render: WS2812 chips power on in undefined state, so
        // write the default-context frame once before listening for
        // events. Otherwise the strip might display garbage until the
        // first event arrives (which on a static layer 0 with no host
        // indicators may be much later, e.g. on first key press).
        self.render_and_write().await;

        loop {
            let event = sub.next_event().await;
            self.process(event).await;
        }
    }
}
