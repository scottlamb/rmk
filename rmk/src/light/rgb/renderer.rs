//! Lighting renderer trait, default renderers, and the keyboard-state
//! context they consume.

use rmk_types::modifier::ModifierCombination;

use super::driver::Rgb;
use super::keymap_source::KeymapSource;

/// Snapshot of keyboard state that renderers consume on each render.
///
/// Mirrors the relevant subset of `display::RenderContext`. Per-key
/// renderers will additionally need a key→LED-index map, but that's
/// the renderer's concern (passed in at construction); the context
/// itself is just keyboard state.
#[derive(Clone, Copy, Default)]
pub struct LightingContext {
    /// Currently-active layer index.
    pub layer: u8,
    /// Modifiers currently in effect for the next keypress: held
    /// modifier keys plus armed one-shots. Mirrors what `ModifierEvent`
    /// publishes.
    pub modifiers: ModifierCombination,
    /// Host's caps-lock indicator state.
    pub caps_lock: bool,
    /// Host's num-lock indicator state.
    pub num_lock: bool,
    /// Host's scroll-lock indicator state.
    pub scroll_lock: bool,
    /// Firmware-side caps-word state. Independent of `caps_lock`:
    /// caps-word shifts subsequent letters by injecting Shift into
    /// resolved modifiers, without ever toggling host caps-lock.
    pub caps_word: bool,
}

/// Computes a frame from keyboard state.
///
/// Stateless or near-stateless — called on every render, not every
/// frame. State changes drive renders via the processor's event
/// subscription, so a renderer that produces the same output for the
/// same context inputs gets skip-on-unchanged for free (the processor
/// only renders when an event has updated the context).
///
/// `frame.len()` is the LED count for this driver — the renderer should
/// fill all entries it cares about and not assume a fixed length.
pub trait LightingRenderer {
    fn render(&mut self, ctx: &LightingContext, frame: &mut [Rgb]);
}

/// Paints every LED a single color. Useful as a smoke test for the
/// hardware path before wiring up event-driven renderers.
pub struct SolidRenderer {
    pub color: Rgb,
}

impl SolidRenderer {
    pub const fn new(color: Rgb) -> Self {
        Self { color }
    }
}

impl LightingRenderer for SolidRenderer {
    fn render(&mut self, _ctx: &LightingContext, frame: &mut [Rgb]) {
        frame.fill(self.color);
    }
}

/// Paints the whole strip with a per-layer color. The renderer holds
/// a palette indexed by layer number; layers beyond the palette length
/// fall back to the `default_color`.
///
/// This is the simplest event-driven renderer — useful for "show me
/// what layer I'm on" without the keymap-walking machinery a per-key
/// `LayerKeyRenderer` would need.
pub struct LayerPaletteRenderer<const MAX_LAYERS: usize> {
    pub palette: [Rgb; MAX_LAYERS],
    pub default_color: Rgb,
}

impl<const MAX_LAYERS: usize> LayerPaletteRenderer<MAX_LAYERS> {
    pub const fn new(palette: [Rgb; MAX_LAYERS], default_color: Rgb) -> Self {
        Self { palette, default_color }
    }
}

impl<const MAX_LAYERS: usize> LightingRenderer for LayerPaletteRenderer<MAX_LAYERS> {
    fn render(&mut self, ctx: &LightingContext, frame: &mut [Rgb]) {
        let color = self
            .palette
            .get(ctx.layer as usize)
            .copied()
            .unwrap_or(self.default_color);
        frame.fill(color);
    }
}

/// Per-key colors driven by which layer currently defines each key's
/// action.
///
/// For each (row, col) that has an LED in `led_map`, the renderer asks
/// a [`KeymapSource`] for the layer currently in effect at that
/// position (respecting active layers + `Transparent` fall-through)
/// and paints the LED with `palette[that_layer]`. Keys with no LED
/// are skipped.
///
/// Const generics: `ROWS` and `COLS` are this half's *local* matrix
/// shape; `MAX_LAYERS` sizes the palette. `row_offset` / `col_offset`
/// translate local matrix coordinates to global keymap coordinates
/// (zero on a unibody or central half; non-zero on a peripheral half
/// whose rows live higher in the global keymap).
///
/// `K` selects how layer ownership is resolved:
/// * On a unibody or split-central, pass `&KeyMap` — accurate
///   resolution including correct fall-through across multiple
///   simultaneously-active layers.
/// * On a split-peripheral, pass `&StaticKeymapView` — uses the
///   topmost-active-layer index from the `LightingContext` (synced
///   from the central via `LayerChangeEvent`) plus a default layer.
pub struct LayerKeyRenderer<'a, K: KeymapSource, const ROWS: usize, const COLS: usize, const MAX_LAYERS: usize> {
    keymap: &'a K,
    led_map: [[Option<u16>; COLS]; ROWS],
    palette: [Rgb; MAX_LAYERS],
    /// Color for keys whose action is `Transparent` on every active
    /// layer — typically only happens for keys that were never defined.
    /// A pure-transparent key fires nothing when pressed, so coloring
    /// it differently from the lit set helps spot configuration gaps.
    pub fallthrough_color: Rgb,
    pub row_offset: u8,
    pub col_offset: u8,
}

impl<'a, K: KeymapSource, const ROWS: usize, const COLS: usize, const MAX_LAYERS: usize>
    LayerKeyRenderer<'a, K, ROWS, COLS, MAX_LAYERS>
{
    pub const fn new(
        keymap: &'a K,
        led_map: [[Option<u16>; COLS]; ROWS],
        palette: [Rgb; MAX_LAYERS],
        fallthrough_color: Rgb,
        row_offset: u8,
        col_offset: u8,
    ) -> Self {
        Self {
            keymap,
            led_map,
            palette,
            fallthrough_color,
            row_offset,
            col_offset,
        }
    }
}

impl<K: KeymapSource, const ROWS: usize, const COLS: usize, const MAX_LAYERS: usize> LightingRenderer
    for LayerKeyRenderer<'_, K, ROWS, COLS, MAX_LAYERS>
{
    fn render(&mut self, ctx: &LightingContext, frame: &mut [Rgb]) {
        for row in 0..ROWS {
            for col in 0..COLS {
                let Some(led_idx) = self.led_map[row][col] else {
                    continue;
                };
                let led_idx = led_idx as usize;
                if led_idx >= frame.len() {
                    continue;
                }
                let global_row = row as u8 + self.row_offset;
                let global_col = col as u8 + self.col_offset;
                let color = match self.keymap.effective_layer_at_pos(ctx, global_row, global_col) {
                    Some(layer) => self
                        .palette
                        .get(layer as usize)
                        .copied()
                        .unwrap_or(self.fallthrough_color),
                    None => self.fallthrough_color,
                };
                frame[led_idx] = color;
            }
        }
    }
}

/// Decorator that brightens specified anchor LEDs on top of any inner
/// renderer's output. Useful for highlighting home-row and thumb keys
/// so the user can find them by feel-then-look without changing the
/// underlying per-layer rendering.
///
/// Each anchor LED's channels are multiplied by `gain_num / gain_den`
/// with saturation. Multiplicative scaling preserves channel ratios,
/// which means hue stays the same — `(16, 0, 0)` with gain 8 becomes
/// pure brighter red `(128, 0, 0)`, not a pinkish red as additive
/// boosting would produce. As long as the inner renderer's base values
/// leave headroom (channels << 255), arbitrarily high gain works
/// without saturation surprises.
///
/// We work in plain sRGB rather than a perceptual space (Oklab, HSL):
/// for the dim brightness ranges this firmware runs at, multiplicative
/// sRGB visually matches what users expect from "make it N× brighter,"
/// and it avoids float math on chips without an FPU.
pub struct AnchorOverlay<'a, R: LightingRenderer> {
    pub inner: R,
    /// LED indices to brighten.
    pub anchors: &'a [u16],
    /// Numerator of the per-channel brightness multiplier.
    pub gain_num: u16,
    /// Denominator of the per-channel brightness multiplier.
    /// `(gain_num, gain_den) = (8, 1)` gives 8×; `(3, 2)` gives 1.5×.
    pub gain_den: u16,
}

impl<'a, R: LightingRenderer> AnchorOverlay<'a, R> {
    pub const fn new(inner: R, anchors: &'a [u16], gain_num: u16, gain_den: u16) -> Self {
        Self {
            inner,
            anchors,
            gain_num,
            gain_den,
        }
    }
}

impl<R: LightingRenderer> LightingRenderer for AnchorOverlay<'_, R> {
    fn render(&mut self, ctx: &LightingContext, frame: &mut [Rgb]) {
        self.inner.render(ctx, frame);
        let num = self.gain_num as u32;
        let den = self.gain_den.max(1) as u32;
        for &idx in self.anchors {
            if let Some(led) = frame.get_mut(idx as usize) {
                led.r = ((led.r as u32 * num / den).min(255)) as u8;
                led.g = ((led.g as u32 * num / den).min(255)) as u8;
                led.b = ((led.b as u32 * num / den).min(255)) as u8;
            }
        }
    }
}

/// Decorator that *sets* a fixed list of LEDs to a specific absolute
/// color, regardless of what the inner renderer painted there. Use
/// for always-on indicators that should look the same no matter what
/// the rest of the strip is doing — e.g. dim-white anchor LEDs on a
/// black base. Unlike [`AnchorOverlay`], which scales whatever the
/// inner renderer painted, this overrides — so it works on a black
/// base where multiplicative scaling would still give black.
pub struct KeyWashOverlay<'a, R: LightingRenderer> {
    pub inner: R,
    pub leds: &'a [u16],
    pub color: Rgb,
}

impl<'a, R: LightingRenderer> KeyWashOverlay<'a, R> {
    pub const fn new(inner: R, leds: &'a [u16], color: Rgb) -> Self {
        Self { inner, leds, color }
    }
}

impl<R: LightingRenderer> LightingRenderer for KeyWashOverlay<'_, R> {
    fn render(&mut self, ctx: &LightingContext, frame: &mut [Rgb]) {
        self.inner.render(ctx, frame);
        for &idx in self.leds {
            if let Some(led) = frame.get_mut(idx as usize) {
                *led = self.color;
            }
        }
    }
}

/// Decorator that lights LEDs whose corresponding modifier bit is set
/// in `ctx.modifiers`. Each entry pairs a single-bit
/// `ModifierCombination` with one or more LED indices — when any of
/// the bits in `mask` is set, all listed LEDs are painted `color`.
/// Reflects the unified modifier view (held + armed one-shot) that
/// `ModifierEvent` publishes — `OSM(LCtrl)` armed lights its key the
/// same way physically holding `LCtrl` would.
pub struct ModifierKeyOverlay<'a, R: LightingRenderer> {
    pub inner: R,
    pub entries: &'a [(ModifierCombination, &'a [u16])],
    pub color: Rgb,
}

impl<'a, R: LightingRenderer> ModifierKeyOverlay<'a, R> {
    pub const fn new(inner: R, entries: &'a [(ModifierCombination, &'a [u16])], color: Rgb) -> Self {
        Self { inner, entries, color }
    }
}

impl<R: LightingRenderer> LightingRenderer for ModifierKeyOverlay<'_, R> {
    fn render(&mut self, ctx: &LightingContext, frame: &mut [Rgb]) {
        self.inner.render(ctx, frame);
        let active_bits = ctx.modifiers.into_bits();
        for (mask, leds) in self.entries {
            if active_bits & mask.into_bits() != 0 {
                for &idx in *leds {
                    if let Some(led) = frame.get_mut(idx as usize) {
                        *led = self.color;
                    }
                }
            }
        }
    }
}

/// Decorator that lights a fixed list of LEDs when `ctx.caps_word` is
/// true. Typical use: highlight Shift keys while caps-word is active,
/// since caps-word's effect (auto-shifted letters) follows the same
/// mental model as holding Shift.
pub struct CapsWordOverlay<'a, R: LightingRenderer> {
    pub inner: R,
    pub leds: &'a [u16],
    pub color: Rgb,
}

impl<'a, R: LightingRenderer> CapsWordOverlay<'a, R> {
    pub const fn new(inner: R, leds: &'a [u16], color: Rgb) -> Self {
        Self { inner, leds, color }
    }
}

impl<R: LightingRenderer> LightingRenderer for CapsWordOverlay<'_, R> {
    fn render(&mut self, ctx: &LightingContext, frame: &mut [Rgb]) {
        self.inner.render(ctx, frame);
        if ctx.caps_word {
            for &idx in self.leds {
                if let Some(led) = frame.get_mut(idx as usize) {
                    *led = self.color;
                }
            }
        }
    }
}

/// Decorator that lights the LED of a layer-trigger key (MO / LT /
/// TG) while its target layer is `ctx.layer`. Mirrors the modifier
/// pattern: only the trigger key glows, not every key on the layer.
/// Each entry: `(layer_idx, led_idx, color)`.
pub struct LayerTriggerOverlay<'a, R: LightingRenderer> {
    pub inner: R,
    pub entries: &'a [(u8, u16, Rgb)],
}

impl<'a, R: LightingRenderer> LayerTriggerOverlay<'a, R> {
    pub const fn new(inner: R, entries: &'a [(u8, u16, Rgb)]) -> Self {
        Self { inner, entries }
    }
}

impl<R: LightingRenderer> LightingRenderer for LayerTriggerOverlay<'_, R> {
    fn render(&mut self, ctx: &LightingContext, frame: &mut [Rgb]) {
        self.inner.render(ctx, frame);
        for &(layer, idx, color) in self.entries {
            if ctx.layer == layer {
                if let Some(led) = frame.get_mut(idx as usize) {
                    *led = color;
                }
            }
        }
    }
}
