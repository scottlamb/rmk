//! Read-only keymap views used by `LayerKeyRenderer`.
//!
//! On a unibody or split-central, the renderer queries the live
//! [`KeyMap`](crate::keymap::KeyMap) directly — that's the source of
//! truth and gives accurate "which layer owns this key right now"
//! results, including correct fall-through across multiple
//! simultaneously-active layers.
//!
//! On a split peripheral, no `KeyMap` runtime exists. The peripheral
//! receives a [`LayerChangeEvent`](crate::event::LayerChangeEvent) with
//! the central's topmost-active-layer index, but doesn't track
//! individual layer activation flags. [`StaticKeymapView`] uses the
//! topmost active layer (from the `LightingContext`) plus a
//! configured default layer to approximate the engine's resolution
//! — accurate for the common case of one momentary/toggle layer held
//! at a time, and one tier off when multiple are stacked.

use rmk_types::action::KeyAction;

use super::renderer::LightingContext;

/// Ask "which layer currently defines the action at (row, col)?".
/// Renderers that paint per-key based on layer ownership go through
/// this trait.
pub trait KeymapSource {
    /// Returns the layer index that's effectively in charge of this
    /// position right now, considering active layer state and
    /// `Transparent` fall-through. Returns `None` if every active
    /// layer is `Transparent` here (i.e. no defined action).
    ///
    /// Implementations that have their own state (e.g. `KeyMap`) can
    /// ignore the `ctx` argument; impls that only have static keymap
    /// data (e.g. `StaticKeymapView`) read `ctx.layer` to know which
    /// layer is currently active.
    fn effective_layer_at_pos(&self, ctx: &LightingContext, row: u8, col: u8) -> Option<u8>;
}

impl KeymapSource for crate::keymap::KeyMap<'_> {
    fn effective_layer_at_pos(&self, _ctx: &LightingContext, row: u8, col: u8) -> Option<u8> {
        crate::keymap::KeyMap::effective_layer_at_pos(self, row, col)
    }
}

/// Read-only view over a static `[[[KeyAction; COL]; ROW]; NUM_LAYER]`
/// array — the shape produced by the codegen-emitted
/// `get_default_keymap()`.
///
/// For peripherals that don't have a `KeyMap` runtime instance.
/// Resolution walks just `ctx.layer` (the topmost active, synced from
/// the central) and `default_layer`, which is correct as long as the
/// user only holds one momentary/toggle layer at a time. Multiple
/// simultaneously-held layers produce slightly off rendering on the
/// peripheral until one is released; this is a visual glitch, not a
/// functional issue.
///
/// `ROW` / `COL` are the *full* matrix dimensions (split: total of
/// both halves), not a single half's local matrix. The renderer's
/// `row_offset` / `col_offset` translate its local iteration to
/// global coordinates this view indexes.
pub struct StaticKeymapView<'a, const ROW: usize, const COL: usize, const NUM_LAYER: usize> {
    pub layers: &'a [[[KeyAction; COL]; ROW]; NUM_LAYER],
    pub default_layer: u8,
}

impl<'a, const ROW: usize, const COL: usize, const NUM_LAYER: usize> StaticKeymapView<'a, ROW, COL, NUM_LAYER> {
    pub const fn new(layers: &'a [[[KeyAction; COL]; ROW]; NUM_LAYER], default_layer: u8) -> Self {
        Self { layers, default_layer }
    }
}

impl<const ROW: usize, const COL: usize, const NUM_LAYER: usize> KeymapSource
    for StaticKeymapView<'_, ROW, COL, NUM_LAYER>
{
    fn effective_layer_at_pos(&self, ctx: &LightingContext, row: u8, col: u8) -> Option<u8> {
        if row as usize >= ROW || col as usize >= COL {
            return None;
        }
        // Try the topmost active layer first; if it's transparent
        // there, fall through to the default layer.
        let candidates: [u8; 2] = [ctx.layer, self.default_layer];
        let mut tried_default = false;
        for candidate in candidates {
            // Skip duplicate work when active layer == default layer.
            if candidate == self.default_layer {
                if tried_default {
                    continue;
                }
                tried_default = true;
            }
            let layer_idx = candidate as usize;
            if layer_idx >= NUM_LAYER {
                continue;
            }
            let action = self.layers[layer_idx][row as usize][col as usize];
            if action != KeyAction::Transparent {
                return Some(candidate);
            }
        }
        None
    }
}
