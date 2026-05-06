//! HID consumer for [`TrackpadEvent`].
//!
//! Subscribes to the chip-agnostic per-cycle multi-touch frame published by a
//! trackpad driver (currently [`super::iqs5xx::Iqs5xx`]) and emits HID reports
//! over a dedicated USB interface. Two reports share that interface:
//!
//! * **Legacy mouse** (Report ID `0x01`): cursor + integrated button. The
//!   default mode at boot, used by hosts that don't speak Microsoft's
//!   Precision Touchpad protocol — most importantly macOS without a
//!   userspace helper.
//! * **PTP touchpad** (Report ID `0x05`): multi-finger absolute report per
//!   the Win8 Precision Touchpad spec, plus the four mandatory Feature
//!   reports. The host opts in by writing Input Mode = 3 to Feature `0x08`.
//!
//! Microsoft's "Windows Precision Touchpad implementation guide" defines
//! the descriptor structure, per-finger record layout, the four
//! mandatory Feature reports, and the host's Input Mode handshake; the
//! wire format below follows it verbatim:
//! <https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/windows-precision-touchpad-implementation-guide>.
//!
//! The interface is sibling to the keyboard's composite HID interface, so
//! it's bound by `hid-multitouch` (Linux) / the precision-touchpad stack
//! (Windows) without affecting the composite interface — keyboard, media,
//! system, and `MouseKey`-driven cursor movement all keep working
//! independently.
//!
//! # Mode switching
//!
//! Each trackpad has its own slot in [`TRACKPAD_MODES`], written by
//! that trackpad's [`TrackpadRequestHandler`] when the host SETs
//! Feature Report `0x08` on its interface, and read by the matching
//! [`TrackpadHidProcessor`] on every event. Default = legacy. Going
//! PTP and then back to legacy resets the per-id contact tracking so
//! the first frame of a new session is clean. Two trackpads on the
//! same device can run in different modes simultaneously.
//!
//! There is no spec-Vendor heartbeat. PTPHQA is wired up (a 256-byte
//! Microsoft pre-cert blob returned for `Get_Report(Feature 0x0F)`),
//! which Linux's `hid-multitouch` needs to promote the device to
//! `HID_GROUP_MULTITOUCH_WIN_8`. Input Mode and Selective Reporting
//! live in a separate Configuration TLC (Usage `0x0E`) per Microsoft's
//! PTP Collection spec — Windows' PTP driver doesn't recognize Input
//! Mode anywhere else and otherwise leaves the device in legacy mode.
//! macOS doesn't bind PTP natively in either case.
//!
//! BLE is not wired up: this processor publishes
//! [`Report::TrackpadReport`] on the USB report channel; the BLE writer
//! drops it. A future commit can extend HOGP with a PTP report map or
//! ship the same bytes through a custom GATT characteristic — see the
//! comment by `Report::TrackpadReport` in `ble_server.rs`.
//!
//! # Tap and drag in legacy mode
//!
//! Single- and two-finger taps and 1-finger press-and-hold-then-drag are
//! detected in software, on the same finger-count-transition state machine
//! as the rmk-tree's `TrackpadProcessor`. Thresholds are configured in
//! millimetres and converted to chip units at construction so tuning is
//! independent of the panel resolution. Scrolling is intentionally not
//! supported in v1 — the chip's gesture engine isn't a portable
//! cross-controller story (no Cirque/maxtouch equivalent), and a software
//! 2-finger scroll detector lands in a follow-up.
//!
//! # Mouse-button routing
//!
//! By default the keymap's `MouseBtn{1..8}` keys feed the keyboard's
//! composite mouse report. When the keyboard config sets
//! `[input_device.mouse_button_routing].trackpad` to a trackpad's name,
//! the codegen calls [`set_mouse_button_destination`] before USB
//! enumeration; the composite path then zeros the buttons and the
//! matching trackpad's processor OR's `keymap.mouse_buttons() & 0x07`
//! into its own report's button bits — so a key bound to `MouseBtn1`
//! held while a finger moves on the surface produces a drag, since
//! both the contact and the button live on the same HID device. Bits
//! 3..=7 of `mouse_buttons()` are dropped (both touchpad TLCs declare
//! Buttons 1..=3).
//!
//! The processor also subscribes to [`MouseButtonsEvent`] so a routed
//! press / release that lands between chip cycles surfaces
//! immediately — without it, an event-mode IQS5xx with no finger on
//! the surface would never produce a chip cycle, and the click would
//! be silently dropped.

use core::cell::Cell;
use core::sync::atomic::{AtomicU8, Ordering};

use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_time::{Duration, Instant};
use embassy_usb::class::hid::{ReportId, RequestHandler};
use embassy_usb::control::OutResponse;
use rmk_macro::processor;
use static_cell::StaticCell;
use usbd_hid::descriptor::{AsInputReport, BufferOverflow, SerializedDescriptor};

use crate::RawMutex;
use crate::channel::USB_REPORT_CHANNEL;
use crate::event::{MouseButtonsEvent, TRACKPAD_MAX_FINGERS, TrackpadEvent, TrackpadFinger};
use crate::hid::Report;
use crate::keymap::KeyMap;

// ============================================================================
// Configuration
// ============================================================================

/// Maximum number of contacts the firmware reports through PTP. Mirrors
/// [`TRACKPAD_MAX_FINGERS`] and is also baked into the PTP descriptor's
/// `Contact Count Maximum` Feature value (returned in
/// [`TrackpadRequestHandler::get_report`]).
pub const TRACKPAD_MAX_CONTACTS: u8 = TRACKPAD_MAX_FINGERS as u8;

const _: () = assert!(
    TRACKPAD_MAX_CONTACTS <= 5,
    "PTP descriptor below declares exactly 5 finger collections; raising this requires extending it",
);

/// Caller-supplied panel parameters. The same values are baked into the HID
/// descriptor (so the host's logical coordinate range matches what the
/// firmware emits — no host-side rescaling needed) and used to clamp
/// emitted finger coordinates.
///
/// Logical maxes should match the chip's reported coordinate range. For
/// the IQS5xx these come from `Resolution X` / `Resolution Y` (computed by
/// the driver from channel counts at init: `(rx − 1) × 256` /
/// `(tx − 1) × 256`, e.g. 9 × 13 channels → 2048 × 3072 for a TPS65-501b).
///
/// Physical mm fields describe the panel's actual size; hosts use them
/// (with the descriptor's Unit Exponent / Unit declarations) to compute
/// density for palm-rejection and gestural-velocity heuristics.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TrackpadDimensions {
    pub logical_max_x: u16,
    pub logical_max_y: u16,
    /// Panel width in tenths of a millimetre (`mm × 10`).
    pub physical_dmm_x: u16,
    /// Panel height in tenths of a millimetre.
    pub physical_dmm_y: u16,
}

impl TrackpadDimensions {
    /// Convenience: physical extent in whole mm.
    pub const fn from_mm(logical_max_x: u16, logical_max_y: u16, mm_x: u16, mm_y: u16) -> Self {
        Self {
            logical_max_x,
            logical_max_y,
            physical_dmm_x: mm_x.saturating_mul(10),
            physical_dmm_y: mm_y.saturating_mul(10),
        }
    }

    /// Chip units per millimetre on each axis. Used to convert
    /// mm-denominated thresholds and sensitivities to chip units at
    /// construction. Per-axis because trackpad pixels aren't square in
    /// general (different channel pitch on X vs Y).
    /// Returns `0` if the panel parameters are degenerate (avoids div-by-0
    /// on misconfigured boards; thresholds clamp to 0 chip units which
    /// disables the corresponding behaviour cleanly).
    fn cu_per_mm_x(self) -> u32 {
        if self.physical_dmm_x == 0 {
            0
        } else {
            (self.logical_max_x as u32 * 10) / self.physical_dmm_x as u32
        }
    }
    fn cu_per_mm_y(self) -> u32 {
        if self.physical_dmm_y == 0 {
            0
        } else {
            (self.logical_max_y as u32 * 10) / self.physical_dmm_y as u32
        }
    }
}

/// Tunable parameters for [`TrackpadHidProcessor`]. Defaults are sane for a
/// ~60 × 90 mm panel; users can override via TOML.
///
/// Everything the report path needs is precomputed here at construction so
/// the per-frame work is just a multiply-shift-clamp and an integer
/// compare — no division, no floats, no helpers walking back to the
/// configured millimetre values.
#[derive(Clone, Copy, Debug)]
pub struct TrackpadParams {
    pub dims: TrackpadDimensions,
    /// Squared tap-deviation budget in chip-units². Compared directly
    /// against the running `max_dev_sq` of finger 0 — both the hold-latch
    /// gate and the tap-on-lift gate use this verbatim. Stored squared so
    /// the report path doesn't redo the multiply on every frame.
    pub tap_max_dev_chip_sq: u32,
    /// Maximum touch duration that still counts as a tap. 150 ms matches
    /// the IQS5xx NV default and feels right for finger-down-up.
    pub tap_time: Duration,
    /// Minimum touch duration before a stationary 1-finger touch latches
    /// button 1 for press-and-hold-then-drag. 450 ms — a tap that doesn't
    /// release becomes a drag at this point.
    pub hold_time: Duration,
    /// Cursor delta scale on each axis: emitted_units = `(d_chip × num) >> shift`.
    /// Computed from a single user-facing `sensitivity` (mouse units per
    /// mm of finger motion) divided by the panel's chip-units-per-mm —
    /// per-axis because the trackpad's pixel pitch usually differs
    /// between X and Y.
    pub scale_num_x: u16,
    pub scale_num_y: u16,
    /// Shared right-shift for both axes. Picked at construction to keep
    /// `scale_num_{x,y}` inside u16 with as much fractional precision as
    /// possible; callers don't normally read it.
    pub scale_shift: u8,
}

impl TrackpadParams {
    /// Construct from the user's "I want X mm of tap budget at Y mouse
    /// units per mm" mental model. `tap_max_dev_mm` and
    /// `sensitivity_px_per_mm` are read from `[input_device.iqs5xx]`
    /// config and converted to chip units / a fixed-point scale against
    /// the supplied panel dims.
    pub fn from_mm(
        dims: TrackpadDimensions,
        tap_max_dev_mm: u16,
        tap_time_ms: u16,
        hold_time_ms: u16,
        sensitivity_px_per_mm: f32,
    ) -> Self {
        let tap_max_dev_chip = (dims.cu_per_mm_x() * tap_max_dev_mm as u32).min(u16::MAX as u32);
        let tap_max_dev_chip_sq = tap_max_dev_chip.saturating_mul(tap_max_dev_chip);
        let (scale_num_x, scale_num_y, scale_shift) =
            compute_scale(sensitivity_px_per_mm, dims.cu_per_mm_x(), dims.cu_per_mm_y());
        Self {
            dims,
            tap_max_dev_chip_sq,
            tap_time: Duration::from_millis(tap_time_ms as u64),
            hold_time: Duration::from_millis(hold_time_ms as u64),
            scale_num_x,
            scale_num_y,
            scale_shift,
        }
    }
}

/// Pick `(num_x, num_y, shift)` so that the legacy-mouse cursor delta
/// `(d_chip * num) >> shift` approximates `d_chip * sensitivity / cu_per_mm`
/// (i.e. produces `sensitivity` mouse units per mm of finger motion on
/// each axis). Single shared shift across both axes — chosen as the
/// largest one in [0, 14] that keeps both nums inside u16 — so the report
/// path doesn't have to remember which axis it's on. Returns all-zeros
/// for degenerate inputs (zero/negative sensitivity or zero cu/mm), which
/// disables cursor motion cleanly without a panic.
fn compute_scale(sensitivity_px_per_mm: f32, cu_per_mm_x: u32, cu_per_mm_y: u32) -> (u16, u16, u8) {
    if !(sensitivity_px_per_mm > 0.0) || cu_per_mm_x == 0 || cu_per_mm_y == 0 {
        return (0, 0, 0);
    }
    let rx = sensitivity_px_per_mm / cu_per_mm_x as f32;
    let ry = sensitivity_px_per_mm / cu_per_mm_y as f32;
    // 14 keeps the multiply (d_chip up to ~3 k × num up to 65 k) inside i32
    // with comfortable headroom. Loop down so a too-large sensitivity still
    // produces *something* sane rather than overflowing num.
    let r_max = if rx > ry { rx } else { ry };
    let mut shift: u8 = 14;
    loop {
        let factor = (1u32 << shift) as f32;
        if r_max * factor <= u16::MAX as f32 {
            // `+ 0.5` rounds; both rx and ry are non-negative.
            let nx = (rx * factor + 0.5) as u16;
            let ny = (ry * factor + 0.5) as u16;
            return (nx, ny, shift);
        }
        if shift == 0 {
            return (u16::MAX, u16::MAX, 0);
        }
        shift -= 1;
    }
}

// ============================================================================
// Mode state + button-routing
// ============================================================================

const MODE_LEGACY: u8 = 0;
const MODE_PTP: u8 = 3;

/// Compile-time ceiling on the number of trackpad HID instances. Each
/// instance has its own [`TRACKPAD_MODES`] slot (the host's Input Mode
/// feature is per-interface) and its own id used for keymap-button
/// routing. The runtime in this revision wires up exactly one trackpad
/// (the USB layer holds a single `trackpad_writer` field), so values
/// above 1 here are slack for a future multi-trackpad commit. Bump and
/// the rest of the module follows; the array is small.
pub const MAX_TRACKPADS: usize = 4;

/// Per-trackpad host-selected mode. Slot `id` is written by that
/// trackpad's [`TrackpadRequestHandler`] on Set Feature `0x08`, and
/// read by the matching [`TrackpadHidProcessor`] on every event. The
/// host can run two trackpads on the same device in different modes
/// (e.g. one in PTP, one in legacy) without crosstalk.
pub static TRACKPAD_MODES: [AtomicU8; MAX_TRACKPADS] = [
    AtomicU8::new(MODE_LEGACY),
    AtomicU8::new(MODE_LEGACY),
    AtomicU8::new(MODE_LEGACY),
    AtomicU8::new(MODE_LEGACY),
];

/// Sentinel for [`MOUSE_BUTTON_DESTINATION_TRACKPAD`]: the keymap's
/// `MouseBtn1..3` keys go to the keyboard's composite mouse report,
/// not to any trackpad.
const ROUTE_TO_COMPOSITE: u8 = u8::MAX;

/// Single global "where do keymap mouse-button keys land?" knob. Holds
/// the trackpad id (0..[`MAX_TRACKPADS`]) of the trackpad whose HID
/// interface should carry keymap-pressed `MouseBtn1..3` keys, or
/// [`ROUTE_TO_COMPOSITE`] for the default (composite mouse report).
/// Single-destination by design: sending the same button to two HID
/// devices simultaneously risks click-coalescing on macOS.
static MOUSE_BUTTON_DESTINATION_TRACKPAD: AtomicU8 = AtomicU8::new(ROUTE_TO_COMPOSITE);

/// Set the global mouse-button routing destination. Pass `Some(id)` to
/// route keymap-pressed `MouseBtn1..3` to the trackpad with id `id`'s
/// HID interface; pass `None` for the default (composite mouse
/// report). Intended for the macro-generated init path; user code
/// should configure this via `[input_device.mouse_button_routing]` in
/// TOML rather than calling this directly.
pub fn set_mouse_button_destination(target: Option<u8>) {
    let v = match target {
        Some(id) if (id as usize) < MAX_TRACKPADS => id,
        _ => ROUTE_TO_COMPOSITE,
    };
    MOUSE_BUTTON_DESTINATION_TRACKPAD.store(v, Ordering::Relaxed);
}

/// True iff the composite mouse-report path should drop mouse-button
/// bits (because they go to a trackpad interface). Used by `keyboard.rs`.
pub fn composite_should_suppress_mouse_buttons() -> bool {
    MOUSE_BUTTON_DESTINATION_TRACKPAD.load(Ordering::Relaxed) != ROUTE_TO_COMPOSITE
}

/// True iff the global routing destination is the trackpad with this
/// `id`. Used by [`TrackpadHidProcessor`] to decide whether to OR
/// keymap mouse buttons into its own report's button bits.
fn destination_matches(id: u8) -> bool {
    MOUSE_BUTTON_DESTINATION_TRACKPAD.load(Ordering::Relaxed) == id
}

// ============================================================================
// HID descriptor
// ============================================================================

/// Wire-size budget for the descriptor. Current structure fits well
/// inside this; the snapshot test below pins the exact figure so any
/// edit is a deliberate change.
const TRACKPAD_DESC_BUF_SIZE: usize = 512;

/// Append-only descriptor writer with helpers for the LE-value HID items
/// we parameterise (Logical Maximum 0x26, Physical Maximum 0x46).
struct DescWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl DescWriter<'_> {
    fn raw(&mut self, bytes: &[u8]) {
        self.buf[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
        self.pos += bytes.len();
    }
    fn item_le16(&mut self, tag: u8, value: u16) {
        let v = value.to_le_bytes();
        self.raw(&[tag, v[0], v[1]]);
    }
}

/// Build the trackpad HID descriptor into `buf` and return the number of
/// bytes written. Two top-level Application Collections:
///
/// * **Mouse** (Report ID `0x01`): buttons(3) + x(i8) + y(i8). 3 buttons
///   matches what we plumb through from the keymap (`MouseBtn1..3`); btn4..8
///   stay on the composite interface regardless of routing.
/// * **Touchpad** (Report ID `0x05`) + Feature reports `0x06`/`0x07`/`0x0F`.
/// * **Configuration** (Usage Page Digitizer / Usage `0x0E`) + Feature
///   reports `0x08` (Input Mode) and `0x09` (Selective Reporting / Function
///   Switch). Microsoft's PTP collection spec puts Input Mode here, not
///   inside the Touchpad TLC; Windows' PTP driver doesn't look for Input
///   Mode anywhere else.
fn write_trackpad_descriptor(buf: &mut [u8], dims: TrackpadDimensions) -> usize {
    let mut w = DescWriter { buf, pos: 0 };

    // ===== Mouse TLC (Report ID 0x01) — boot-default fallback =====
    w.raw(&[
        0x05, 0x01, // Usage Page (Generic Desktop)
        0x09, 0x02, // Usage (Mouse)
        0xA1, 0x01, // Collection (Application)
        0x85, 0x01, //   Report ID (1)
        0x09, 0x01, //   Usage (Pointer)
        0xA1, 0x00, //   Collection (Physical)
        0x05, 0x09, //     Usage Page (Button)
        0x19, 0x01, 0x29, 0x03, //     Usage Min/Max (Button 1..3)
        0x15, 0x00, 0x25, 0x01, //     Logical Min/Max (0..1)
        0x75, 0x01, 0x95, 0x03, //     Report Size 1, Count 3
        0x81, 0x02, //     Input (Data,Var,Abs)
        0x95, 0x05, 0x81, 0x03, //     5-bit padding (Cnst)
        0x05, 0x01, //     Usage Page (Generic Desktop)
        0x09, 0x30, 0x09, 0x31, //     Usage X, Y
        0x15, 0x81, 0x25, 0x7F, //     Logical Min/Max (-127..127)
        0x75, 0x08, 0x95, 0x02, //     Report Size 8, Count 2
        0x81, 0x06, //     Input (Data,Var,Rel)
        0xC0, //   End Collection (Physical)
        0xC0, // End Collection (Application)
    ]);

    // ===== Touchpad TLC (Report ID 0x05) =====
    w.raw(&[0x05, 0x0D, 0x09, 0x05, 0xA1, 0x01, 0x85, 0x05]);

    // ----- Finger 1 (declares Physical Maximum X/Y; fingers 2..5 inherit) -----
    w.raw(&[
        0x09, 0x22, 0xA1, 0x02, // Usage (Finger), Collection (Logical)
        0x09, 0x47, 0x09, 0x42, // Usage (Confidence, Tip Switch)
        0x15, 0x00, 0x25, 0x01, // Logical Min/Max (0..1)
        0x75, 0x01, 0x95, 0x02, // Report Size 1, Count 2
        0x81, 0x02, // Input (Data,Var,Abs)
        0x95, 0x06, 0x81, 0x03, // 6-bit padding (Cnst)
        0x75, 0x08, 0x09, 0x51, // Report Size 8, Usage (Contact ID)
        0x95, 0x01, 0x25, 0x7F, // Count 1, Logical Max 127
        0x81, 0x02, // Input (Data,Var,Abs)
        0x05, 0x01, // Usage Page (Generic Desktop)
    ]);
    w.item_le16(0x26, dims.logical_max_x); // Logical Maximum (X)
    w.raw(&[
        0x75, 0x10, 0x55, 0x0E, 0x65, 0x11, // Report Size 16, Unit Exp -2, Unit cm
        0x09, 0x30, 0x35, 0x00, // Usage X, Physical Min 0
    ]);
    w.item_le16(0x46, dims.physical_dmm_x); // Physical Maximum (X)
    w.raw(&[0x95, 0x01, 0x81, 0x02]); // Count 1, Input (Data,Var,Abs)
    w.item_le16(0x46, dims.physical_dmm_y); // Physical Maximum (Y)
    w.item_le16(0x26, dims.logical_max_y); // Logical Maximum (Y)
    w.raw(&[0x09, 0x31, 0x81, 0x02, 0xC0]); // Usage Y, Input, End Collection (Logical)

    // ----- Fingers 2..=5 (Physical Maximum inherits from finger 1) -----
    for _ in 0..(TRACKPAD_MAX_CONTACTS - 1) {
        w.raw(&[
            0x05, 0x0D, 0x09, 0x22, 0xA1, 0x02, 0x05, 0x0D, 0x09, 0x47, 0x09, 0x42, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,
            0x95, 0x02, 0x81, 0x02, 0x95, 0x06, 0x81, 0x03, 0x75, 0x08, 0x09, 0x51, 0x95, 0x01, 0x25, 0x7F, 0x81, 0x02,
            0x05, 0x01,
        ]);
        w.item_le16(0x26, dims.logical_max_x);
        w.raw(&[0x75, 0x10, 0x09, 0x30, 0x81, 0x02]);
        w.item_le16(0x26, dims.logical_max_y);
        w.raw(&[0x09, 0x31, 0x81, 0x02, 0xC0]);
    }

    // ----- Scan Time + Contact Count + Button + Features -----
    w.raw(&[
        // Scan Time (16 bits, 100 µs units)
        0x05, 0x0D, 0x55, 0x0C, 0x66, 0x01, 0x10, 0x47, 0xFF, 0xFF, 0x00, 0x00, // Physical Maximum (65535)
        0x27, 0xFF, 0xFF, 0x00, 0x00, // Logical Maximum (65535)
        0x75, 0x10, 0x95, 0x01, 0x09, 0x56, 0x81, 0x02, // Contact Count (8 bits)
        0x09, 0x54, 0x25, 0x05, 0x95, 0x01, 0x75, 0x08, 0x81, 0x02, 0x55, 0x00, 0x65, 0x00, // reset units
        // Buttons 1..=3 (3 bits + 5 pad). Microsoft's PTP spec defines
        // Button 1 as the integrated touchpad button, Buttons 2 and 3 as
        // external primary/secondary clickers; we expose all three so
        // keymap-routed `MouseBtn{1,2,3}` keys land on the same HID
        // device as touch contacts. Declaring three Button-page usages
        // also breaks Linux's `hid-multitouch` auto-buttonpad heuristic
        // (which fires when a multitouch-pointer TLC has exactly one
        // button) — see the Pad Type override below.
        0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x25, 0x01, 0x75, 0x01, 0x95, 0x03, 0x81, 0x02, 0x95, 0x05, 0x81, 0x03,
        // Feature: Device Capabilities (Report ID 0x07) — max_contacts low nibble, pad_type high nibble
        0x05, 0x0D, 0x85, 0x07, 0x09, 0x55, 0x09, 0x59, 0x75, 0x04, 0x95, 0x02, 0x25, 0x0F, 0xB1, 0x02,
        // Feature: Latency Mode (Report ID 0x06)
        0x85, 0x06, 0x09, 0x60, 0x75, 0x08, 0x95, 0x01, 0x25, 0x01, 0xB1, 0x02,
        // Feature: Device Certification Status / PTPHQA (Report ID 0x0F) —
        // 256 bytes at vendor usage 0xff00:0xc5. Linux's hid-multitouch
        // requires this exact shape (count 256, size 8, that usage) to
        // promote the device to HID_GROUP_MULTITOUCH_WIN_8 and apply the
        // Win8 PTP quirks (notably WIN8_PTP_BUTTONS for click-while-no-
        // finger, plus STICKY_FINGERS / IGNORE_DUPLICATES / HOVERING).
        // The blob bytes themselves come from Microsoft's WPT
        // implementation guide and are returned by `get_report`.
        0x06, 0x00, 0xFF, // Usage Page (Vendor 0xFF00)
        0x85, 0x0F, // Report ID (15)
        0x09, 0xC5, // Usage (0xC5)
        0x15, 0x00, // Logical Minimum (0)
        0x26, 0xFF, 0x00, // Logical Maximum (255)
        0x75, 0x08, // Report Size (8)
        0x96, 0x00, 0x01, // Report Count (256)
        0xB1, 0x02, // Feature (Data, Var, Abs)
        0xC0, // End Collection (Touchpad TLC)
    ]);

    // ===== Configuration TLC (Usage Page Digitizer, Usage 0x0E) =====
    // Microsoft's PTP Collection spec requires Input Mode to live in
    // its own Configuration top-level collection, wrapped in a
    // Logical-scoped Finger collection. Windows' PTP driver doesn't
    // look for Input Mode inside the Touchpad TLC, so without this
    // separate TLC the driver never sends `SET_FEATURE(Input Mode=3)`
    // and the device stays in legacy mode (Service=Empty on the
    // touch-pad PDO, no Settings → Touchpad page). QMK's
    // known-working PTP descriptor uses this same structure (see
    // `tmk_core/protocol/usb_descriptor.c`). Selective Reporting goes
    // in a Physical-scoped Finger collection alongside it.
    w.raw(&[
        0x05, 0x0D, // Usage Page (Digitizer)
        0x09, 0x0E, // Usage (Configuration)
        0xA1, 0x01, // Collection (Application)
        // ----- Feature: Input Mode (Report ID 0x08) — host writes 3 to enter PTP -----
        0x09, 0x22, // Usage (Finger)
        0xA1, 0x02, // Collection (Logical)
        0x85, 0x08, // Report ID (8)
        0x09, 0x52, // Usage (Input Mode)
        0x15, 0x00, 0x25, 0x0A, // Logical Min 0, Max 10
        0x95, 0x01, 0x75, 0x08, // Report Count 1, Size 8
        0xB1, 0x02, // Feature (Data, Var, Abs)
        0xC0, // End Collection (Logical Finger)
        // ----- Feature: Selective Reporting / Function Switch (Report ID 0x09) -----
        0x09, 0x22, // Usage (Finger)
        0xA1, 0x00, // Collection (Physical)
        0x85, 0x09, // Report ID (9)
        0x09, 0x57, // Usage (Surface Switch)
        0x09, 0x58, // Usage (Button Switch)
        0x15, 0x00, 0x25, 0x01, // Logical Min 0, Max 1
        0x95, 0x02, 0x75, 0x01, // Report Count 2, Size 1
        0xB1, 0x02, // Feature (Data, Var, Abs)
        0x95, 0x06, 0xB1, 0x03, // 6-bit padding (Const)
        0xC0, // End Collection (Physical Finger)
        0xC0, // End Collection (Configuration TLC)
    ]);

    w.pos
}

/// Microsoft's pre-certification PTPHQA blob, copied verbatim from
/// "Windows Precision Touchpad Collection" → "Device Certification
/// Status Feature Report":
/// <https://learn.microsoft.com/en-us/windows-hardware/design/component-guidelines/touchpad-windows-precision-touchpad-collection>.
/// Required for Windows 8.1 backward compatibility; on Linux only the
/// structural presence (256 bytes at usage 0xff00:0xc5) gates the Win8
/// PTP quirks. We return this for `Get_Report(Feature 0x0F)`.
const PTPHQA_BLOB: [u8; 256] = [
    0xfc, 0x28, 0xfe, 0x84, 0x40, 0xcb, 0x9a, 0x87, 0x0d, 0xbe, 0x57, 0x3c, 0xb6, 0x70, 0x09, 0x88, 0x07,
    0x97, 0x2d, 0x2b, 0xe3, 0x38, 0x34, 0xb6, 0x6c, 0xed, 0xb0, 0xf7, 0xe5, 0x9c, 0xf6, 0xc2, 0x2e, 0x84,
    0x1b, 0xe8, 0xb4, 0x51, 0x78, 0x43, 0x1f, 0x28, 0x4b, 0x7c, 0x2d, 0x53, 0xaf, 0xfc, 0x47, 0x70, 0x1b,
    0x59, 0x6f, 0x74, 0x43, 0xc4, 0xf3, 0x47, 0x18, 0x53, 0x1a, 0xa2, 0xa1, 0x71, 0xc7, 0x95, 0x0e, 0x31,
    0x55, 0x21, 0xd3, 0xb5, 0x1e, 0xe9, 0x0c, 0xba, 0xec, 0xb8, 0x89, 0x19, 0x3e, 0xb3, 0xaf, 0x75, 0x81,
    0x9d, 0x53, 0xb9, 0x41, 0x57, 0xf4, 0x6d, 0x39, 0x25, 0x29, 0x7c, 0x87, 0xd9, 0xb4, 0x98, 0x45, 0x7d,
    0xa7, 0x26, 0x9c, 0x65, 0x3b, 0x85, 0x68, 0x89, 0xd7, 0x3b, 0xbd, 0xff, 0x14, 0x67, 0xf2, 0x2b, 0xf0,
    0x2a, 0x41, 0x54, 0xf0, 0xfd, 0x2c, 0x66, 0x7c, 0xf8, 0xc0, 0x8f, 0x33, 0x13, 0x03, 0xf1, 0xd3, 0xc1, 0x0b,
    0x89, 0xd9, 0x1b, 0x62, 0xcd, 0x51, 0xb7, 0x80, 0xb8, 0xaf, 0x3a, 0x10, 0xc1, 0x8a, 0x5b, 0xe8, 0x8a,
    0x56, 0xf0, 0x8c, 0xaa, 0xfa, 0x35, 0xe9, 0x42, 0xc4, 0xd8, 0x55, 0xc3, 0x38, 0xcc, 0x2b, 0x53, 0x5c,
    0x69, 0x52, 0xd5, 0xc8, 0x73, 0x02, 0x38, 0x7c, 0x73, 0xb6, 0x41, 0xe7, 0xff, 0x05, 0xd8, 0x2b, 0x79,
    0x9a, 0xe2, 0x34, 0x60, 0x8f, 0xa3, 0x32, 0x1f, 0x09, 0x78, 0x62, 0xbc, 0x80, 0xe3, 0x0f, 0xbd, 0x65,
    0x20, 0x08, 0x13, 0xc1, 0xe2, 0xee, 0x53, 0x2d, 0x86, 0x7e, 0xa7, 0x5a, 0xc5, 0xd3, 0x7d, 0x98, 0xbe,
    0x31, 0x48, 0x1f, 0xfb, 0xda, 0xaf, 0xa2, 0xa8, 0x6a, 0x89, 0xd6, 0xbf, 0xf2, 0xd3, 0x32, 0x2a, 0x9a,
    0xe4, 0xcf, 0x17, 0xb7, 0xb8, 0xf4, 0xe1, 0x33, 0x08, 0x24, 0x8b, 0xc4, 0x43, 0xa5, 0xe5, 0x24, 0xc2,
];

/// Slice into the static buffer that [`install_trackpad_descriptor`] populated.
/// Read by [`TrackpadDescriptor::desc`] when `add_usb_writer!` builds the
/// HID interface.
static TRACKPAD_DESC_SLICE: BlockingMutex<RawMutex, Cell<Option<&'static [u8]>>> = BlockingMutex::new(Cell::new(None));

/// Build the descriptor for the supplied panel and stash it where the
/// USB HID class can pick it up at enumeration. Returns the params
/// unchanged for chaining into `TrackpadHidProcessor::new`.
///
/// Must be called before USB enumeration. The user's central is
/// single-threaded at startup, so calling this from the main task before
/// `rmk.run().await` satisfies that ordering.
pub fn install_trackpad_descriptor(params: TrackpadParams) -> TrackpadParams {
    static BUF: StaticCell<[u8; TRACKPAD_DESC_BUF_SIZE]> = StaticCell::new();
    let buf = BUF.init([0u8; TRACKPAD_DESC_BUF_SIZE]);
    let len = write_trackpad_descriptor(buf, params.dims);
    let slice: &'static [u8] = &buf[..len];
    TRACKPAD_DESC_SLICE.lock(|c| c.set(Some(slice)));
    params
}

/// Marker type for `embassy-usb-hid`'s [`SerializedDescriptor`] indirection.
/// `add_usb_writer!` calls `<TrackpadDescriptor>::desc()` to fetch the byte
/// slice; we hand it the static buffer that
/// [`install_trackpad_descriptor`] populated.
pub struct TrackpadDescriptor;

impl SerializedDescriptor for TrackpadDescriptor {
    fn desc() -> &'static [u8] {
        TRACKPAD_DESC_SLICE
            .lock(|c| c.get())
            .expect("install_trackpad_descriptor must run before USB enumeration")
    }
}

/// On-wire bytes per finger record. Confidence(1b) + tip_switch(1b) + 6b
/// padding (1 byte) + contact_id(8b) (1 byte) + x(16b LE) (2 bytes) +
/// y(16b LE) (2 bytes) = 6 bytes.
const PTP_FINGER_BYTES: usize = 6;

/// PTP touchpad report payload size, excluding the 1-byte Report ID
/// prefix: 5 × finger(6) + scan_time(2) + contact_count(1) + button(1) =
/// 34 bytes.
const PTP_REPORT_PAYLOAD_BYTES: usize = (TRACKPAD_MAX_CONTACTS as usize) * PTP_FINGER_BYTES + 2 + 1 + 1;

/// IN-endpoint buffer size for the trackpad HID writer. Sized for the
/// largest report on this interface (PTP touchpad payload + Report ID).
pub const TRACKPAD_WRITER_BUF: usize = PTP_REPORT_PAYLOAD_BYTES + 1;

// ============================================================================
// Request handler — Set/Get Feature
// ============================================================================

/// Request handler for one trackpad's HID interface. Implements the
/// four PTP-spec Feature reports plus the Input Mode latch — writes the
/// mode into [`TRACKPAD_MODES`]`[id]`, so two trackpads on the same
/// device can run in different modes simultaneously.
pub struct TrackpadRequestHandler {
    id: u8,
}

impl TrackpadRequestHandler {
    pub const fn new(id: u8) -> Self {
        Self { id }
    }

    fn mode(&self) -> &'static AtomicU8 {
        &TRACKPAD_MODES[(self.id as usize).min(MAX_TRACKPADS - 1)]
    }
}

impl RequestHandler for TrackpadRequestHandler {
    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        match id {
            ReportId::Feature(0x08) => {
                // embassy-usb forwards the raw control-transfer payload.
                // For numbered reports the host prefixes it with the
                // Report ID byte (HID 1.11 §7.2.2) — data[0] is 0x08,
                // the InputMode value lives at data[1].
                if let Some(&value) = data.get(1) {
                    let new_mode = if value == MODE_PTP { MODE_PTP } else { MODE_LEGACY };
                    // load+store rather than `swap`: thumbv6m / RP2040
                    // has no atomic RMW. The cooperative embassy
                    // executor means the only writer (this request
                    // handler) and the only reader on the central
                    // (`TrackpadHidProcessor::current_mode`) never race
                    // — and a torn read here would just print the wrong
                    // "mode ->" log line, the new mode is what subsequent
                    // events see either way.
                    let mode = self.mode();
                    let old = mode.load(Ordering::Relaxed);
                    mode.store(new_mode, Ordering::Relaxed);
                    if old != new_mode {
                        info!("trackpad {} mode -> {}", self.id, new_mode);
                    }
                }
                OutResponse::Accepted
            }
            // 0x06 (Latency Mode), 0x09 (Selective Reporting): accept and
            // ignore. We don't yet act on either; the host's writes are
            // harmless and acknowledging them keeps PTP recognition happy.
            ReportId::Feature(0x06) | ReportId::Feature(0x09) => OutResponse::Accepted,
            _ => {
                // Reject (STALL) anything we don't speak. Hosts probing
                // for an optional vendor extension (e.g. the
                // macos-trackpad-companion's Feature `0x10` PTP-control
                // pulse) need to see a hard "no" so their fallback to
                // the spec Feature `0x08` actually fires — accepting
                // would silently swallow the request and leave the
                // firmware in legacy mode while the host thinks PTP is
                // active.
                debug!("trackpad {} set_report unhandled id={:?} data={:?}", self.id, id, data);
                OutResponse::Rejected
            }
        }
    }

    fn get_report(&mut self, id: ReportId, buf: &mut [u8]) -> Option<usize> {
        // For numbered descriptors the host expects the response to start
        // with the Report ID byte (HID 1.11 §7.2.1); Linux's
        // `hid_input_report` reads `data[0]` as the ID and skips it.
        // embassy-usb writes back exactly what we put in `buf`, so we
        // include the prefix ourselves.
        if let ReportId::Feature(0x0F) = id {
            const N: usize = 1 + PTPHQA_BLOB.len();
            if buf.len() < N {
                return None;
            }
            buf[0] = 0x0F;
            buf[1..N].copy_from_slice(&PTPHQA_BLOB);
            return Some(N);
        }
        let (rid, value) = match id {
            // Device Capabilities: max_contacts in low nibble, pad_type
            // in high nibble. Pad Type 2 = "Non-Clickable / Discrete-pad"
            // matches the IQS5xx surface — rigid, no integrated mechanical
            // click; clicks come from the keymap-routed Mouse buttons.
            // Together with declaring Buttons 1..=3 (instead of just
            // Button 1) in the touchpad TLC, this defeats both branches
            // of Linux hid-multitouch's auto-buttonpad detection
            // (`buttons_count == 1` and `BUTTONTYPE == CLICKPAD`), so
            // `INPUT_PROP_BUTTONPAD` is not set and libinput's
            // clickpad-only "click requires a finger on the surface"
            // filter doesn't apply.
            ReportId::Feature(0x07) => (0x07, (2 << 4) | (TRACKPAD_MAX_CONTACTS & 0x0F)),
            // Input Mode mirrors the live atomic so a paranoid host can
            // verify its Set took.
            ReportId::Feature(0x08) => (0x08, self.mode().load(Ordering::Relaxed)),
            // Selective Reporting: surface_switch=1, button_switch=1.
            // Report tracking is enabled and the integrated button is live.
            ReportId::Feature(0x09) => (0x09, 0b11),
            // Latency Mode: 0 = normal latency.
            ReportId::Feature(0x06) => (0x06, 0),
            _ => return None,
        };
        if buf.len() < 2 {
            return None;
        }
        buf[0] = rid;
        buf[1] = value;
        Some(2)
    }
}

// ============================================================================
// Wire reports
// ============================================================================

/// Legacy-mouse report. Wire layout: 3 bytes after the 1-byte Report ID
/// prefix (buttons(1) + x(i8) + y(i8)).
#[derive(Default, Clone, Copy, Debug)]
pub struct TrackpadLegacyMouseReport {
    pub buttons: u8,
    pub x: i8,
    pub y: i8,
}

impl TrackpadLegacyMouseReport {
    fn serialize(&self, buf: &mut [u8]) -> Result<usize, BufferOverflow> {
        if buf.len() < 4 {
            return Err(BufferOverflow);
        }
        buf[0] = 0x01;
        buf[1] = self.buttons & 0b0000_0111;
        buf[2] = self.x as u8;
        buf[3] = self.y as u8;
        Ok(4)
    }
}

/// One PTP finger record. 5 bytes packed; serialized big-endian within the
/// flag byte but the integers are little-endian (HID convention).
#[derive(Default, Clone, Copy, Debug)]
pub struct TrackpadPtpFinger {
    pub confidence: bool,
    pub tip_switch: bool,
    pub contact_id: u8,
    pub x: u16,
    pub y: u16,
}

/// PTP touchpad input report. Wire layout (after the 1-byte Report ID):
/// 5 × finger(5) + scan_time(2) + contact_count(1) + button(1) = 29 bytes.
#[derive(Default, Clone, Copy, Debug)]
pub struct TrackpadPtpReport {
    pub fingers: [TrackpadPtpFinger; TRACKPAD_MAX_CONTACTS as usize],
    pub scan_time: u16,
    pub contact_count: u8,
    pub button: u8,
}

impl TrackpadPtpReport {
    fn serialize(&self, buf: &mut [u8]) -> Result<usize, BufferOverflow> {
        const N: usize = 1 + PTP_REPORT_PAYLOAD_BYTES;
        if buf.len() < N {
            return Err(BufferOverflow);
        }
        buf[0] = 0x05;
        let mut p = 1;
        for f in &self.fingers {
            let mut flags = 0u8;
            if f.confidence {
                flags |= 0b01;
            }
            if f.tip_switch {
                flags |= 0b10;
            }
            buf[p] = flags;
            buf[p + 1] = f.contact_id & 0x7F;
            buf[p + 2..p + 4].copy_from_slice(&f.x.to_le_bytes());
            buf[p + 4..p + 6].copy_from_slice(&f.y.to_le_bytes());
            p += PTP_FINGER_BYTES;
        }
        buf[p..p + 2].copy_from_slice(&self.scan_time.to_le_bytes());
        buf[p + 2] = self.contact_count;
        buf[p + 3] = self.button & 0b111;
        Ok(p + 4)
    }
}

/// Either-or report carried over `KEYBOARD_REPORT_CHANNEL` for the trackpad
/// interface. The USB writer task dispatches both to the same `HidWriter`.
#[derive(Clone, Copy, Debug)]
pub enum TrackpadReport {
    LegacyMouse(TrackpadLegacyMouseReport),
    Ptp(TrackpadPtpReport),
}

impl AsInputReport for TrackpadReport {
    fn serialize(&self, buf: &mut [u8]) -> Result<usize, BufferOverflow> {
        match self {
            TrackpadReport::LegacyMouse(r) => r.serialize(buf),
            TrackpadReport::Ptp(r) => r.serialize(buf),
        }
    }
}

// ============================================================================
// Processor
// ============================================================================

/// Per-contact PTP tracking state. `active=false` slots have no live
/// finger and no pending lift report.
#[derive(Clone, Copy, Default)]
struct PtpContact {
    x: u16,
    y: u16,
    active: bool,
}

/// State accumulated while at least one finger is on the pad. Used on the
/// lift transition to decide tap / drag / nothing in legacy mode. Outside
/// a touch session this is `None`.
struct TouchSession {
    begin: Instant,
    /// Maximum start-to-now squared deviation of finger 0, in chip-units².
    max_dev_sq: u32,
    /// Largest finger count seen during the session — distinguishes a
    /// 1-finger tap (button 1) from a 2-finger tap (button 2).
    max_n: u8,
    /// Once latched, button 1 is held down for the rest of the session
    /// and the touch is excluded from tap evaluation on lift. Drag is the
    /// motion that follows.
    hold_latched: bool,
    /// Position of finger 0 at session start, anchor for the deviation
    /// running max.
    start_x: u16,
    start_y: u16,
}

/// Maximum reports a single [`TrackpadEvent`] can produce on either
/// path. Legacy-mode tap-on-lift is the worst case: a press + a synthetic
/// release. PTP frames are always exactly one report.
const MAX_OUTPUTS: usize = 2;

/// 100-µs scan-time stamp truncated to the spec's 16-bit field.
fn scan_time_at(now: Instant) -> u16 {
    ((now.as_micros() / 100) & 0xFFFF) as u16
}

/// Legacy-mouse path state. Pure decoder — no async, no channel — so the
/// behavioural tests at the bottom of this file can drive it directly.
#[derive(Default)]
struct LegacyState {
    /// In-progress touch session, if any.
    session: Option<TouchSession>,
    /// Last frame's finger-0 chip position, used to compute dx/dy.
    prev_xy: Option<(u16, u16)>,
    /// Cursor delta the last frame produced; held back one cycle so a
    /// finger-lift transition can discard it (capacitive trackpads
    /// commonly emit a centroid jump on the last frame).
    pending_motion: Option<(i16, i16)>,
    /// Last button bits emitted. Lets us skip a no-op report when the
    /// new frame's button state matches what we already sent.
    last_buttons_emitted: u8,
}

impl LegacyState {
    /// Decode one [`TrackpadEvent`] into zero or more reports, pushing
    /// each into `out`. `now` is injected so tests can drive the
    /// state machine through scripted timestamps. `routed_buttons` is
    /// the keymap-pressed `MouseBtn1..3` bitmap (masked) when this
    /// trackpad is the global routing destination, otherwise 0; it's
    /// OR'd into firmware-detected buttons so a `MouseBtn1` held while
    /// the user drags reads as the same HID device's button + contact
    /// moving together.
    fn process(
        &mut self,
        event: &TrackpadEvent,
        params: &TrackpadParams,
        now: Instant,
        routed_buttons: u8,
        out: &mut heapless::Vec<TrackpadReport, MAX_OUTPUTS>,
    ) {
        // Count *live* contacts (`tip == true`). A lifted finger leaves a
        // single `tip = false` record on the cycle it lifts (the iqs5xx
        // driver mirrors that for the spec PTP `tip_switch = 0`
        // semantics), so `event.fingers.len()` overcounts on lift and
        // would prevent the `n == 0` branch from ever firing — the tap
        // and drag-release paths below only run on the live-finger
        // transition, so getting this wrong silently kills tap detection.
        let n = event.fingers.iter().filter(|f| f.tip).count() as u8;
        let f0 = event.fingers.iter().find(|f| f.id == 0 && f.tip);

        // Update or open a touch session.
        if n == 0 {
            // Lift handled below — leave self.session in place for the
            // lift logic to consume.
        } else if let Some(s) = self.session.as_mut() {
            s.max_n = s.max_n.max(n);
            if let Some(f) = f0 {
                // Saturating arithmetic: with `f.x` a u16 (up to 65535),
                // `dx` lands in [-65535, 65535] and `dx * dx` overflows
                // i32 once |dx| > 46340. Tap thresholds are small (a
                // few mm of chip units), so saturating at i32::MAX
                // still puts us well above `tap_max_dev_chip_sq` and the
                // hold-latch comparison stays correct.
                let dx = f.x as i32 - s.start_x as i32;
                let dy = f.y as i32 - s.start_y as i32;
                let dsq = dx.saturating_mul(dx).saturating_add(dy.saturating_mul(dy)).max(0) as u32;
                s.max_dev_sq = s.max_dev_sq.max(dsq);
                if !s.hold_latched
                    && s.max_n == 1
                    && s.max_dev_sq <= params.tap_max_dev_chip_sq
                    && now.saturating_duration_since(s.begin) >= params.hold_time
                {
                    s.hold_latched = true;
                    debug!(
                        "trackpad legacy: hold latched after {} ms (dev_sq={}/{})",
                        now.saturating_duration_since(s.begin).as_millis(),
                        s.max_dev_sq,
                        params.tap_max_dev_chip_sq,
                    );
                }
            }
        } else {
            let (sx, sy) = f0.map(|f| (f.x, f.y)).unwrap_or((0, 0));
            debug!("trackpad legacy: session start n={} at ({}, {})", n, sx, sy);
            self.session = Some(TouchSession {
                begin: now,
                max_dev_sq: 0,
                max_n: n,
                hold_latched: false,
                start_x: sx,
                start_y: sy,
            });
        }

        // Cursor delta from finger 0's previous absolute position. Only
        // on frames with at least one tip=true finger; sentinel/lift
        // frames shouldn't move the cursor.
        let new_motion = match (f0, self.prev_xy) {
            (Some(f), Some((px, py))) => {
                let dx = f.x as i32 - px as i32;
                let dy = f.y as i32 - py as i32;
                let scaled = (
                    scale(dx, params.scale_num_x, params.scale_shift),
                    scale(dy, params.scale_num_y, params.scale_shift),
                );
                self.prev_xy = Some((f.x, f.y));
                Some(scaled)
            }
            (Some(f), None) => {
                self.prev_xy = Some((f.x, f.y));
                None
            }
            (None, _) => None,
        };

        // Lift: emit any pending tap and clear session. Discard the
        // buffered motion (centroid jump on release).
        if n == 0 {
            if let Some(s) = self.session.take() {
                let duration = now.saturating_duration_since(s.begin);
                let dur_ms = duration.as_millis();
                let tap_time_ms = params.tap_time.as_millis();
                let was_tap =
                    !s.hold_latched && duration <= params.tap_time && s.max_dev_sq <= params.tap_max_dev_chip_sq;
                debug!(
                    "trackpad legacy: lift dur={}/{} ms dev_sq={}/{} max_n={} hold={} -> tap={}",
                    dur_ms, tap_time_ms, s.max_dev_sq, params.tap_max_dev_chip_sq, s.max_n, s.hold_latched, was_tap,
                );
                if was_tap {
                    let btn: u8 = match s.max_n {
                        1 => 0b001,
                        _ => 0b010,
                    };
                    // Press, then synthetic release. Routed bits travel
                    // along on both edges so a "tap while holding btn1
                    // from a key" still surfaces as a press-release pair.
                    let pressed = btn | routed_buttons;
                    debug!("trackpad legacy: tap pulse btn={:#x} routed={:#x}", btn, routed_buttons);
                    self.last_buttons_emitted = pressed;
                    let _ = out.push(TrackpadReport::LegacyMouse(TrackpadLegacyMouseReport {
                        buttons: pressed,
                        x: 0,
                        y: 0,
                    }));
                    self.last_buttons_emitted = routed_buttons;
                    let _ = out.push(TrackpadReport::LegacyMouse(TrackpadLegacyMouseReport {
                        buttons: routed_buttons,
                        x: 0,
                        y: 0,
                    }));
                } else if s.hold_latched {
                    // Drag release. Any still-held routed button stays.
                    debug!("trackpad legacy: drag release routed={:#x}", routed_buttons);
                    self.last_buttons_emitted = routed_buttons;
                    let _ = out.push(TrackpadReport::LegacyMouse(TrackpadLegacyMouseReport {
                        buttons: routed_buttons,
                        x: 0,
                        y: 0,
                    }));
                }
            }
            self.prev_xy = None;
            self.pending_motion = None;
            return;
        }

        // Publish the *previous* frame's motion (one-cycle hold-back).
        // ~13 ms of cursor latency in exchange for not teleporting on
        // release.
        let firmware_buttons = match &self.session {
            Some(s) if s.hold_latched => 0b001,
            _ => 0,
        };
        let buttons = firmware_buttons | routed_buttons;
        if let Some((dx, dy)) = self.pending_motion.take() {
            self.last_buttons_emitted = buttons;
            let _ = out.push(TrackpadReport::LegacyMouse(TrackpadLegacyMouseReport {
                buttons,
                x: clamp_i8(dx),
                y: clamp_i8(dy),
            }));
        } else if buttons != self.last_buttons_emitted {
            // Button transition with no buffered motion — surface it
            // now so a hold-latch at HOLD_TIME doesn't wait for the
            // next cycle.
            self.last_buttons_emitted = buttons;
            let _ = out.push(TrackpadReport::LegacyMouse(TrackpadLegacyMouseReport {
                buttons,
                x: 0,
                y: 0,
            }));
        }
        self.pending_motion = new_motion;
    }

    /// Surface a between-chip-cycle keymap-button transition. Emits a
    /// movement-zero mouse report with the new buttons (firmware-latched
    /// hold OR'd with routed) when it differs from what we last sent.
    /// No-op when the buttons match the last emission — keeps the line
    /// quiet on no-op events.
    fn process_button_event(&mut self, routed_buttons: u8, out: &mut heapless::Vec<TrackpadReport, MAX_OUTPUTS>) {
        let firmware_buttons = match &self.session {
            Some(s) if s.hold_latched => 0b001,
            _ => 0,
        };
        let buttons = firmware_buttons | routed_buttons;
        if buttons == self.last_buttons_emitted {
            return;
        }
        self.last_buttons_emitted = buttons;
        let _ = out.push(TrackpadReport::LegacyMouse(TrackpadLegacyMouseReport {
            buttons,
            x: 0,
            y: 0,
        }));
    }
}

/// PTP touchpad-mode state. Pure decoder, same shape as
/// [`LegacyState`]: no async, no channel.
struct PtpState {
    /// Per-contact tracking, indexed by `contact_id`.
    contacts: [PtpContact; TRACKPAD_MAX_CONTACTS as usize],
    /// Last button-byte emitted. Lets [`Self::process_button_event`]
    /// dedupe — if a chip cycle already carried the new state, no
    /// separate button-only report is needed.
    last_button_emitted: u8,
}

impl Default for PtpState {
    fn default() -> Self {
        Self {
            contacts: [PtpContact::default(); TRACKPAD_MAX_CONTACTS as usize],
            last_button_emitted: 0,
        }
    }
}

impl PtpState {
    /// Decode one [`TrackpadEvent`] into one PTP report. `button` is
    /// Buttons 1..=3 packed into the low 3 bits — typically
    /// `routed_buttons & 0b111` from the keymap when this trackpad is
    /// the routing destination, otherwise 0.
    fn process(
        &mut self,
        event: &TrackpadEvent,
        params: &TrackpadParams,
        now: Instant,
        button: u8,
        out: &mut heapless::Vec<TrackpadReport, MAX_OUTPUTS>,
    ) {
        let button = button & 0b111;
        // Index live fingers by id so the per-slot walk pairs each
        // tracked id with its (possibly absent) live record in one pass.
        let mut cur: [Option<&TrackpadFinger>; TRACKPAD_MAX_CONTACTS as usize] = [None; TRACKPAD_MAX_CONTACTS as usize];
        for f in event.fingers.iter() {
            if let Some(slot) = cur.get_mut(f.id as usize) {
                *slot = Some(f);
            }
        }

        let mut report = TrackpadPtpReport {
            scan_time: scan_time_at(now),
            button,
            ..Default::default()
        };
        let mut next = 0;
        for (id, c) in self.contacts.iter_mut().enumerate() {
            let pf = match cur[id] {
                Some(f) => {
                    c.x = f.x;
                    c.y = f.y;
                    c.active = f.tip;
                    TrackpadPtpFinger {
                        confidence: f.confidence,
                        tip_switch: f.tip,
                        contact_id: id as u8,
                        x: f.x.min(params.dims.logical_max_x),
                        y: f.y.min(params.dims.logical_max_y),
                    }
                }
                // Source dropped this id without a tip=false record —
                // synthesise the lift report at its last known position.
                None if c.active => {
                    c.active = false;
                    TrackpadPtpFinger {
                        confidence: true,
                        tip_switch: false,
                        contact_id: id as u8,
                        x: c.x.min(params.dims.logical_max_x),
                        y: c.y.min(params.dims.logical_max_y),
                    }
                }
                None => continue,
            };
            if next < report.fingers.len() {
                report.fingers[next] = pf;
                next += 1;
            }
        }
        report.contact_count = next as u8;
        self.last_button_emitted = button;
        let _ = out.push(TrackpadReport::Ptp(report));
    }

    /// Surface a between-chip-cycle button transition. PTP wants the
    /// host to see the *current* contact set on every report, even
    /// when only the button changed — so this replays the live
    /// contacts at their last known positions (zeroes them out for
    /// any slot that's gone idle). Dedupes against
    /// [`Self::last_button_emitted`] so identical buttons skip the
    /// emission.
    fn process_button_event(
        &mut self,
        params: &TrackpadParams,
        now: Instant,
        button: u8,
        out: &mut heapless::Vec<TrackpadReport, MAX_OUTPUTS>,
    ) {
        let button = button & 0b111;
        if button == self.last_button_emitted {
            return;
        }
        let mut report = TrackpadPtpReport {
            scan_time: scan_time_at(now),
            button,
            ..Default::default()
        };
        let mut next = 0;
        for (id, c) in self.contacts.iter().enumerate() {
            if c.active && next < report.fingers.len() {
                report.fingers[next] = TrackpadPtpFinger {
                    confidence: true,
                    tip_switch: true,
                    contact_id: id as u8,
                    x: c.x.min(params.dims.logical_max_x),
                    y: c.y.min(params.dims.logical_max_y),
                };
                next += 1;
            }
        }
        report.contact_count = next as u8;
        self.last_button_emitted = button;
        let _ = out.push(TrackpadReport::Ptp(report));
    }
}

/// Translates [`TrackpadEvent`] into HID reports on the trackpad
/// interface. Thin async wrapper around the pure [`LegacyState`] /
/// [`PtpState`] decoders, switching between them on the host's
/// Input Mode (per-trackpad slot in [`TRACKPAD_MODES`]).
///
/// Also subscribes to [`MouseButtonsEvent`] so a routed `MouseBtn1..3`
/// press / release that lands between chip cycles surfaces
/// immediately.
#[processor(subscribe = [TrackpadEvent, MouseButtonsEvent])]
pub struct TrackpadHidProcessor<'a> {
    /// 0..[`MAX_TRACKPADS`]. Indexes [`TRACKPAD_MODES`] for this
    /// processor's mode and is matched against the global routing
    /// destination for keymap mouse-button injection. Must match the
    /// id passed to that interface's [`TrackpadRequestHandler`].
    id: u8,
    params: TrackpadParams,
    keymap: &'a KeyMap<'a>,
    legacy: LegacyState,
    ptp: PtpState,
}

impl<'a> TrackpadHidProcessor<'a> {
    /// `id` is this trackpad's slot index in [`TRACKPAD_MODES`] and
    /// must match the value passed to its [`TrackpadRequestHandler`].
    /// With `MAX_TRACKPADS = 4`, valid ids are 0..=3; out-of-range
    /// values are clamped to the last slot rather than panicking on
    /// the chip-cycle hot path.
    pub fn new(id: u8, params: TrackpadParams, keymap: &'a KeyMap<'a>) -> Self {
        Self {
            id,
            params,
            keymap,
            legacy: LegacyState::default(),
            ptp: PtpState::default(),
        }
    }

    fn current_mode(&self) -> u8 {
        TRACKPAD_MODES[(self.id as usize).min(MAX_TRACKPADS - 1)].load(Ordering::Relaxed)
    }

    /// Mouse-button bits to OR into this trackpad's report. Masked by
    /// `mask` (`0b111` for both legacy mode's 3-button mouse TLC and
    /// PTP's Buttons 1..=3). Returns 0 when the global routing
    /// destination doesn't point at this trackpad.
    fn routed_buttons(&self, mask: u8) -> u8 {
        if destination_matches(self.id) {
            self.keymap.mouse_buttons() & mask
        } else {
            0
        }
    }

    async fn on_trackpad_event(&mut self, event: TrackpadEvent) {
        let now = Instant::now();
        let mut out: heapless::Vec<TrackpadReport, MAX_OUTPUTS> = heapless::Vec::new();
        // Mode change resets the *other* mode's state so the first
        // frame after a flip is clean.
        if self.current_mode() == MODE_PTP {
            self.ptp
                .process(&event, &self.params, now, self.routed_buttons(0b111), &mut out);
            self.legacy = LegacyState::default();
        } else {
            self.legacy
                .process(&event, &self.params, now, self.routed_buttons(0b111), &mut out);
            self.ptp = PtpState::default();
        }
        for r in out {
            send(r).await;
        }
    }

    /// Surface a between-chip-cycle keymap-button transition through
    /// this trackpad's HID interface. No-op when this trackpad isn't
    /// the routing destination — there's nothing for the keymap-press
    /// to feed.
    async fn on_mouse_buttons_event(&mut self, _event: MouseButtonsEvent) {
        if !destination_matches(self.id) {
            return;
        }
        let now = Instant::now();
        let mut out: heapless::Vec<TrackpadReport, MAX_OUTPUTS> = heapless::Vec::new();
        if self.current_mode() == MODE_PTP {
            self.ptp
                .process_button_event(&self.params, now, self.routed_buttons(0b111), &mut out);
        } else {
            self.legacy.process_button_event(self.routed_buttons(0b111), &mut out);
        }
        for r in out {
            send(r).await;
        }
    }
}

// ============================================================================
// Helpers
// ============================================================================

async fn send(r: TrackpadReport) {
    USB_REPORT_CHANNEL.send(Report::TrackpadReport(r)).await;
}

fn scale(d: i32, num: u16, shift: u8) -> i16 {
    let v = (d * num as i32) >> shift;
    v.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

fn clamp_i8(v: i16) -> i8 {
    v.clamp(i8::MIN as i16, i8::MAX as i16) as i8
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn descriptor_fits_in_buffer() {
        let mut buf = [0u8; TRACKPAD_DESC_BUF_SIZE];
        let dims = TrackpadDimensions::from_mm(2048, 3072, 60, 90);
        let len = write_trackpad_descriptor(&mut buf, dims);
        assert!(len <= TRACKPAD_DESC_BUF_SIZE, "{} > {}", len, TRACKPAD_DESC_BUF_SIZE);
    }

    #[test]
    fn descriptor_size_snapshot() {
        // Pin the exact size so any descriptor edit is a deliberate
        // change. Bumping is fine; bump this number too.
        let mut buf = [0u8; 1024];
        let dims = TrackpadDimensions::from_mm(2048, 3072, 60, 90);
        let len = write_trackpad_descriptor(&mut buf, dims);
        assert_eq!(len, 499);
    }

    #[test]
    fn legacy_mouse_serialize() {
        let r = TrackpadLegacyMouseReport {
            buttons: 0b101,
            x: -3,
            y: 7,
        };
        let mut buf = [0u8; 4];
        let n = r.serialize(&mut buf).unwrap();
        assert_eq!(n, 4);
        assert_eq!(buf[0], 0x01);
        assert_eq!(buf[1], 0b101);
        assert_eq!(buf[2] as i8, -3);
        assert_eq!(buf[3], 7);
    }

    #[test]
    fn cu_per_mm_matches_panel() {
        // 2048 chip units across 60 mm = 34 cu/mm; 3072 across 90 mm = 34 cu/mm.
        let d = TrackpadDimensions::from_mm(2048, 3072, 60, 90);
        assert_eq!(d.cu_per_mm_x(), 34);
        assert_eq!(d.cu_per_mm_y(), 34);
        // Different physical mm per axis → different chip-units-per-mm,
        // motivating the per-axis cursor scale.
        let d = TrackpadDimensions::from_mm(2048, 3072, 50, 90);
        assert_eq!(d.cu_per_mm_x(), 40);
        assert_eq!(d.cu_per_mm_y(), 34);
    }

    #[test]
    fn ptp_serialize_layout() {
        let mut r = TrackpadPtpReport {
            scan_time: 0x1234,
            contact_count: 1,
            button: 1,
            ..Default::default()
        };
        r.fingers[0] = TrackpadPtpFinger {
            confidence: true,
            tip_switch: true,
            contact_id: 2,
            x: 0xCAFE,
            y: 0xBEEF,
        };
        let mut buf = [0u8; 1 + PTP_REPORT_PAYLOAD_BYTES];
        let n = r.serialize(&mut buf).unwrap();
        assert_eq!(n, 1 + PTP_REPORT_PAYLOAD_BYTES);
        assert_eq!(buf[0], 0x05);
        assert_eq!(buf[1], 0b11); // flags
        assert_eq!(buf[2], 2); // contact_id
        assert_eq!(&buf[3..5], &[0xFE, 0xCA]);
        assert_eq!(&buf[5..7], &[0xEF, 0xBE]);
        // Tail: scan_time(2) + contact_count(1) + button(1) sit at the end.
        let tail = 1 + (TRACKPAD_MAX_CONTACTS as usize) * PTP_FINGER_BYTES;
        assert_eq!(&buf[tail..tail + 2], &[0x34, 0x12]);
        assert_eq!(buf[tail + 2], 1);
        assert_eq!(buf[tail + 3], 1);
    }

    #[test]
    fn cu_per_mm_zero_dmm() {
        // Misconfigured panel — return 0, don't panic.
        let d = TrackpadDimensions {
            logical_max_x: 2048,
            logical_max_y: 3072,
            physical_dmm_x: 0,
            physical_dmm_y: 0,
        };
        assert_eq!(d.cu_per_mm_x(), 0);
        assert_eq!(d.cu_per_mm_y(), 0);
    }

    #[test]
    fn compute_scale_picks_per_axis_nums_with_shared_shift() {
        // Square pixels: same num both axes.
        let (nx, ny, shift) = compute_scale(3.0, 34, 34);
        assert_eq!(nx, ny);
        assert!(shift > 0);
        // Verify the encoded ratio matches the requested px/mm to within
        // one fixed-point ULP: 50 chip-units of motion at 34 cu/mm =
        // 1.47 mm, expected ≈ 4.41 mouse-units.
        let actual = (50i32 * nx as i32) >> shift;
        assert!((4..=5).contains(&actual), "{actual}");

        // Non-square pixels: x denser than y → smaller nx than ny.
        let (nx, ny, _) = compute_scale(3.0, 40, 30);
        assert!(
            nx < ny,
            "denser x axis -> fewer mouse units per chip unit ({nx} vs {ny})"
        );

        // Degenerate inputs disable cursor motion cleanly.
        assert_eq!(compute_scale(0.0, 34, 34), (0, 0, 0));
        assert_eq!(compute_scale(-1.0, 34, 34), (0, 0, 0));
        assert_eq!(compute_scale(3.0, 0, 34), (0, 0, 0));
    }

    #[test]
    fn from_mm_precomputes_tap_sq() {
        // 2 mm × 34 cu/mm = 68 cu → 4624 cu² squared budget.
        let p = TrackpadParams::from_mm(TrackpadDimensions::from_mm(2048, 3072, 60, 90), 2, 150, 450, 3.0);
        assert_eq!(p.tap_max_dev_chip_sq, 68 * 68);
    }

    // ====================================================================
    // Behavioural tests — drive the pure decoders directly.
    // ====================================================================

    use crate::event::{TrackpadEvent, TrackpadFinger, TrackpadFingers};

    /// Reference panel for the tests: TPS65-501b dims, default tap/hold
    /// thresholds. 2 mm of tap-distance budget converts to 68 chip-units
    /// at 34 cu/mm, so the squared threshold sits at 4624 cu².
    fn default_params() -> TrackpadParams {
        TrackpadParams::from_mm(
            TrackpadDimensions::from_mm(2048, 3072, 60, 90),
            /* tap_max_dev_mm */ 2,
            /* tap_time_ms */ 150,
            /* hold_time_ms */ 450,
            /* sensitivity_px_per_mm */ 3.0,
        )
    }

    fn finger_at(id: u8, x: u16, y: u16) -> TrackpadFinger {
        TrackpadFinger {
            id,
            x,
            y,
            touch_strength: 100,
            area: 0,
            tip: true,
            confidence: true,
        }
    }

    /// Builds successive [`TrackpadEvent`]s with finger 0's absolute
    /// position advancing by each call's `(dx, dy)`. Mirrors what the
    /// real chip would report when the finger moves: the processor
    /// derives cursor motion from the absolute path (the chip's
    /// relative is unreliable at touch onset), so motion tests need
    /// realistic absolute coordinates to exercise that code path.
    struct Path {
        x: i32,
        y: i32,
    }

    impl Path {
        /// Arbitrary base far from 0 and from `u16::MAX` so realistic
        /// motion in either direction stays in range.
        fn new() -> Self {
            Self { x: 1000, y: 1000 }
        }

        fn step(&mut self, n: usize, dx: i16, dy: i16) -> TrackpadEvent {
            self.x = self.x.saturating_add(dx as i32);
            self.y = self.y.saturating_add(dy as i32);
            let mut fs = TrackpadFingers::default();
            for i in 0..n {
                let _ = fs.push(finger_at(
                    i as u8,
                    self.x.clamp(0, u16::MAX as i32) as u16,
                    self.y.clamp(0, u16::MAX as i32) as u16,
                ));
            }
            TrackpadEvent { fingers: fs }
        }

        /// Lift `n` fingers — the event the iqs5xx driver actually emits
        /// on the cycle a touch ends: one `tip = false` record per slot
        /// at the contact's last known position (the chip transitions
        /// the slot to 0xFFFF on the *next* cycle, no record). The
        /// processor counts only `tip == true` fingers, so this lands
        /// on the `n == 0` lift branch — using an empty `TrackpadFingers`
        /// here would not exercise the same code path the real chip drives.
        fn lift(&self, n: usize) -> TrackpadEvent {
            let mut fs = TrackpadFingers::default();
            let x = self.x.clamp(0, u16::MAX as i32) as u16;
            let y = self.y.clamp(0, u16::MAX as i32) as u16;
            for i in 0..n {
                let _ = fs.push(TrackpadFinger {
                    id: i as u8,
                    x,
                    y,
                    touch_strength: 0,
                    area: 0,
                    tip: false,
                    confidence: true,
                });
            }
            TrackpadEvent { fingers: fs }
        }
    }

    fn at(t0: Instant, ms: u64) -> Instant {
        t0 + Duration::from_millis(ms)
    }

    fn run(state: &mut LegacyState, ev: &TrackpadEvent, now: Instant) -> heapless::Vec<TrackpadReport, MAX_OUTPUTS> {
        let params = default_params();
        let mut out = heapless::Vec::new();
        // Routed buttons = 0 — no keymap press in flight for these tests.
        state.process(ev, &params, now, 0, &mut out);
        out
    }

    fn legacy_buttons(r: &TrackpadReport) -> u8 {
        match r {
            TrackpadReport::LegacyMouse(m) => m.buttons,
            _ => panic!("expected legacy mouse report, got {:?}", r),
        }
    }
    fn legacy_xy(r: &TrackpadReport) -> (i8, i8) {
        match r {
            TrackpadReport::LegacyMouse(m) => (m.x, m.y),
            _ => panic!("expected legacy mouse report, got {:?}", r),
        }
    }

    #[test]
    fn stationary_one_finger_tap_fires_button_1_pulse_on_lift() {
        // Sanity check: the basic shape of the state machine. Touch,
        // hold still briefly, lift — get a press + release pair.
        let mut s = LegacyState::default();
        let t0 = Instant::from_ticks(0);
        let mut path = Path::new();

        assert!(run(&mut s, &path.step(1, 0, 0), at(t0, 0)).is_empty());
        // Lift well within `tap_time_ms` (150) and well under
        // `tap_max_dev_chip_sq` (deviation 0). Tap fires.
        let out = run(&mut s, &path.lift(1), at(t0, 50));
        assert_eq!(out.len(), 2, "tap = press + release ({out:?})");
        assert_eq!(legacy_buttons(&out[0]), 0b001, "press");
        assert_eq!(legacy_buttons(&out[1]), 0, "release");
        assert_eq!(legacy_xy(&out[0]), (0, 0));
    }

    #[test]
    fn two_finger_stationary_tap_fires_button_2_pulse_on_lift() {
        let mut s = LegacyState::default();
        let t0 = Instant::from_ticks(0);
        let mut path = Path::new();

        assert!(run(&mut s, &path.step(2, 0, 0), at(t0, 0)).is_empty());
        let out = run(&mut s, &path.lift(2), at(t0, 50));
        assert_eq!(out.len(), 2);
        assert_eq!(legacy_buttons(&out[0]), 0b010, "two-finger tap = button 2");
        assert_eq!(legacy_buttons(&out[1]), 0);
    }

    #[test]
    fn motion_laden_touch_does_not_fire_tap() {
        // Real-device scenario: the user starts dragging the cursor
        // immediately. The session's deviation passes `tap_max_dev_chip_sq`
        // before the lift, so no tap fires.
        let mut s = LegacyState::default();
        let t0 = Instant::from_ticks(0);
        let mut path = Path::new();

        assert!(run(&mut s, &path.step(1, 0, 0), at(t0, 0)).is_empty());
        // First motion frame: held in pending_motion, not emitted yet.
        // Step puts the finger 50√2 ≈ 70 cu from start — past the
        // 68-cu deviation threshold (2 mm × 34 cu/mm), so this frame
        // already disqualifies the session for tap.
        assert!(run(&mut s, &path.step(1, 50, 50), at(t0, 30)).is_empty());
        // Second motion frame surfaces the first as cursor motion.
        let mid = run(&mut s, &path.step(1, 50, 50), at(t0, 60));
        assert_eq!(mid.len(), 1);
        assert_ne!(legacy_xy(&mid[0]), (0, 0), "previously buffered motion surfaces");
        assert_eq!(legacy_buttons(&mid[0]), 0);

        // Lift: buffered (50, 50) is dropped (one-frame lift suppression),
        // and the deviation past TAP_DIST disqualifies the tap. No press
        // pulse fires.
        let out = run(&mut s, &path.lift(1), at(t0, 90));
        assert!(out.is_empty(), "no tap and no surfaced motion on lift ({out:?})");
    }

    #[test]
    fn software_press_and_hold_latches_button_then_drags_and_releases() {
        let mut s = LegacyState::default();
        let t0 = Instant::from_ticks(0);
        let mut path = Path::new();

        // Touchdown plus continued press well before hold_time elapses.
        // Both frames may surface a (0, 0) report (the prev-frame
        // motion buffer fires whenever there's a finger present), but
        // the buttons must stay 0 throughout.
        let out = run(&mut s, &path.step(1, 0, 0), at(t0, 0));
        for r in &out {
            assert_eq!(legacy_buttons(r), 0, "no buttons before hold_time");
        }
        let out = run(&mut s, &path.step(1, 0, 0), at(t0, 200));
        for r in &out {
            assert_eq!(legacy_buttons(r), 0, "no buttons before hold_time");
        }

        // First frame past hold_time (450 ms): hold latches, button-1
        // goes down. (Motion is zero and any buffered prev-frame motion
        // is also zero, so the report carries no displacement.)
        let out = run(&mut s, &path.step(1, 0, 0), at(t0, 460));
        assert_eq!(out.len(), 1);
        assert_eq!(legacy_buttons(&out[0]), 0b001);
        assert_eq!(legacy_xy(&out[0]), (0, 0));

        // Drag motion under the held button. Use 50-cu steps so the
        // 3/32 cursor scale doesn't truncate to 0. The buffered frame
        // surfaces on the next call.
        let _ = run(&mut s, &path.step(1, 50, 0), at(t0, 475));
        let out = run(&mut s, &path.step(1, 50, 0), at(t0, 488));
        assert!(!out.is_empty());
        assert_eq!(legacy_buttons(&out[0]), 0b001);
        let (dx, _) = legacy_xy(&out[0]);
        assert!(dx > 0, "drag motion surfaces with button held ({dx})");

        // Lift: buffered drag-frame motion is dropped by lift
        // suppression, button 1 releases. Hold-latched sessions are
        // excluded from tap evaluation, so no synthetic press/release
        // fires either.
        let out = run(&mut s, &path.lift(1), at(t0, 501));
        assert_eq!(out.len(), 1);
        assert_eq!(legacy_buttons(&out[0]), 0);
        assert_eq!(legacy_xy(&out[0]), (0, 0));
    }

    #[test]
    fn hold_does_not_latch_with_motion_or_two_fingers() {
        // Deviation past tap_max_dev disqualifies hold for the rest of
        // the session, even after the touch settles down.
        let mut s = LegacyState::default();
        let t0 = Instant::from_ticks(0);
        let mut path = Path::new();
        let _ = run(&mut s, &path.step(1, 0, 0), at(t0, 0));
        // Step puts the finger 50√2 ≈ 70 cu from start, past the 68-cu
        // threshold.
        let _ = run(&mut s, &path.step(1, 50, 50), at(t0, 30));
        let out = run(&mut s, &path.step(1, 0, 0), at(t0, 460));
        for r in &out {
            assert_eq!(legacy_buttons(r), 0, "deviation disqualifies hold ({r:?})");
        }

        // Two-finger sessions never qualify — chord behavior is
        // reserved for two-finger tap (right-click), not drag.
        let mut s = LegacyState::default();
        let mut path = Path::new();
        let _ = run(&mut s, &path.step(2, 0, 0), at(t0, 0));
        let out = run(&mut s, &path.step(2, 0, 0), at(t0, 460));
        for r in &out {
            assert_eq!(legacy_buttons(r), 0, "two-finger touch must not latch hold ({r:?})");
        }
    }

    #[test]
    fn lift_suppresses_prior_frame_centroid_shift_jump() {
        // Capacitive trackpad rolloff: as the finger lifts, the chip's
        // centroid jumps as the contact patch shrinks asymmetrically.
        // The processor must drop the buffered last-frame motion so the
        // cursor doesn't teleport on release.
        let mut s = LegacyState::default();
        let t0 = Instant::from_ticks(0);
        let mut path = Path::new();

        // Touchdown plus normal tracking. The buffer adds one frame of
        // delay, so the first emitted (50, 0) lands at the third
        // process() call rather than the second. 50-cu steps so the
        // 3/32 cursor scale doesn't truncate to 0.
        assert!(run(&mut s, &path.step(1, 0, 0), at(t0, 0)).is_empty());
        let _ = run(&mut s, &path.step(1, 50, 0), at(t0, 13));
        let out = run(&mut s, &path.step(1, 50, 0), at(t0, 26));
        assert_eq!(out.len(), 1);
        let (dx_normal, _) = legacy_xy(&out[0]);
        assert!(dx_normal > 0, "previously-buffered motion surfaces normally");

        // Final frame with finger reports the centroid-shift jump.
        // (500, 0) is the spurious displacement; the previous frame's
        // (50, 0) surfaces here, the (500, 0) is parked in
        // pending_motion awaiting next frame's verdict.
        let out = run(&mut s, &path.step(1, 500, 0), at(t0, 39));
        assert_eq!(out.len(), 1);
        let (dx_pre_lift, _) = legacy_xy(&out[0]);
        assert!(
            (1..30).contains(&dx_pre_lift),
            "modest prior-frame motion, not the jump ({dx_pre_lift})"
        );

        // Lift: buffered (500, 0) is the centroid-shift artifact and
        // must NOT surface as cursor motion.
        let out = run(&mut s, &path.lift(1), at(t0, 52));
        for r in &out {
            assert_eq!(legacy_xy(r), (0, 0), "lift suppresses last-frame motion ({r:?})");
        }

        // After the lift, pending_motion is cleared and prev_xy resets;
        // the next touch starts fresh.
        let mut path = Path::new();
        assert!(run(&mut s, &path.step(1, 0, 0), at(t0, 200)).is_empty());
        let _ = run(&mut s, &path.step(1, 70, 0), at(t0, 213));
        let out = run(&mut s, &path.step(1, 70, 0), at(t0, 226));
        assert_eq!(out.len(), 1);
        let (dx, _) = legacy_xy(&out[0]);
        assert!(dx > 0, "new session emits its own buffered motion");
    }

    #[test]
    fn cursor_motion_derived_from_absolute_position_not_chip_relative() {
        // The processor's only input for motion is the per-finger
        // absolute position: `TrackpadEvent` doesn't carry a chip-side
        // relative dx/dy. This test pins that contract — events with
        // identical relative semantics but different absolute paths
        // produce different cursor deltas.
        let mut s = LegacyState::default();
        let t0 = Instant::from_ticks(0);

        let event_at = |x: u16, y: u16| TrackpadEvent {
            fingers: {
                let mut fs = TrackpadFingers::default();
                let _ = fs.push(finger_at(0, x, y));
                fs
            },
        };

        assert!(run(&mut s, &event_at(1216, 2288), at(t0, 0)).is_empty());
        // Move (-6, -20). Buffered for one cycle.
        assert!(run(&mut s, &event_at(1210, 2268), at(t0, 13)).is_empty());
        // Move (-8, -26). The buffered (-6, -20) surfaces.
        let out = run(&mut s, &event_at(1202, 2242), at(t0, 26));
        assert_eq!(out.len(), 1);
        let (dx, dy) = legacy_xy(&out[0]);
        assert!(dx < 0 && dy < 0, "negative motion surfaces ({dx},{dy})");
    }
}
