# Azoteq IQS5xx Trackpad

The Azoteq IQS5xx-B000 family (IQS550, IQS572, IQS525) are I²C capacitive
trackpad controllers, commonly used in keyboards via Azoteq's TPS43 and TPS65
trackpad modules.

::: note

- A working pointer (cursor + tap + drag) ships behind the optional
  `ptp` Cargo feature — see [HID output](#hid-output). Without `ptp`, the
  driver still publishes `TrackpadEvent`s but emits no HID reports of its
  own; you'd need to add a custom consumer.
- Multi-finger absolute positions, pressure, and area are read from the
  IC and surfaced through `TrackpadEvent`. Software scrolling on top of
  that is not yet implemented.
- An `RDY` (ready) pin is strongly recommended. Without it, the driver falls
  back to timed polling and may stall the I²C bus through clock-stretching if
  it polls mid-cycle. See [RDY vs polling](#rdy-vs-polling).
- Each `[[input_device.iqs5xx]]` claims its own I²C peripheral. Sharing a bus
  with another I²C device (e.g. an OLED) isn't supported yet.
- Persisting parameters to the IC's non-volatile memory (which requires
  toggling `NRST`) isn't supported; configuration is rewritten on every boot.

:::

## Hardware

- `SDA` / `SCL` — I²C bus, 7-bit address `0x74`.
- `RDY` — active-high digital output the device drives high during the I²C
  communication window. Connect to a GPIO that supports async edge waits
  (`embedded_hal_async::digital::Wait`).
- `NRST` — active-low reset. Not used by this driver, but required if you
  want to persist parameters to the IC's non-volatile memory (out of scope
  here).

## `toml` configuration

```toml
[[input_device.iqs5xx]]
name = "trackpad0"
id = 0 # optional 0-255. Used for debug prints. Defaults to 0.

i2c.instance = "I2C0"  # RP2040: I2C0 / I2C1.  nRF52: TWISPI0 / TWISPI1 / TWISPI2.
i2c.sda = "PIN_4"
i2c.scl = "PIN_5"

# Optional: RDY (data-ready) pin. Strongly recommended.
rdy = "PIN_15"

# Axis tweaks applied on-chip.Set whichever of these your physical mounting
# needs; defaults are all false.
# invert_x = true
# invert_y = true
# swap_xy = true

# --- Required for `feature = "ptp"` HID output (see "HID output" below) ---
# Panel physical extent in whole millimetres. Match what the trackpad
# module's datasheet says.
# physical_mm_x = 60
# physical_mm_y = 90
# Panel logical-coordinate maximums. The IQS5xx itself prints these on
# the first boot — `(rx_channels - 1) * 256` for X and likewise for Y.
# A TPS65-501b after `swap_xy` is 9 × 13 channels, so 2048 × 3072.
# logical_max_x = 2048
# logical_max_y = 3072

# --- Optional knobs, all with sensible defaults ---
# Tap / hold tuning (mm and ms).
# tap_max_dev_mm = 2     # max start-to-now deviation that still counts as a tap
# tap_time_ms = 150      # max duration that still counts as a tap
# hold_time_ms = 450     # stationary 1-finger hold latches button 1 (drag)
# Cursor sensitivity in mouse units per mm of finger motion (same
# convention macos-trackpad-companion uses for its PTP cursor scale).
# sensitivity = 25.0
```

### Split

To add the trackpad to the central or a peripheral:

```toml
[[split.central.input_device.iqs5xx]]
name = ...

# resp.
[[split.peripheral.input_device.iqs5xx]]
name = ...
```

For split keyboards the device runs on whichever side it's wired to; the
matching `PointingProcessor` is generated on the central automatically.

## HID output

The `ptp` Cargo feature adds a dedicated USB HID interface for the
trackpad — sibling to the keyboard's composite interface, so keyboard
input keeps working independently. The interface exposes two reports
sharing one Application Collection per type:

- **Legacy mouse** (Report ID `0x01`): cursor + integrated button. The
  default at boot. macOS, which does not natively bind PTP, stays in
  this mode and gets a working cursor + tap + drag without any
  userspace helper.
- **PTP touchpad** (Report ID `0x05`) plus the four spec-mandated
  Feature reports. The host opts in by writing Input Mode = 3 to
  Feature `0x08` (Linux's `hid-multitouch` does this on bind; Windows
  needs the PTPHQA certification blob in the descriptor before it will
  bind, which v1 does not include).

To enable, add `ptp` to your build's `rmk` feature list and provide the
panel parameters in your `[[input_device.iqs5xx]]` block:

```toml
[[input_device.iqs5xx]]
name = "trackpad0"
i2c.instance = "I2C0"
i2c.sda = "PIN_4"
i2c.scl = "PIN_5"
rdy = "PIN_15"

physical_mm_x = 60
physical_mm_y = 90
logical_max_x = 2048
logical_max_y = 3072
```

The processor is generated automatically — no Rust glue required, the
`#[rmk_keyboard]` macro wires it up when those four panel fields are set.
Cursor scaling is configurable; see the TOML snippet above.

### Tap and drag

In legacy-mouse mode the firmware's tap/hold state machine maps
finger-count transitions to button events:

- 1-finger tap (touch < `tap_time_ms`, deviation ≤ `tap_max_dev_mm`):
  emits a button-1 click pulse.
- 2-finger tap: button-2 click pulse.
- 1-finger stationary touch held ≥ `hold_time_ms`: latches button 1
  for the rest of the session, with subsequent finger motion emitted as
  cursor deltas — i.e. drag.

Keymap-pressed `MouseBtn1..8` keys still go to the keyboard's composite
mouse report. A follow-up commit lets you route them through a trackpad
HID interface so a `MouseBtn1` held while a finger moves on the surface
reads as a drag (the click and the contact then live on the same HID
device, which is the precondition for drag detection on macOS /
Windows).

::: note

The trackpad-HID processor must run on the **central** side, even if
the trackpad is wired to a peripheral. The peripheral runs the `Iqs5xx`
device and forwards events over the split link; the central converts
them to USB HID reports.

:::

## Rust configuration

If you're not using `keyboard.toml`, construct the device + processor
directly. For a split keyboard, run the device on whichever side the
trackpad is wired to and the processor on the central.

```rust
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::i2c::{Config, I2c};
use rmk::input_device::iqs5xx::{Iqs5xx, Iqs5xxConfig};
use rmk::input_device::trackpad_hid::{
    install_trackpad_descriptor,
    TrackpadDimensions, TrackpadHidProcessor, TrackpadParams,
};

// 1. Bring up the I2C bus the trackpad is on.
let mut i2c_cfg = Config::default();
i2c_cfg.frequency = 400_000;
let i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, i2c_cfg);
let rdy = Some(Input::new(p.PIN_15, Pull::None));

// 2. Construct the device.
const POINTING_DEV_ID: u8 = 0;
let mut trackpad = Iqs5xx::new(
    POINTING_DEV_ID,
    i2c,
    rdy,
    Iqs5xxConfig::default(),
);

// 3. Install the descriptor + construct the processor (central only).
//    Slot 0 is the only slot wired through USB right now.
const TRACKPAD_SLOT: u8 = 0;
let dims = TrackpadDimensions::from_mm(2048, 3072, 60, 90);
let params = TrackpadParams::from_mm(dims, 2, 150, 450, 3, 5);
let params = install_trackpad_descriptor(params);
let mut trackpad_proc = TrackpadHidProcessor::new(TRACKPAD_SLOT, params, &keymap);

run_all!(trackpad, trackpad_proc, /* matrix, ... */);
```

`install_trackpad_descriptor` must run before USB enumeration (i.e.
before `rmk.run().await`); it stashes the report descriptor for the HID
class to pick up at enumeration time.

## RDY vs polling

The IQS5xx alternates between _scanning_ the touch panel and an I²C
_communication window_. When a window is open it drives `RDY` high.

- **With `RDY`**: the driver waits for `RDY` high before issuing I²C reads,
  so transactions complete inside the window with no clock-stretching. The
  driver puts the IC into "event mode" so the device only opens a window when
  it actually has touch data, which keeps idle bus traffic minimal.
- **Without `RDY`** (`rdy = None` / no `rdy` in TOML): the driver issues
  reads on a fixed ~15 ms cadence. If a read lands mid-scan the IC
  clock-stretches SCL until the current cycle ends, freezing any device
  sharing the bus. The driver compensates with a conservative report
  interval and a longer per-transaction timeout, but you may still see
  latency spikes — particularly during long holds.

If your PCB doesn't route `RDY`, hand-soldering a jumper to a spare GPIO is
generally worth it.

## References

- [IQS5xx-B000 Trackpad and Touchpad Datasheet (Azoteq)](https://www.azoteq.com/images/stories/pdf/iqs5xx-b000_trackpad_datasheet.pdf)
