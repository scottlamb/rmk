//! Implementation of the function-style `central_init!` proc macro.
//!
//! Reads `keyboard.toml`, then emits a sequence of statements that the
//! caller's own `fn main()` consumes inline. The end of the emission is
//! a `let rmk = ::rmk::builder::Rmk::new(...).with_usb(...).with_keymap(
//! ...).with_storage(...).with_matrix(...).with_split_central_uart(...);`
//! ready for the caller to chain `.with_input_device(...)` /
//! `.with_usb_extend(...)` before calling `.run()`.
//!
//! Scope: RP2040 + USB-only + split-central. Other chips/configurations
//! follow the same pattern; this is the minimum-viable implementation
//! used to validate the API shape against `rmk-sofleplus2` before
//! upstream alignment.

use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use rmk_config::resolved::hardware::{BoardConfig, ChipSeries};
use syn::parse::{Parse, ParseStream};
use syn::{Ident, Token};

use super::behavior::expand_behavior_config;
use super::chip::bind_interrupt::expand_bind_interrupt;
use super::chip::comm::usb_config_default;
use super::display::expand_display_config;
use super::feature::{get_rmk_features, is_feature_enabled};
use super::input_device::encoder::expand_encoder_device;
use super::input_device::iqs5xx::{expand_iqs5xx_device, expand_mouse_button_routing};
use super::keyboard_config::{expand_keyboard_info, expand_vial_config, read_keyboard_toml_config};
use super::layout::expand_default_keymap;
use super::matrix::{
    expand_bootmagic_check, expand_matrix_config, expand_matrix_input_output_pins,
};
use super::split::central::{expand_serial_init, expand_split_central_config};

/// Parsed input to `central_init!(p, Irqs, rmk_config, flash)`.
///
/// The flash binding is caller-supplied so the user can read the
/// hardware unique ID off it before handing it over (e.g. to derive a
/// runtime USB serial number — see `rmk-sofleplus2/src/central.rs`).
/// `expand_flash_init`'s default `let flash = Flash::new(p.FLASH, …)`
/// would otherwise consume `p.FLASH` a second time.
struct CentralInitArgs {
    peripherals: Ident,
    _irqs: Ident,
    rmk_config: Ident,
    flash: Ident,
}

impl Parse for CentralInitArgs {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let peripherals: Ident = input.parse()?;
        let _: Token![,] = input.parse()?;
        let irqs: Ident = input.parse()?;
        let _: Token![,] = input.parse()?;
        let rmk_config: Ident = input.parse()?;
        let _: Token![,] = input.parse()?;
        let flash: Ident = input.parse()?;
        Ok(Self {
            peripherals,
            _irqs: irqs,
            rmk_config,
            flash,
        })
    }
}

/// Module-level emission for `rmk::central_setup!()`. Emits the items that
/// have to live at module scope: `KEYBOARD_DEVICE_CONFIG`, `ROW`/`COL`/
/// `NUM_LAYER`/`NUM_ENCODER` consts, `get_default_keymap`, `get_default_encoder_map`,
/// `VIAL_KEYBOARD_DEF` / `VIAL_KEYBOARD_ID` / `VIAL_CONFIG` statics, and
/// `bind_interrupts!(struct Irqs { … })`. Together with the inline emission
/// from [`expand_central_init`], these replace what `keymap.rs` and the
/// hand-written `bind_interrupts!` do today.
///
/// Crate-level imports (`panic_probe`, `defmt_rtt`) that the existing
/// `#[rmk_keyboard]` codegen emits are *not* emitted here — out-of-tree
/// users may already use a different panic handler (e.g. `panic_persist`)
/// or logger, and we want to leave those choices to the user.
pub(crate) fn expand_central_setup(_input: TokenStream2) -> TokenStream2 {
    let keyboard_config = read_keyboard_toml_config();
    let identity = match keyboard_config.identity() {
        Ok(i) => i,
        Err(e) => {
            return syn::Error::new(proc_macro2::Span::call_site(), format!("identity: {e}"))
                .to_compile_error();
        }
    };
    let host = keyboard_config.host();
    let hardware = match keyboard_config.hardware() {
        Ok(h) => h,
        Err(e) => {
            return syn::Error::new(proc_macro2::Span::call_site(), format!("hardware: {e}"))
                .to_compile_error();
        }
    };
    let behavior = match keyboard_config.behavior() {
        Ok(b) => b,
        Err(e) => {
            return syn::Error::new(proc_macro2::Span::call_site(), format!("behavior: {e}"))
                .to_compile_error();
        }
    };
    let layout = match keyboard_config.layout() {
        Ok(l) => l,
        Err(e) => {
            return syn::Error::new(proc_macro2::Span::call_site(), format!("layout: {e}"))
                .to_compile_error();
        }
    };

    let keyboard_info = expand_keyboard_info(&identity, &layout);
    let default_keymap = expand_default_keymap(&layout, &behavior);
    let vial_config = expand_vial_config(&host);
    // `expand_bind_interrupt` wants an `ItemMod` so it can scan for
    // user-supplied `#[Overwritten(bind_interrupts)]` items. For the
    // function-style macro we have no such mod — pass an empty one.
    let empty_mod: syn::ItemMod = syn::parse_quote! { mod __rmk_central_setup_dummy {} };
    let bind_interrupt = expand_bind_interrupt(&hardware, &empty_mod);

    quote! {
        #keyboard_info
        #vial_config
        #default_keymap
        #bind_interrupt
    }
}

/// Module-level emission for `rmk::macros::keymap_setup!()`. Emits
/// only the matrix-shape consts and `get_default_keymap` /
/// `get_default_encoder_map` functions — see the public macro doc on
/// `lib.rs::keymap_setup`.
pub(crate) fn expand_keymap_setup(_input: TokenStream2) -> TokenStream2 {
    let keyboard_config = read_keyboard_toml_config();
    let identity = match keyboard_config.identity() {
        Ok(i) => i,
        Err(e) => {
            return syn::Error::new(proc_macro2::Span::call_site(), format!("identity: {e}"))
                .to_compile_error();
        }
    };
    let behavior = match keyboard_config.behavior() {
        Ok(b) => b,
        Err(e) => {
            return syn::Error::new(proc_macro2::Span::call_site(), format!("behavior: {e}"))
                .to_compile_error();
        }
    };
    let layout = match keyboard_config.layout() {
        Ok(l) => l,
        Err(e) => {
            return syn::Error::new(proc_macro2::Span::call_site(), format!("layout: {e}"))
                .to_compile_error();
        }
    };

    // expand_keyboard_info emits ROW/COL/NUM_LAYER/NUM_ENCODER + a
    // KEYBOARD_DEVICE_CONFIG. The shape consts are required by
    // get_default_keymap's return type. KEYBOARD_DEVICE_CONFIG is
    // harmless to include — it's a const, no runtime cost — but we
    // don't really want it in shared modules. Slice it out by
    // re-emitting just the shape consts directly.
    let _ = identity; // identity is used to gate behavior in some paths; not needed here.
    let num_col = layout.cols as usize;
    let num_row = layout.rows as usize;
    let num_layer = layout.layers as usize;
    let total_num_encoder: usize = layout.encoder_counts.iter().sum();

    let default_keymap = expand_default_keymap(&layout, &behavior);

    quote! {
        pub const COL: usize = #num_col;
        pub const ROW: usize = #num_row;
        pub const NUM_LAYER: usize = #num_layer;
        pub const NUM_ENCODER: usize = #total_num_encoder;
        #default_keymap
    }
}

pub(crate) fn expand_central_init(input: TokenStream2) -> TokenStream2 {
    let args = match syn::parse2::<CentralInitArgs>(input) {
        Ok(a) => a,
        Err(e) => return e.to_compile_error(),
    };

    let keyboard_config = read_keyboard_toml_config();
    let hardware = match keyboard_config.hardware() {
        Ok(h) => h,
        Err(e) => {
            return syn::Error::new(
                proc_macro2::Span::call_site(),
                format!("hardware config: {e}"),
            )
            .to_compile_error();
        }
    };
    let behavior = match keyboard_config.behavior() {
        Ok(b) => b,
        Err(e) => {
            return syn::Error::new(
                proc_macro2::Span::call_site(),
                format!("behavior config: {e}"),
            )
            .to_compile_error();
        }
    };
    let layout = match keyboard_config.layout() {
        Ok(l) => l,
        Err(e) => {
            return syn::Error::new(
                proc_macro2::Span::call_site(),
                format!("layout config: {e}"),
            )
            .to_compile_error();
        }
    };

    if hardware.chip.series != ChipSeries::Rp2040 {
        return syn::Error::new(
            proc_macro2::Span::call_site(),
            "central_init! currently supports RP2040 only — extending to other chips is mechanical follow-up",
        )
        .to_compile_error();
    }
    if hardware.storage.is_none() {
        return syn::Error::new(
            proc_macro2::Span::call_site(),
            "central_init! requires `[storage]` in keyboard.toml",
        )
        .to_compile_error();
    }

    let split = match &hardware.board {
        BoardConfig::Split(split) => split,
        BoardConfig::UniBody(_) => {
            return syn::Error::new(
                proc_macro2::Span::call_site(),
                "central_init! currently supports split keyboards only",
            )
            .to_compile_error();
        }
    };

    // The existing helpers reference `p` and `Irqs` literally. For the
    // first pass we wrap the emitted code in a block that aliases the
    // caller's identifiers to those names — works because every output
    // binding (`rmk`, `keymap`) is hoisted out of the block via
    // `let-else`-like patterns... actually that won't work for let
    // bindings. Pragmatic alternative: require the caller to name their
    // peripherals `p` and their IRQ struct `Irqs`. The macro_rules!
    // version did the same hardcoding in practice; future iterations
    // can parameterise.
    //
    // Detect the mismatch and emit a helpful error rather than a
    // confusing borrow-check failure.
    if args.peripherals != "p" {
        return syn::Error::new(
            args.peripherals.span(),
            "central_init! currently requires the peripherals binding to be named `p`",
        )
        .to_compile_error();
    }

    let usb_init = usb_config_default(&hardware);
    // We don't call `expand_flash_init` — the caller supplies the
    // already-constructed `flash` binding so they can read the hardware
    // unique ID off it before we hand it to storage. We do still need
    // the toml-derived `storage_config` from there, so emit it directly.
    let storage_config_init = match hardware.storage.as_ref() {
        Some(storage) => {
            let num_sectors = storage.num_sectors;
            let start_addr = storage.start_addr;
            let clear_storage = storage.clear_storage;
            let clear_layout = storage.clear_layout;
            quote! {
                let storage_config = ::rmk::config::StorageConfig {
                    num_sectors: #num_sectors,
                    start_addr: #start_addr,
                    clear_storage: #clear_storage,
                    clear_layout: #clear_layout,
                };
            }
        }
        None => quote! {},
    };
    let behavior_config = expand_behavior_config(&behavior);
    let rmk_features = get_rmk_features();
    let matrix_config = expand_matrix_config(&hardware, &rmk_features);
    // Bootmagic check on the central's matrix. Emits a no-op token stream
    // when `[split.central.matrix].bootmagic` is absent from the toml.
    let bootmagic = expand_bootmagic_check(&split.central.matrix);

    // Keymap data + storage initialisation. We don't reuse the existing
    // `expand_keymap_and_storage` helper because it leaves `keymap_data`,
    // `behavior_config`, and `per_key_config` as local stack bindings —
    // their lifetimes infect the resulting `KeyMap<'a>`, which then can't
    // sit in a `StaticCell<KeyMap<'static>>`. Instead, wrap each through
    // a `StaticCell` so the result is `KeyMap<'static>` outright.
    let row = layout.rows as usize;
    let col = layout.cols as usize;
    let num_layer_const = layout.layers as usize;
    let total_num_encoder: usize = layout.encoder_counts.iter().sum();
    let keymap_data_init = if total_num_encoder == 0 {
        quote! { ::rmk::KeymapData::new(get_default_keymap()) }
    } else {
        quote! { ::rmk::KeymapData::new_with_encoder(get_default_keymap(), get_default_encoder_map()) }
    };

    // Encoder devices on the central. Each `[[split.central.input_device.encoder]]`
    // entry becomes a `let mut encoder_<id> = …;` binding plus a
    // `.with_input_device(encoder_<id>)` call on the builder. The encoder
    // ids start at 0 (no peripheral encoders precede them on the central
    // build).
    let central_input_devices = split.central.input_device.clone().unwrap_or_default();
    let central_encoder_configs = central_input_devices.encoder.clone().unwrap_or_default();
    let (encoder_initializers, _encoder_processor_initializers) =
        expand_encoder_device(0, central_encoder_configs, &hardware.chip);
    let encoder_inits: Vec<_> = encoder_initializers
        .iter()
        .map(|i| &i.initializer)
        .collect();
    let encoder_var_names: Vec<_> = encoder_initializers.iter().map(|i| &i.var_name).collect();

    // Display: `[split.central.display]` builds a `display_processor` if
    // present. Same `.with_input_device(...)` wiring as encoders.
    let (display_init, display_var_name) = match split.central.display.as_ref() {
        Some(display_config) => {
            let (init, processor) = expand_display_config(&hardware.chip.series, display_config);
            let processor_init = processor.initializer;
            let processor_var = processor.var_name;
            (
                quote! {
                    #init
                    #processor_init
                },
                Some(processor_var),
            )
        }
        None => (quote! {}, None),
    };
    let display_with_input_device = match display_var_name {
        Some(v) => quote! { .with_input_device(#v) },
        None => quote! {},
    };

    // IQS5xx processors. The processor lives on the central regardless
    // of whether the device sits in `[split.central.…iqs5xx]` or
    // `[[split.peripheral.…iqs5xx]]`: the central is where `keymap`
    // lives and where TrackpadEvents land after travelling the split
    // link. `expand_iqs5xx_device` returns `(devices, processors)`; the
    // processor list is non-empty only when the user provided panel
    // dimensions on the iqs5xx config (and `feature = "ptp"` is on at
    // build time). For peripheral-side iqs5xx the device initializers
    // live on the peripheral side (emitted by `peripheral_init!`); we
    // discard them here and keep only the processors.
    let routing_target_name = central_input_devices
        .mouse_button_routing
        .as_ref()
        .and_then(|r| r.trackpad.clone());
    let central_iqs5xx_configs = central_input_devices.iqs5xx.clone().unwrap_or_default();

    // Build the (name, slot id) pairing across every iqs5xx the
    // firmware will instantiate — central, then each peripheral in
    // emission order. Slot ids match the `id` passed to
    // `TrackpadHidProcessor::new`. Used by `expand_mouse_button_routing`
    // to resolve the routing target by name across both halves.
    let mut iqs5xx_slots: Vec<(String, u8)> = Vec::new();
    for sensor in &central_iqs5xx_configs {
        let id = iqs5xx_slots.len() as u8;
        iqs5xx_slots.push((sensor.name.clone(), id));
    }
    for peripheral in &split.peripheral {
        let configs = peripheral
            .input_device
            .as_ref()
            .and_then(|id| id.iqs5xx.as_ref())
            .cloned()
            .unwrap_or_default();
        for sensor in &configs {
            let id = iqs5xx_slots.len() as u8;
            iqs5xx_slots.push((sensor.name.clone(), id));
        }
    }
    let routing_setup = expand_mouse_button_routing(routing_target_name.as_deref(), &iqs5xx_slots);

    let (central_iqs5xx_dev_inits, central_iqs5xx_proc_inits) =
        expand_iqs5xx_device(central_iqs5xx_configs, &hardware.chip);
    let mut iqs5xx_device_inits: Vec<TokenStream2> =
        central_iqs5xx_dev_inits.into_iter().map(|i| i.initializer).collect();
    let mut iqs5xx_processor_inits: Vec<TokenStream2> = Vec::new();
    let mut iqs5xx_processor_var_names: Vec<Ident> = Vec::new();
    for i in central_iqs5xx_proc_inits {
        iqs5xx_processor_inits.push(i.initializer);
        iqs5xx_processor_var_names.push(i.var_name);
    }
    for peripheral in &split.peripheral {
        let configs = peripheral
            .input_device
            .as_ref()
            .and_then(|id| id.iqs5xx.as_ref())
            .cloned()
            .unwrap_or_default();
        let (_, proc_inits) = expand_iqs5xx_device(configs, &hardware.chip);
        for i in proc_inits {
            iqs5xx_processor_inits.push(i.initializer);
            iqs5xx_processor_var_names.push(i.var_name);
        }
    }
    // Suppress unused-variable warning when no central-side iqs5xx
    // dev initializers exist (peripheral-only setup).
    let _ = &mut iqs5xx_device_inits;

    let num_layer = layout.layers as usize;
    let total_num_encoders: usize = layout.encoder_counts.iter().sum();

    // Central's local matrix dimensions and offset into the global keymap.
    let central_row = split.central.rows;
    let central_col = split.central.cols;
    let central_row_offset = split.central.row_offset;
    let central_col_offset = split.central.col_offset;
    let col2row = !split.central.matrix.row2col;

    // Split-central UART setup. `expand_split_central_config` emits its
    // own `bind_interrupts!(struct IrqsUart0 { … })` block (separate from
    // the user-facing `Irqs` from `central_setup!()`), plus the
    // `let pio0 = ::rmk::split::rp::uart::BufferedUart::…` binding. The
    // variable name follows lowercased instance name (e.g. `PIO0` →
    // `pio0`).
    let split_central_config = expand_split_central_config(&hardware);
    let serial = match split.central.serial.as_ref().and_then(|v| v.first()) {
        Some(s) => s,
        None => {
            return syn::Error::new(
                proc_macro2::Span::call_site(),
                "central_init! requires [split.central.serial] for the split UART",
            )
            .to_compile_error();
        }
    };
    let uart_var = syn::parse_str::<Ident>(&serial.instance.to_lowercase()).unwrap();

    // Peripheral matrix dimensions for `with_split_central_uart` const
    // generics — those describe where the peripheral's keys go in the
    // global keymap, not the central's local matrix.
    let peripheral = match split.peripheral.first() {
        Some(p) => p,
        None => {
            return syn::Error::new(
                proc_macro2::Span::call_site(),
                "central_init! requires at least one [[split.peripheral]]",
            )
            .to_compile_error();
        }
    };
    let per_row = peripheral.rows;
    let per_col = peripheral.cols;
    let per_row_offset = peripheral.row_offset;
    let per_col_offset = peripheral.col_offset;

    let rmk_config_ident = &args.rmk_config;
    let user_flash_ident = &args.flash;

    quote! {
        // USB driver, flash, matrix pin extraction, behavior_config,
        // (keymap_data + initialize_keymap_and_storage) → produce
        // `driver`, `flash`, `row_pins`, `col_pins`, `behavior_config`,
        // `keymap_data`, `keymap`, `storage`, `per_key_config`.
        #usb_init
        // Adopt the caller's already-constructed flash under the
        // canonical name `flash` that `expand_keymap_and_storage`
        // expects.
        let flash = #user_flash_ident;
        #storage_config_init
        #behavior_config
        #matrix_config
        // Bootmagic: jump straight to the bootloader if the configured
        // corner key is held during init. Must run after pin extraction
        // but before `Matrix::new` consumes the arrays.
        #bootmagic

        // Keymap data, behavior config, positional config — all need to
        // be 'static so the resulting `KeyMap<'static>` can sit in a
        // `StaticCell<KeyMap<'static>>` and the builder slot can hold a
        // single-lifetime `&'static KeyMap<'static>`. The
        // `static FOO: StaticCell<…>` declarations are scoped to this
        // function but their data is 'static regardless.
        static __RMK_KEYMAP_DATA: ::static_cell::StaticCell<
            ::rmk::KeymapData<{ #row }, { #col }, { #num_layer_const }, { #total_num_encoder }>,
        > = ::static_cell::StaticCell::new();
        let keymap_data = __RMK_KEYMAP_DATA.init(#keymap_data_init);
        static __RMK_BEHAVIOR_CONFIG: ::static_cell::StaticCell<::rmk::config::BehaviorConfig> = ::static_cell::StaticCell::new();
        let behavior_config = __RMK_BEHAVIOR_CONFIG.init(behavior_config);
        static __RMK_POSITIONAL_CONFIG: ::static_cell::StaticCell<
            ::rmk::config::PositionalConfig<{ #row }, { #col }>,
        > = ::static_cell::StaticCell::new();
        let per_key_config = __RMK_POSITIONAL_CONFIG.init(::rmk::config::PositionalConfig::default());

        let (keymap, mut storage) = ::rmk::initialize_keymap_and_storage(
            keymap_data,
            flash,
            &storage_config,
            behavior_config,
            per_key_config,
        )
        .await;

        // Split-link UART for run_peripheral_manager. Emits its own
        // `bind_interrupts!(struct IrqsUart0 { … })` block plus a
        // `let pio0 = …;` binding (variable name = lowercased instance).
        #split_central_config

        // Promote KeyMap into a StaticCell so the builder slot can hold a
        // single-lifetime `&'static KeyMap<'static>`.
        static __RMK_KEYMAP_CELL: ::static_cell::StaticCell<::rmk::keymap::KeyMap<'static>> = ::static_cell::StaticCell::new();
        let keymap: &'static ::rmk::keymap::KeyMap<'static> = __RMK_KEYMAP_CELL.init(keymap);

        // Matrix construction (using the row_pins / col_pins introduced
        // by `expand_matrix_config` above). 5 const generics: ROW, COL,
        // COL2ROW, ROW_OFFSET, COL_OFFSET — same as the existing
        // `#[rmk_keyboard]` codegen path.
        let __rmk_debouncer = ::rmk::debounce::default_debouncer::DefaultDebouncer::new();
        let __rmk_matrix = ::rmk::matrix::Matrix::<
            _,
            _,
            _,
            #central_row,
            #central_col,
            #col2row,
            #central_row_offset,
            #central_col_offset,
        >::new(row_pins, col_pins, __rmk_debouncer);

        // Encoder devices from `[[split.central.input_device.encoder]]`.
        // Each emits `let mut encoder_<id> = RotaryEncoder::…;` bindings.
        #( #encoder_inits )*

        // Display from `[split.central.display]`, if any. Emits the I²C/SPI
        // bus, the panel struct, and `let mut display_processor = …;`.
        #display_init

        // Central-side IQS5xx device bindings (Iqs5xx::new etc.). The
        // matching processor instances are emitted next; peripheral-side
        // device init lives in `peripheral_init!`.
        #( #iqs5xx_device_inits )*

        // IQS5xx processors (TrackpadHidProcessor or similar) — one per
        // [[…iqs5xx]] entry that supplied panel dims, central or
        // peripheral side. Wired via `.with_input_device(...)` below.
        #( #iqs5xx_processor_inits )*

        // Mouse-button routing setup. Empty when the user didn't set
        // `[input_device.mouse_button_routing.trackpad]`; a
        // `set_mouse_button_destination(Some(id))` call when the named
        // trackpad resolved to a slot; a `compile_error!` when the
        // name didn't match anything.
        #routing_setup

        // Builder construction. Adds USB driver, keymap, storage, matrix,
        // split-link UART, plus `.with_input_device(...)` for each encoder
        // and the display processor. The caller can chain further
        // `.with_input_device` / `.with_usb_extend` calls and then `.run()`
        // on the resulting `rmk`.
        let rmk = ::rmk::builder::Rmk::new(#rmk_config_ident)
            .with_usb(driver)
            .with_keymap(keymap)
            .with_storage(storage)
            .with_matrix(__rmk_matrix)
            .with_split_central_uart::<_, #per_row, #per_col, #per_row_offset, #per_col_offset>(
                0,
                #uart_var,
            )
            #( .with_input_device(#encoder_var_names) )*
            #( .with_input_device(#iqs5xx_processor_var_names) )*
            #display_with_input_device
            ;

        // Suppress unused-variable warning for layout/encoder counts the
        // helpers introduced but the user may not reference.
        let _ = (#num_layer, #total_num_encoders);
    }
}

/// Parsed input to `peripheral_init!(p)`.
struct PeripheralInitArgs {
    peripherals: Ident,
}

impl Parse for PeripheralInitArgs {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let peripherals: Ident = input.parse()?;
        Ok(Self { peripherals })
    }
}

/// Implementation of `rmk::macros::peripheral_init!(p)`. See the public
/// macro doc on `lib.rs::peripheral_init`.
pub(crate) fn expand_peripheral_init(input: TokenStream2) -> TokenStream2 {
    let args = match syn::parse2::<PeripheralInitArgs>(input) {
        Ok(a) => a,
        Err(e) => return e.to_compile_error(),
    };

    let keyboard_config = read_keyboard_toml_config();
    let hardware = match keyboard_config.hardware() {
        Ok(h) => h,
        Err(e) => {
            return syn::Error::new(proc_macro2::Span::call_site(), format!("hardware: {e}"))
                .to_compile_error();
        }
    };

    if hardware.chip.series != ChipSeries::Rp2040 {
        return syn::Error::new(
            proc_macro2::Span::call_site(),
            "peripheral_init! currently supports RP2040 only — extending to other chips is mechanical follow-up",
        )
        .to_compile_error();
    }

    let split = match &hardware.board {
        BoardConfig::Split(split) => split,
        BoardConfig::UniBody(_) => {
            return syn::Error::new(
                proc_macro2::Span::call_site(),
                "peripheral_init! is only valid for split keyboards",
            )
            .to_compile_error();
        }
    };

    if split.connection != "serial" {
        return syn::Error::new(
            proc_macro2::Span::call_site(),
            "peripheral_init! currently supports serial split only (no BLE)",
        )
        .to_compile_error();
    }

    // First peripheral entry. Multi-peripheral (id > 0) is a follow-up.
    let peripheral_config = match split.peripheral.first() {
        Some(p) => p,
        None => {
            return syn::Error::new(
                proc_macro2::Span::call_site(),
                "peripheral_init! requires at least one [[split.peripheral]]",
            )
            .to_compile_error();
        }
    };

    if args.peripherals != "p" {
        return syn::Error::new(
            args.peripherals.span(),
            "peripheral_init! currently requires the peripherals binding to be named `p`",
        )
        .to_compile_error();
    }

    // Matrix pin extraction + bootmagic + Matrix::new.
    let rmk_features = get_rmk_features();
    let async_matrix = is_feature_enabled(&rmk_features, "async_matrix");
    let row_pins = peripheral_config
        .matrix
        .row_pins
        .clone()
        .expect("split.peripheral.matrix.row_pins is required");
    let col_pins = peripheral_config
        .matrix
        .col_pins
        .clone()
        .expect("split.peripheral.matrix.col_pins is required");
    let pin_init = expand_matrix_input_output_pins(
        &hardware.chip,
        row_pins,
        col_pins,
        peripheral_config.matrix.row2col,
        async_matrix,
    );
    let bootmagic = expand_bootmagic_check(&peripheral_config.matrix);
    let local_row = peripheral_config.rows;
    let local_col = peripheral_config.cols;
    let col2row = !peripheral_config.matrix.row2col;

    // Split UART. The peripheral has at most one entry; `expand_serial_init`
    // emits its own internal `bind_interrupts!` block plus a
    // `let pio0 = …;` binding (variable name = lowercased instance).
    let serial = peripheral_config
        .serial
        .clone()
        .expect("[[split.peripheral.serial]] is required");
    if serial.len() != 1 {
        return syn::Error::new(
            proc_macro2::Span::call_site(),
            "peripheral expects exactly one [[split.peripheral.serial]] entry",
        )
        .to_compile_error();
    }
    let serial_init = expand_serial_init(&hardware.chip, serial);

    // IQS5xx device init from `[[split.peripheral.input_device.iqs5xx]]`.
    // The processor side lives on the central (emitted by central_init!);
    // here we just construct the device and its run-loop. Device idents
    // become `let mut <name>_id<id>_device = …;` from the codegen helper.
    //
    // The device init references `Irqs` for the I²C bus — the user's own
    // `bind_interrupts!(struct Irqs { … })` needs to include the matching
    // `I2C{N}_IRQ => i2c::InterruptHandler<…>;` entry, which is what
    // rmk-sofleplus2's existing peripheral.rs already does.
    let peripheral_iqs5xx_configs = peripheral_config
        .input_device
        .as_ref()
        .and_then(|id| id.iqs5xx.as_ref())
        .cloned()
        .unwrap_or_default();
    let (iqs5xx_device_initializers, _iqs5xx_processor_initializers) =
        expand_iqs5xx_device(peripheral_iqs5xx_configs, &hardware.chip);
    let iqs5xx_device_inits: Vec<_> = iqs5xx_device_initializers.iter().map(|i| &i.initializer).collect();
    // var_names not needed at the call site — the user references the
    // emitted bindings directly in `run_all!(...)`.
    let _ = &iqs5xx_device_initializers;

    quote! {
        // Matrix pin extraction.
        #pin_init

        // Bootmagic: jump straight to the bootloader if the configured
        // corner key is held during init. No-op if `bootmagic` isn't set
        // on `[[split.peripheral.matrix]]`.
        #bootmagic

        // Matrix.
        let __rmk_debouncer = ::rmk::debounce::default_debouncer::DefaultDebouncer::new();
        let mut matrix = ::rmk::matrix::Matrix::<
            _,
            _,
            _,
            #local_row,
            #local_col,
            #col2row,
        >::new(row_pins, col_pins, __rmk_debouncer);

        // Split-link UART. Emits a `bind_interrupts!(struct IrqsUart0 { … })`
        // block and a `let pio0 = …;` (or matching lowercased instance) so
        // the caller can hand it to `run_rmk_split_peripheral`.
        #serial_init

        // IQS5xx pointing-device(s) on the peripheral. Each emits a
        // `let mut <name>_id<id>_device = ::rmk::input_device::iqs5xx::Iqs5xx::new(...);`
        // binding (defaults to `iqs5xx_<idx>_id<id>_device` when the
        // toml entry has no `name`). Hand them to `run_all!` alongside
        // `matrix`.
        #( #iqs5xx_device_inits )*
    }
}
