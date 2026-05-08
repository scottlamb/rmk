use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use rmk_config::resolved::hardware::{ChipModel, ChipSeries, Iqs5xxConfig};

use super::Initializer;

/// Expand IQS5xx device configuration.
///
/// Returns `(device initializers, processor initializers)`. The processor
/// list is non-empty only on builds with `feature = "ptp"`, where each
/// `[input_device.iqs5xx]` block also gets a `TrackpadHidProcessor` that
/// publishes legacy-mouse / PTP HID reports on a dedicated USB interface.
/// Without `ptp`, this codegen wires up only the device — there is no
/// default pointer output in the upstream tree until the user adds their
/// own consumer (or enables `ptp`).
pub(crate) fn expand_iqs5xx_device(
    iqs5xx_config: Vec<Iqs5xxConfig>,
    chip: &ChipModel,
) -> (Vec<Initializer>, Vec<Initializer>) {
    if iqs5xx_config.is_empty() {
        return (Vec::new(), Vec::new());
    }

    match chip.series {
        ChipSeries::Nrf52 | ChipSeries::Rp2040 => {}
        _ => {
            panic!("IQS5xx is only supported on nRF52 and RP2040 chips");
        }
    }

    let mut device_initializers = vec![];
    let mut processor_initializers: Vec<Initializer> = vec![];

    for (idx, sensor) in iqs5xx_config.iter().enumerate() {
        let sensor_id = sensor.id.unwrap_or(0);
        let sensor_name = if sensor.name.is_empty() {
            format!("iqs5xx_{}_id{}", idx, sensor_id)
        } else {
            format!("{}_id{}", sensor.name.clone(), sensor_id)
        };

        let device_ident = format_ident!("{}_device", sensor_name);
        let i2c_ident = format_ident!("{}_i2c", sensor_name);
        let rdy_ident = format_ident!("{}_rdy", sensor_name);
        let cfg_ident = format_ident!("{}_config", sensor_name);

        let instance_ident = format_ident!("{}", sensor.i2c.instance.to_uppercase());
        let sda_ident = format_ident!("{}", sensor.i2c.sda);
        let scl_ident = format_ident!("{}", sensor.i2c.scl);

        let invert_x = sensor.invert_x;
        let invert_y = sensor.invert_y;
        let swap_xy = sensor.swap_xy;
        let cfg_init = quote! {
            let #cfg_ident = ::rmk::input_device::iqs5xx::Iqs5xxConfig {
                invert_x: #invert_x,
                invert_y: #invert_y,
                swap_xy: #swap_xy,
            };
        };

        let rdy_init = match (&sensor.rdy, &chip.series) {
            (Some(rdy_pin), ChipSeries::Nrf52) => {
                let rdy_pin_ident = format_ident!("{}", rdy_pin);
                quote! {
                    let #rdy_ident = Some(::embassy_nrf::gpio::Input::new(
                        p.#rdy_pin_ident,
                        ::embassy_nrf::gpio::Pull::None,
                    ));
                }
            }
            (Some(rdy_pin), ChipSeries::Rp2040) => {
                let rdy_pin_ident = format_ident!("{}", rdy_pin);
                quote! {
                    let #rdy_ident = Some(::embassy_rp::gpio::Input::new(
                        p.#rdy_pin_ident,
                        ::embassy_rp::gpio::Pull::None,
                    ));
                }
            }
            (None, ChipSeries::Nrf52) => quote! {
                let #rdy_ident: Option<::embassy_nrf::gpio::Input<'static>> = None;
            },
            (None, ChipSeries::Rp2040) => quote! {
                let #rdy_ident: Option<::embassy_rp::gpio::Input<'static>> = None;
            },
            _ => unreachable!(),
        };

        let device_init = match chip.series {
            ChipSeries::Nrf52 => quote! {
                #cfg_init
                #rdy_init
                static #i2c_ident: ::static_cell::StaticCell<[u8; 16]> = ::static_cell::StaticCell::new();
                let #i2c_ident = #i2c_ident.init([0u8; 16]);
                let #i2c_ident = ::embassy_nrf::twim::Twim::new(
                    p.#instance_ident,
                    Irqs,
                    p.#sda_ident,
                    p.#scl_ident,
                    ::embassy_nrf::twim::Config::default(),
                    #i2c_ident,
                );
                let mut #device_ident = ::rmk::input_device::iqs5xx::Iqs5xx::new(
                    #sensor_id,
                    #i2c_ident,
                    #rdy_ident,
                    #cfg_ident,
                );
            },
            ChipSeries::Rp2040 => quote! {
                #cfg_init
                #rdy_init
                let #i2c_ident = ::embassy_rp::i2c::I2c::new_async(
                    p.#instance_ident,
                    p.#scl_ident,
                    p.#sda_ident,
                    Irqs,
                    ::embassy_rp::i2c::Config::default(),
                );
                let mut #device_ident = ::rmk::input_device::iqs5xx::Iqs5xx::new(
                    #sensor_id,
                    #i2c_ident,
                    #rdy_ident,
                    #cfg_ident,
                );
            },
            _ => unreachable!(),
        };

        device_initializers.push(Initializer {
            initializer: device_init,
            var_name: device_ident,
        });

        // PTP-mode HID consumer: emits legacy-mouse or PTP-touchpad reports
        // on a dedicated USB interface (`feature = "ptp"`). One processor
        // per `[input_device.iqs5xx]` block; the descriptor install runs
        // before USB enumeration via the processor's constructor side
        // effect.
        //
        // Opt-in: only emit when the user provided panel dimensions
        // (`physical_mm_x/y` + `logical_max_x/y`). Without those, the
        // descriptor would advertise a 0-by-0 panel and the cursor delta
        // scaling would degenerate, so it's better to leave the processor
        // off entirely than to emit silently-broken HID. Users who don't
        // want any HID output from this trackpad just leave the dimension
        // fields unset (the default).
        if sensor.physical_mm_x == 0
            || sensor.physical_mm_y == 0
            || sensor.logical_max_x == 0
            || sensor.logical_max_y == 0
        {
            continue;
        }
        let processor_ident = format_ident!("{}_processor", sensor_name);
        let slot_id: u8 = idx as u8;
        let logical_max_x = sensor.logical_max_x;
        let logical_max_y = sensor.logical_max_y;
        let physical_mm_x = sensor.physical_mm_x;
        let physical_mm_y = sensor.physical_mm_y;
        let tap_max_dev_mm = sensor.tap_max_dev_mm;
        let tap_time_ms = sensor.tap_time_ms;
        let hold_time_ms = sensor.hold_time_ms;
        let sensitivity = sensor.sensitivity;
        let processor_init = quote! {
            let #processor_ident = {
                let dims = ::rmk::input_device::trackpad_hid::TrackpadDimensions::from_mm(
                    #logical_max_x,
                    #logical_max_y,
                    #physical_mm_x,
                    #physical_mm_y,
                );
                let params = ::rmk::input_device::trackpad_hid::TrackpadParams::from_mm(
                    dims,
                    #tap_max_dev_mm,
                    #tap_time_ms,
                    #hold_time_ms,
                    #sensitivity,
                );
                let params = ::rmk::input_device::trackpad_hid::install_trackpad_descriptor(params);
                ::rmk::input_device::trackpad_hid::TrackpadHidProcessor::new(#slot_id, params, &keymap)
            };
        };
        processor_initializers.push(Initializer {
            initializer: processor_init,
            var_name: processor_ident,
        });
    }

    (device_initializers, processor_initializers)
}

/// Generate `bind_interrupts!` entries for the I²C peripherals used by IQS5xx
/// devices on `chip`. Returns an empty token stream if there are no devices.
pub(crate) fn expand_iqs5xx_interrupts(
    chip_series: &ChipSeries,
    iqs5xx_config: &[Iqs5xxConfig],
) -> TokenStream {
    if iqs5xx_config.is_empty() {
        return quote! {};
    }
    let entries = iqs5xx_config.iter().map(|sensor| {
        let instance = format_ident!("{}", sensor.i2c.instance.to_uppercase());
        match chip_series {
            ChipSeries::Nrf52 => quote! {
                #instance => ::embassy_nrf::twim::InterruptHandler<::embassy_nrf::peripherals::#instance>;
            },
            ChipSeries::Rp2040 => {
                let irq = format_ident!("{}_IRQ", sensor.i2c.instance.to_uppercase());
                quote! {
                    #irq => ::embassy_rp::i2c::InterruptHandler<::embassy_rp::peripherals::#instance>;
                }
            }
            _ => quote! {},
        }
    });
    quote! { #(#entries)* }
}
