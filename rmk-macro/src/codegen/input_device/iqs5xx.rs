use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use rmk_config::resolved::hardware::{ChipModel, ChipSeries, Iqs5xxConfig};

use super::Initializer;

/// Expand IQS5xx device configuration.
///
/// Returns (device initializers, processor initializers). The processor list
/// is currently always empty: the driver publishes `TrackpadEvent` and the
/// matching consumer (which translates to `PointingEvent` / a HID report)
/// has not landed yet, so this codegen wires up only the device. Users who
/// want a working pointer in the meantime can add a consumer in their app
/// layer.
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
    }

    (device_initializers, Vec::new())
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
