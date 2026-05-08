use adc::expand_adc_device;
use encoder::expand_encoder_device;
use iqs5xx::{expand_iqs5xx_device, expand_mouse_button_routing};
use pmw33xx::expand_pmw33xx_device;
use pmw3610::expand_pmw3610_device;
use proc_macro2::{Ident, TokenStream};
use quote::quote;
use rmk_config::resolved::Hardware;
use rmk_config::resolved::hardware::{
    BleConfig, BoardConfig, CommunicationConfig, InputDeviceConfig, UniBodyConfig,
};

pub(crate) mod adc;
pub(crate) mod encoder;
pub(crate) mod iqs5xx;
pub(crate) mod pmw33xx;
pub(crate) mod pmw3610;

/// Initializer struct for input devices
pub(crate) struct Initializer {
    pub(crate) initializer: TokenStream,
    pub(crate) var_name: Ident,
}

/// Expands the input device configuration.
/// Returns a tuple containing: (device_and_processors_initialization, devices, processors)
pub(crate) fn expand_input_device_config(
    hardware: &Hardware,
) -> (TokenStream, Vec<TokenStream>, Vec<TokenStream>) {
    let mut initialization = TokenStream::new();
    let mut devices = Vec::new();
    let mut processors = Vec::new();

    // generate ADC configuration
    let communication = &hardware.communication;
    let ble_config = match communication {
        CommunicationConfig::Ble(ble_config) | CommunicationConfig::Both(_, ble_config) => {
            Some(ble_config.clone())
        }
        _ => None,
    };
    let board = &hardware.board;
    let chip = &hardware.chip;
    let (adc_initializers, adc_processors) = match board {
        BoardConfig::UniBody(UniBodyConfig { input_device, .. }) => expand_adc_device(
            input_device.clone().joystick.unwrap_or(Vec::new()),
            ble_config,
            chip.series.clone(),
        ),
        BoardConfig::Split(split_config) => {
            // For split central, read battery config from split.central instead of [ble]
            // This provides better consistency with peripheral configuration
            let central_ble_config = if split_config.central.battery_adc_pin.is_some() {
                // Validate: warn if both [ble] and [split.central] have different battery configs
                if let Some(ref ble_cfg) = ble_config
                    && ble_cfg.battery_adc_pin.is_some()
                {
                    let ble_matches = ble_cfg.battery_adc_pin
                        == split_config.central.battery_adc_pin
                        && ble_cfg.adc_divider_measured
                            == split_config.central.adc_divider_measured
                        && ble_cfg.adc_divider_total == split_config.central.adc_divider_total;

                    if !ble_matches {
                        eprintln!(
                            "warning: Battery configuration found in both [ble] and [split.central] sections with different values"
                        );
                        eprintln!(
                            "help: [split.central] configuration will be used. Remove [ble] battery config to avoid confusion."
                        );
                    }
                }

                // Central has its own battery config in [split.central]
                Some(BleConfig {
                    enabled: ble_config.as_ref().map(|c| c.enabled).unwrap_or(false),
                    battery_adc_pin: split_config.central.battery_adc_pin.clone(),
                    adc_divider_measured: split_config.central.adc_divider_measured,
                    adc_divider_total: split_config.central.adc_divider_total,
                    ..Default::default()
                })
            } else {
                // Fall back to [ble] section for backward compatibility
                ble_config
            };

            expand_adc_device(
                split_config
                    .central
                    .input_device
                    .clone()
                    .unwrap_or(InputDeviceConfig::default())
                    .joystick
                    .unwrap_or(Vec::new()),
                central_ble_config,
                chip.series.clone(),
            )
        }
    };

    for initializer in adc_initializers {
        initialization.extend(initializer.initializer);
        let device_name = initializer.var_name;
        devices.push(quote! { #device_name });
    }

    for initializer in adc_processors {
        initialization.extend(initializer.initializer);
        let processor_name = initializer.var_name;
        processors.push(quote! { #processor_name });
    }

    // generate encoder configuration
    let (device_initializer, processor_initializer) = match board {
        BoardConfig::UniBody(UniBodyConfig { input_device, .. }) => {
            expand_encoder_device(0, input_device.clone().encoder.unwrap_or(Vec::new()), chip)
        }
        BoardConfig::Split(split_config) => expand_encoder_device(
            0,
            split_config
                .central
                .input_device
                .clone()
                .unwrap_or(InputDeviceConfig::default())
                .encoder
                .unwrap_or(Vec::new()),
            chip,
        ),
    };
    for initializer in device_initializer {
        initialization.extend(initializer.initializer);
        let device_name = initializer.var_name;
        devices.push(quote! { #device_name });
    }

    for initializer in processor_initializer {
        initialization.extend(initializer.initializer);
        let processor_name = initializer.var_name;
        processors.push(quote! { #processor_name });
    }

    // generate PMW3610 configuration
    let (pmw3610_device_initializers, pmw3610_processor_initializers) = match board {
        BoardConfig::UniBody(UniBodyConfig { input_device, .. }) => {
            expand_pmw3610_device(input_device.clone().pmw3610.unwrap_or(Vec::new()), chip)
        }
        BoardConfig::Split(split_config) => expand_pmw3610_device(
            split_config
                .central
                .input_device
                .clone()
                .unwrap_or(InputDeviceConfig::default())
                .pmw3610
                .unwrap_or(Vec::new()),
            chip,
        ),
    };

    for initializer in pmw3610_device_initializers {
        initialization.extend(initializer.initializer);
        let device_name = initializer.var_name;
        devices.push(quote! { #device_name });
    }

    for initializer in pmw3610_processor_initializers {
        initialization.extend(initializer.initializer);
        let processor_name = initializer.var_name;
        processors.push(quote! { #processor_name });
    }

    // For split keyboards, also generate processors for PMW3610 devices on peripherals
    // The devices run on peripherals, but processors need to run on central to handle the events
    if let BoardConfig::Split(split_config) = board {
        for peripheral in &split_config.peripheral {
            let peripheral_pmw3610_config = peripheral
                .input_device
                .clone()
                .unwrap_or(InputDeviceConfig::default())
                .pmw3610
                .unwrap_or(Vec::new());

            // Only generate processors (not devices) for peripheral PMW3610
            let (_, peripheral_pmw3610_processors) =
                expand_pmw3610_device(peripheral_pmw3610_config, chip);

            for initializer in peripheral_pmw3610_processors {
                initialization.extend(initializer.initializer);
                let processor_name = initializer.var_name;
                processors.push(quote! { #processor_name });
            }
        }
    }

    // generate PMW33xx configuration
    let (pmw33xx_device_initializers, pmw33xx_processor_initializers) = match board {
        BoardConfig::UniBody(UniBodyConfig { input_device, .. }) => {
            expand_pmw33xx_device(input_device.clone().pmw33xx.unwrap_or(Vec::new()), chip)
        }
        BoardConfig::Split(split_config) => expand_pmw33xx_device(
            split_config
                .central
                .input_device
                .clone()
                .unwrap_or(InputDeviceConfig::default())
                .pmw33xx
                .unwrap_or(Vec::new()),
            chip,
        ),
    };

    for initializer in pmw33xx_device_initializers {
        initialization.extend(initializer.initializer);
        let device_name = initializer.var_name;
        devices.push(quote! { #device_name });
    }

    for initializer in pmw33xx_processor_initializers {
        initialization.extend(initializer.initializer);
        let processor_name = initializer.var_name;
        processors.push(quote! { #processor_name });
    }

    // For split keyboards, also generate processors for PMW33xx devices on peripherals
    // The devices run on peripherals, but processors need to run on central to handle the events
    if let BoardConfig::Split(split_config) = board {
        for peripheral in &split_config.peripheral {
            let peripheral_pmw33xx_config = peripheral
                .input_device
                .clone()
                .unwrap_or(InputDeviceConfig::default())
                .pmw33xx
                .unwrap_or(Vec::new());

            // Only generate processors (not devices) for peripheral PMW33xx
            let (_, peripheral_pmw33xx_processors) =
                expand_pmw33xx_device(peripheral_pmw33xx_config, chip);

            for initializer in peripheral_pmw33xx_processors {
                initialization.extend(initializer.initializer);
                let processor_name = initializer.var_name;
                processors.push(quote! { #processor_name });
            }
        }
    }

    // Generate IQS5xx configuration. The mouse-button-routing destination
    // (if set) is read from the central's `[input_device]` block and
    // resolved against every iqs5xx the firmware will instantiate —
    // central + every peripheral — by name. Slot ids are assigned in
    // emission order: central iqs5xxs first, then each peripheral's in
    // turn.
    let (central_iqs5xx_configs, routing_target_name): (Vec<_>, Option<String>) = match board {
        BoardConfig::UniBody(UniBodyConfig { input_device, .. }) => {
            let configs = input_device.clone().iqs5xx.unwrap_or_default();
            let routing = input_device
                .mouse_button_routing
                .as_ref()
                .and_then(|r| r.trackpad.clone());
            (configs, routing)
        }
        BoardConfig::Split(split_config) => {
            let central_input = split_config
                .central
                .input_device
                .clone()
                .unwrap_or(InputDeviceConfig::default());
            let routing = central_input.mouse_button_routing.as_ref().and_then(|r| r.trackpad.clone());
            (central_input.iqs5xx.unwrap_or_default(), routing)
        }
    };

    // Build the (name, slot id) pairing for every iqs5xx the firmware
    // will instantiate. Slot ids are assigned in emission order.
    let mut iqs5xx_slots: Vec<(String, u8)> = Vec::new();
    for sensor in &central_iqs5xx_configs {
        let id = iqs5xx_slots.len() as u8;
        iqs5xx_slots.push((sensor.name.clone(), id));
    }
    if let BoardConfig::Split(split_config) = board {
        for peripheral in &split_config.peripheral {
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
    }

    let (iqs5xx_device_initializers, iqs5xx_processor_initializers) =
        expand_iqs5xx_device(central_iqs5xx_configs, chip);

    for initializer in iqs5xx_device_initializers {
        initialization.extend(initializer.initializer);
        let device_name = initializer.var_name;
        devices.push(quote! { #device_name });
    }

    for initializer in iqs5xx_processor_initializers {
        initialization.extend(initializer.initializer);
        let processor_name = initializer.var_name;
        processors.push(quote! { #processor_name });
    }

    // For split keyboards, also generate processors for IQS5xx devices on peripherals
    // The devices run on peripherals, but processors need to run on central to handle the events
    if let BoardConfig::Split(split_config) = board {
        for peripheral in &split_config.peripheral {
            let peripheral_iqs5xx_config = peripheral
                .input_device
                .clone()
                .unwrap_or(InputDeviceConfig::default())
                .iqs5xx
                .unwrap_or(Vec::new());

            // Only generate processors (not devices) for peripheral IQS5xx.
            let (_, peripheral_iqs5xx_processors) = expand_iqs5xx_device(peripheral_iqs5xx_config, chip);

            for initializer in peripheral_iqs5xx_processors {
                initialization.extend(initializer.initializer);
                let processor_name = initializer.var_name;
                processors.push(quote! { #processor_name });
            }
        }
    }

    // Mouse-button routing setup. Emits a one-shot
    // `set_mouse_button_destination(Some(id))` (or compile_error! on a
    // misnamed target) before USB enumeration. Resolved against the
    // combined slot list above so a peripheral-side iqs5xx is reachable
    // by name from the central's TOML.
    initialization.extend(expand_mouse_button_routing(
        routing_target_name.as_deref(),
        &iqs5xx_slots,
    ));

    (initialization, devices, processors)
}
