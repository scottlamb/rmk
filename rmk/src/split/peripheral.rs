#[cfg(feature = "_ble")]
use bt_hci::{cmd::le::LeSetPhy, controller::ControllerCmdAsync};
use embassy_futures::select::{Either, select};
#[cfg(not(feature = "_ble"))]
use embedded_io_async::{Read, Write};
use futures::FutureExt;
#[cfg(all(feature = "_ble", feature = "storage"))]
use {super::ble::PeerAddress, crate::channel::FLASH_CHANNEL};
#[cfg(feature = "_ble")]
use {
    crate::event::{BatteryStatusEvent, ChargingStateEvent, EventSubscriber},
    rmk_types::battery::BatteryStatus,
    trouble_host::prelude::*,
};

use super::SplitMessage;
use super::driver::{SplitReader, SplitWriter};
use crate::event::{
    KeyboardEvent, LayerChangeEvent, LedIndicatorEvent, PointingEvent, SubscribableEvent, TrackpadEvent, publish_event,
};
#[cfg(feature = "display")]
use crate::event::{ModifierEvent, SleepStateEvent, WpmUpdateEvent};
#[cfg(not(feature = "_ble"))]
use crate::split::serial::SerialSplitDriver;
use crate::state::update_status;

/// Run the split peripheral service.
///
/// # Arguments
///
/// * `id` - (optional) The id of the peripheral
/// * `stack` - (optional) The TrouBLE stack
/// * `serial` - (optional) serial port used to send peripheral split message. This argument is enabled only for serial split now
/// * `storage` - (optional) The storage to save the central address
#[allow(clippy::extra_unused_lifetimes)]
pub async fn run_rmk_split_peripheral<
    'b,
    's,
    #[cfg(feature = "_ble")] C: Controller + ControllerCmdAsync<LeSetPhy>,
    #[cfg(not(feature = "_ble"))] S: Write + Read,
>(
    #[cfg(feature = "_ble")] id: usize,
    #[cfg(feature = "_ble")] stack: &'b Stack<'s, C, DefaultPacketPool>,
    #[cfg(not(feature = "_ble"))] serial: S,
) where
    's: 'b,
{
    #[cfg(not(feature = "_ble"))]
    {
        let mut peripheral = SplitPeripheral::new(SerialSplitDriver::new(serial));
        loop {
            peripheral.run().await;
        }
    }

    #[cfg(feature = "_ble")]
    crate::split::ble::peripheral::initialize_nrf_ble_split_peripheral_and_run(id, stack).await;
}

/// The split peripheral instance.
pub(crate) struct SplitPeripheral<S: SplitWriter + SplitReader> {
    split_driver: S,
}

/// Build the next peripheral→central trackpad split message. The
/// `Trackpad` variant of `SplitMessage` only exists on `not(_ble)`
/// (see `split/mod.rs` for why), so on `_ble` we never resolve a
/// SplitMessage and the `select_biased!` arm is effectively dead. The
/// peripheral still runs the trackpad subscriber unconditionally so
/// the rest of the pipeline (compile-time wiring, local consumers in a
/// future commit) doesn't need an extra cfg gate.
#[cfg(not(feature = "_ble"))]
async fn next_trackpad_split_msg<E: crate::event::EventSubscriber<Event = TrackpadEvent>>(
    sub: &mut E,
) -> SplitMessage {
    SplitMessage::Trackpad(sub.next_event().await)
}
#[cfg(feature = "_ble")]
async fn next_trackpad_split_msg<E: crate::event::EventSubscriber<Event = TrackpadEvent>>(
    _sub: &mut E,
) -> SplitMessage {
    core::future::pending::<SplitMessage>().await
}

/// Liveness ping for the serial split. Returns a `Heartbeat` message
/// once per [`HEARTBEAT_INTERVAL_MS`](crate::split::driver::HEARTBEAT_INTERVAL_MS).
/// Re-created on every run-loop iteration, so the next firing schedules
/// from the moment the previous one was sent.
///
/// On the BLE split the connection state is already exposed by
/// [`PeripheralConnectedEvent`](crate::event::PeripheralConnectedEvent)
/// at the GATT layer, and `SplitMessage::Heartbeat` doesn't exist on
/// that build, so this returns a never-resolving future.
#[cfg(not(feature = "_ble"))]
async fn next_heartbeat_split_msg() -> SplitMessage {
    embassy_time::Timer::after_millis(crate::split::driver::HEARTBEAT_INTERVAL_MS).await;
    SplitMessage::Heartbeat
}
#[cfg(feature = "_ble")]
async fn next_heartbeat_split_msg() -> SplitMessage {
    core::future::pending::<SplitMessage>().await
}

impl<S: SplitWriter + SplitReader> SplitPeripheral<S> {
    pub(crate) fn new(split_driver: S) -> Self {
        Self { split_driver }
    }

    /// Run the peripheral keyboard service.
    ///
    /// The peripheral uses the general matrix, does scanning and send the key events through `SplitWriter`.
    /// If also receives split messages from the central through `SplitReader`.
    pub(crate) async fn run(&mut self) {
        let mut key_sub = KeyboardEvent::subscriber();
        #[cfg(feature = "_ble")]
        let mut charging_state_sub = ChargingStateEvent::subscriber();
        let mut pointing_sub = PointingEvent::subscriber();
        let mut trackpad_sub = TrackpadEvent::subscriber();
        #[cfg(feature = "_ble")]
        let mut battery_sub = BatteryStatusEvent::subscriber();

        loop {
            let read_message_to_send = async {
                crate::select_biased_with_feature! {
                    e = key_sub.next_message_pure().fuse() => SplitMessage::Key(e),
                    with_feature("_ble"): e = charging_state_sub.next_message_pure().fuse() => {
                        SplitMessage::BatteryStatus(BatteryStatus::Available {
                            charge_state: e.charging.into(),
                            level: None,
                        }.into())
                    },
                    e = pointing_sub.next_message_pure().fuse() => SplitMessage::Pointing(e),
                    msg = next_trackpad_split_msg(&mut trackpad_sub).fuse() => msg,
                    msg = next_heartbeat_split_msg().fuse() => msg,
                    with_feature("_ble"): e = battery_sub.next_event().fuse() => SplitMessage::BatteryStatus(e),
                }
            };

            match select(self.split_driver.read(), read_message_to_send).await {
                Either::First(m) => match m {
                    // Process split messages from the central
                    Ok(split_message) => match split_message {
                        SplitMessage::ConnectionStatus(status) => {
                            trace!("Received central connection status: {:?}", status);
                            update_status(|c| *c = status);
                        }
                        #[cfg(all(feature = "_ble", feature = "storage"))]
                        SplitMessage::ClearPeer => {
                            // Clear the peer address
                            FLASH_CHANNEL
                                .send(crate::storage::FlashOperationMessage::PeerAddress(PeerAddress::new(
                                    0, false, [0; 6],
                                )))
                                .await;
                        }
                        SplitMessage::KeyboardIndicator(indicator) => {
                            // Publish KeyboardIndicator event
                            publish_event(LedIndicatorEvent::new(
                                rmk_types::led_indicator::LedIndicator::from_bits(indicator),
                            ));
                        }
                        SplitMessage::Layer(layer) => {
                            // Publish Layer event
                            publish_event(LayerChangeEvent::new(layer));
                        }
                        #[cfg(feature = "display")]
                        SplitMessage::Wpm(wpm) => {
                            publish_event(WpmUpdateEvent::new(wpm));
                        }
                        #[cfg(feature = "display")]
                        SplitMessage::Modifier(bits) => {
                            publish_event(ModifierEvent {
                                modifier: rmk_types::modifier::ModifierCombination::from_bits(bits),
                            });
                        }
                        #[cfg(feature = "display")]
                        SplitMessage::SleepState(sleeping) => {
                            publish_event(SleepStateEvent::new(sleeping));
                        }
                        #[cfg(all(feature = "rgb_lighting", not(feature = "_ble")))]
                        SplitMessage::LightingFrame(bytes) => {
                            // Drop on full — the receiver task is running
                            // and a freshly-arriving frame supersedes any
                            // stale one. With capacity 1, this only loses
                            // a frame if RX is starved (extremely unlikely
                            // in practice).
                            let _ = crate::light::rgb::LIGHTING_FRAME_RX.try_send(bytes);
                        }
                        _ => (),
                    },
                    Err(e) => {
                        error!("Split message read error: {:?}", e);
                        if let crate::split::driver::SplitDriverError::Disconnected = e {
                            break;
                        }
                    }
                },
                Either::Second(e) => {
                    debug!("Writing split message {:?} to central", e);
                    self.split_driver.write(&e).await.ok();
                }
            }
        }
    }
}
