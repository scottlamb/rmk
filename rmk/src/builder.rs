//! `Rmk` keyboard builder.
//!
//! Construct from your [`RmkConfig`], wire up runtime peripherals and hooks
//! via `with_*` methods, then `.run()`:
//!
//! ```ignore
//! Rmk::new(rmk_config)
//!     .with_usb(driver)
//!     .with_keymap(keymap)
//!     .with_storage(storage)
//!     .with_usb_extend(|builder| picotool_reset::register(builder, …))
//!     .run()
//!     .await
//! ```
//!
//! # Status: in-progress slot expansion
//!
//! Slots present today: USB driver, USB-extension hook, keymap, storage. The
//! plan is for *every* runtime peripheral and behavior hook to live on the
//! builder eventually — matrix, split-central UART, encoder, trackpad
//! processor, display — at which point `.run()` joins all of those internal
//! futures and the user's `main` only needs to join its own bespoke futures
//! (custom GPIO event sources, panic loggers, etc.) alongside `rmk.run()`.
//!
//! Adding a new slot is purely additive: a generic param on `Rmk` (default
//! is a `No*` marker), a wrapper holding the populated state, and one new
//! `with_*` method that swaps the generic.

#[cfg(feature = "_ble")]
use bt_hci::{
    cmd::le::{LeReadLocalSupportedFeatures, LeSetPhy},
    controller::{ControllerCmdAsync, ControllerCmdSync},
};
#[cfg(not(feature = "_no_usb"))]
use embassy_usb::Builder;
#[cfg(not(feature = "_no_usb"))]
use embassy_usb::driver::Driver;
#[cfg(feature = "storage")]
use embedded_storage_async::nor_flash::NorFlash as AsyncNorFlash;
#[cfg(feature = "_ble")]
use trouble_host::prelude::{Controller, DefaultPacketPool, Stack};

use crate::config::RmkConfig;
#[cfg(feature = "host")]
use crate::keymap::KeyMap;
#[cfg(feature = "storage")]
use crate::storage::Storage;

/// Marker filling any builder slot with a no-op / "not configured."
pub struct NoExtension;

/// USB-driver slot, populated by [`Rmk::with_usb`]. Holds the user's driver
/// until `.run()` consumes it.
#[cfg(not(feature = "_no_usb"))]
pub struct Usb<D>(pub(crate) D);

/// "USB driver slot is empty" marker. Default for the slot; the only legal
/// state in `_no_usb` builds.
pub struct NoUsb;

/// Keymap slot, populated by [`Rmk::with_keymap`]. Holds a `'static`
/// reference to a caller-allocated `KeyMap<'static>` (typically the
/// `&'static KeyMap<'static>` returned by a `StaticCell::init` after
/// `initialize_keymap_and_storage`).
///
/// Why a reference instead of by-value ownership? `KeyMap<'a>` is invariant
/// in `'a` (it contains `&'a mut` slices), so a by-value `KeyMap<'static>`
/// borrowed as `&'_ KeyMap<'static>` doesn't satisfy internal helpers
/// signed `&'a KeyMap<'a>` (outer = inner lifetime). The cleaner fix would
/// be to plumb two free lifetimes through every helper — but for the
/// builder draft, paying for one extra `StaticCell` to land at a single
/// `'static` lifetime keeps the diff local.
#[cfg(feature = "host")]
pub struct Keymap(pub(crate) &'static KeyMap<'static>);

/// "Keymap slot is empty" marker.
pub struct NoKeymap;

/// Storage slot, populated by [`Rmk::with_storage`]. Owns the storage by
/// value (`Storage<F, …>` has no lifetime parameter).
#[cfg(feature = "storage")]
pub struct StorageHeld<
    F: AsyncNorFlash,
    const ROW: usize,
    const COL: usize,
    const NUM_LAYER: usize,
    const NUM_ENCODER: usize,
>(pub(crate) Storage<F, ROW, COL, NUM_LAYER, NUM_ENCODER>);

/// "Storage slot is empty" marker.
pub struct NoStorage;

/// Matrix slot, populated by [`Rmk::with_matrix`]. Holds any
/// [`crate::core_traits::Runnable`] (typically a `Matrix<…>`) by value;
/// `.run()` consumes it and joins its run-loop with the rest.
pub struct MatrixHeld<M>(pub(crate) M);

/// "Matrix slot is empty" marker.
pub struct NoMatrix;

/// Split-central UART link slot, populated by
/// [`Rmk::with_split_central_uart`]. Holds the per-peripheral metadata
/// (id + matrix dimensions + offset into the global keymap) plus the UART
/// receiver. `.run()` invokes [`crate::split::central::run_peripheral_manager`]
/// internally so the user doesn't need to spell out its turbofish.
#[cfg(all(feature = "split", not(feature = "_ble")))]
pub struct SplitCentralUart<R, const ROW: usize, const COL: usize, const ROW_OFFSET: usize, const COL_OFFSET: usize> {
    pub(crate) id: usize,
    pub(crate) receiver: R,
}

/// "Split-central UART slot is empty" marker.
pub struct NoSplitCentralUart;

/// Slot trait for `SCU`. Lets `.run()` join a peripheral-manager future
/// regardless of whether the slot is populated — `NoSplitCentralUart`
/// resolves to a never-completing future, and a populated
/// `SplitCentralUart` resolves to
/// [`crate::split::central::run_peripheral_manager`].
#[cfg(not(feature = "_ble"))]
pub trait SplitCentralUartSlot {
    /// Drive the peripheral-manager future to completion. Never returns
    /// in practice (both arms run forever).
    async fn run_peripheral_manager(self);
}

#[cfg(not(feature = "_ble"))]
impl SplitCentralUartSlot for NoSplitCentralUart {
    async fn run_peripheral_manager(self) {
        core::future::pending::<()>().await
    }
}

#[cfg(all(feature = "split", not(feature = "_ble")))]
impl<R, const ROW: usize, const COL: usize, const ROW_OFFSET: usize, const COL_OFFSET: usize> SplitCentralUartSlot
    for SplitCentralUart<R, ROW, COL, ROW_OFFSET, COL_OFFSET>
where
    R: embedded_io_async::Read + embedded_io_async::Write,
{
    async fn run_peripheral_manager(self) {
        crate::split::central::run_peripheral_manager::<ROW, COL, ROW_OFFSET, COL_OFFSET, _>(self.id, self.receiver)
            .await
    }
}

/// Empty input-device chain. Implements [`crate::core_traits::Runnable`]
/// as a future that never resolves, so it composes cleanly into the
/// builder's `.run()` join even when the user has registered no input
/// devices beyond the matrix.
pub struct NoInputDevices;

impl crate::core_traits::Runnable for NoInputDevices {
    async fn run(&mut self) -> ! {
        // Wait forever. The other futures in `.run()`'s join drive their
        // own work; this slot just needs to exist.
        core::future::pending::<()>().await;
        unreachable!()
    }
}

/// One node of the input-device chain — head device + tail (which is
/// either another `InputDeviceCons` or [`NoInputDevices`]).
///
/// Each [`Rmk::with_input_device`] call wraps the new device on the head
/// of the existing chain, so the chain grows like a cons-list. The shape
/// at the call site is just `.with_input_device(a).with_input_device(b)`;
/// the type spelling stays internal.
pub struct InputDeviceCons<H, T> {
    head: H,
    tail: T,
}

impl<H: crate::core_traits::Runnable, T: crate::core_traits::Runnable> crate::core_traits::Runnable
    for InputDeviceCons<H, T>
{
    async fn run(&mut self) -> ! {
        let Self { head, tail } = self;
        embassy_futures::join::join(head.run(), tail.run()).await;
        unreachable!()
    }
}

/// Slot trait for the USB-extension hook. Runs on the central's
/// `embassy_usb::Builder` after RMK has registered its own HID interfaces
/// and the optional `usb_log` CDC, but before `Builder::build()`. Use it to
/// add vendor reset interfaces, debug CDCs, custom HIDs — anything that
/// needs to ride alongside RMK's own USB stack on the central. (Split
/// peripherals build their own `Builder` by hand and should call their
/// registration helpers directly.)
///
/// Blanket impl covers `FnOnce(&mut Builder<'d, D>)` closures; the
/// [`NoExtension`] marker fills the slot with a no-op when nothing was
/// supplied.
#[cfg(not(feature = "_no_usb"))]
pub trait UsbExtensionFn<'d, D: Driver<'d>> {
    fn extend(self, builder: &mut Builder<'d, D>);
}

#[cfg(not(feature = "_no_usb"))]
impl<'d, D: Driver<'d>, F: FnOnce(&mut Builder<'d, D>)> UsbExtensionFn<'d, D> for F {
    fn extend(self, builder: &mut Builder<'d, D>) {
        self(builder)
    }
}

#[cfg(not(feature = "_no_usb"))]
impl<'d, D: Driver<'d>> UsbExtensionFn<'d, D> for NoExtension {
    fn extend(self, _builder: &mut Builder<'d, D>) {}
}

/// Keyboard runtime — builder + entry point.
///
/// Field privacy is intentional: the layout is internal so future versions
/// can move things around (e.g. add new slots, change how hooks are stored)
/// without breaking out-of-tree code that builds via `Rmk::new` + `with_*`.
///
/// Generic type parameters track per-slot state. Defaults are no-op markers
/// so the all-defaults case writes as `Rmk::new(config)`.
pub struct Rmk<
    U = NoUsb,
    S = NoStorage,
    K = NoKeymap,
    M = NoMatrix,
    SCU = NoSplitCentralUart,
    ID = NoInputDevices,
    X = NoExtension,
> {
    rmk_config: RmkConfig<'static>,
    #[cfg_attr(feature = "_no_usb", allow(dead_code))]
    usb: U,
    storage: S,
    keymap: K,
    matrix: M,
    split_central_uart: SCU,
    input_devices: ID,
    usb_extend: X,
}

impl Rmk {
    /// Start a new builder from a populated [`RmkConfig`].
    pub fn new(rmk_config: RmkConfig<'static>) -> Self {
        Self {
            rmk_config,
            usb: NoUsb,
            storage: NoStorage,
            keymap: NoKeymap,
            matrix: NoMatrix,
            split_central_uart: NoSplitCentralUart,
            input_devices: NoInputDevices,
            usb_extend: NoExtension,
        }
    }
}

impl<U, S, K, M, SCU, ID, X> Rmk<U, S, K, M, SCU, ID, X> {
    /// Set the USB driver. Required when the `_no_usb` feature is *not*
    /// enabled — `.run()` is only available on `Rmk<Usb<D>, …>` for
    /// non-`_no_usb` builds.
    #[cfg(not(feature = "_no_usb"))]
    pub fn with_usb<D: Driver<'static>>(self, driver: D) -> Rmk<Usb<D>, S, K, M, SCU, ID, X> {
        Rmk {
            rmk_config: self.rmk_config,
            usb: Usb(driver),
            storage: self.storage,
            keymap: self.keymap,
            matrix: self.matrix,
            split_central_uart: self.split_central_uart,
            input_devices: self.input_devices,
            usb_extend: self.usb_extend,
        }
    }

    /// Set the keymap. Required: `.run()` is only available once a keymap
    /// has been provided. Takes a `'static` reference (typically the
    /// `&'static KeyMap<'static>` from `StaticCell::init` on the result of
    /// `initialize_keymap_and_storage`).
    #[cfg(feature = "host")]
    pub fn with_keymap(self, keymap: &'static KeyMap<'static>) -> Rmk<U, S, Keymap, M, SCU, ID, X> {
        Rmk {
            rmk_config: self.rmk_config,
            usb: self.usb,
            storage: self.storage,
            keymap: Keymap(keymap),
            matrix: self.matrix,
            split_central_uart: self.split_central_uart,
            input_devices: self.input_devices,
            usb_extend: self.usb_extend,
        }
    }

    /// Set the storage. Required when the `storage` feature is enabled.
    /// Takes ownership.
    #[cfg(feature = "storage")]
    pub fn with_storage<
        F: AsyncNorFlash,
        const ROW: usize,
        const COL: usize,
        const NUM_LAYER: usize,
        const NUM_ENCODER: usize,
    >(
        self,
        storage: Storage<F, ROW, COL, NUM_LAYER, NUM_ENCODER>,
    ) -> Rmk<U, StorageHeld<F, ROW, COL, NUM_LAYER, NUM_ENCODER>, K, M, SCU, ID, X> {
        Rmk {
            rmk_config: self.rmk_config,
            usb: self.usb,
            storage: StorageHeld(storage),
            keymap: self.keymap,
            matrix: self.matrix,
            split_central_uart: self.split_central_uart,
            input_devices: self.input_devices,
            usb_extend: self.usb_extend,
        }
    }

    /// Set the matrix runner. Anything that implements
    /// [`crate::core_traits::Runnable`] works — typically a
    /// [`crate::matrix::Matrix<…>`]. `.run()` joins its run-loop with the
    /// USB / storage / keymap stack.
    pub fn with_matrix<NewM: crate::core_traits::Runnable>(
        self,
        matrix: NewM,
    ) -> Rmk<U, S, K, MatrixHeld<NewM>, SCU, ID, X> {
        Rmk {
            rmk_config: self.rmk_config,
            usb: self.usb,
            storage: self.storage,
            keymap: self.keymap,
            matrix: MatrixHeld(matrix),
            split_central_uart: self.split_central_uart,
            input_devices: self.input_devices,
            usb_extend: self.usb_extend,
        }
    }

    /// Set the split-central UART link. The const generics specify the
    /// peripheral's local matrix dimensions and where they sit in the
    /// global keymap. `.run()` invokes
    /// [`crate::split::central::run_peripheral_manager`] internally.
    #[cfg(all(feature = "split", not(feature = "_ble")))]
    pub fn with_split_central_uart<
        R,
        const ROW: usize,
        const COL: usize,
        const ROW_OFFSET: usize,
        const COL_OFFSET: usize,
    >(
        self,
        id: usize,
        receiver: R,
    ) -> Rmk<U, S, K, M, SplitCentralUart<R, ROW, COL, ROW_OFFSET, COL_OFFSET>, ID, X> {
        Rmk {
            rmk_config: self.rmk_config,
            usb: self.usb,
            storage: self.storage,
            keymap: self.keymap,
            matrix: self.matrix,
            split_central_uart: SplitCentralUart { id, receiver },
            input_devices: self.input_devices,
            usb_extend: self.usb_extend,
        }
    }

    /// Add an input device to the run-loop. Anything that implements
    /// [`crate::core_traits::Runnable`] (rotary encoder, trackpad
    /// processor, pointing processor, etc.) works. Each call prepends
    /// onto the existing chain — call as many times as needed.
    pub fn with_input_device<NewID: crate::core_traits::Runnable>(
        self,
        device: NewID,
    ) -> Rmk<U, S, K, M, SCU, InputDeviceCons<NewID, ID>, X> {
        Rmk {
            rmk_config: self.rmk_config,
            usb: self.usb,
            storage: self.storage,
            keymap: self.keymap,
            matrix: self.matrix,
            split_central_uart: self.split_central_uart,
            input_devices: InputDeviceCons {
                head: device,
                tail: self.input_devices,
            },
            usb_extend: self.usb_extend,
        }
    }

    /// Set the USB-extension callback. See [`UsbExtensionFn`].
    pub fn with_usb_extend<X2>(self, f: X2) -> Rmk<U, S, K, M, SCU, ID, X2> {
        Rmk {
            rmk_config: self.rmk_config,
            usb: self.usb,
            storage: self.storage,
            keymap: self.keymap,
            matrix: self.matrix,
            split_central_uart: self.split_central_uart,
            input_devices: self.input_devices,
            usb_extend: f,
        }
    }
}

// `.run()` for the USB-only path (no `_ble`). Works for both split and
// non-split builds — the `SCU` slot's trait impl handles peripheral
// management when populated and resolves to a never-completing future
// when empty.
#[cfg(all(
    not(feature = "_no_usb"),
    not(feature = "_ble"),
    feature = "host",
    feature = "storage",
))]
impl<D, F, M, SCU, ID, const STG_ROW: usize, const STG_COL: usize, const NUM_LAYER: usize, const NUM_ENCODER: usize, X>
    Rmk<Usb<D>, StorageHeld<F, STG_ROW, STG_COL, NUM_LAYER, NUM_ENCODER>, Keymap, MatrixHeld<M>, SCU, ID, X>
where
    D: Driver<'static>,
    F: AsyncNorFlash,
    M: crate::core_traits::Runnable,
    SCU: SplitCentralUartSlot,
    ID: crate::core_traits::Runnable,
    X: UsbExtensionFn<'static, D>,
{
    pub async fn run(self) -> ! {
        let Rmk {
            rmk_config,
            usb: Usb(usb_driver),
            storage: StorageHeld(mut storage),
            keymap: Keymap(keymap),
            matrix: MatrixHeld(mut matrix),
            split_central_uart,
            mut input_devices,
            usb_extend,
        } = self;

        let mut keyboard = crate::keyboard::Keyboard::new(keymap);
        // Vial request consumer. Without it, Vial's USB endpoint enqueues
        // requests into HOST_REQUEST_CHANNEL but nothing drains them —
        // Vial Desktop times out.
        //
        // `HostService::new` takes `&'a KeyboardContext<'a>`, and our
        // keymap is `&'static`, so the context borrow must also be
        // `'static`. Allocate the context in a function-local
        // `StaticCell` to give it a `'static` address.
        #[cfg(feature = "vial")]
        let host_ctx: &'static crate::host::KeyboardContext<'static> = {
            static HOST_CTX: static_cell::StaticCell<crate::host::KeyboardContext<'static>> =
                static_cell::StaticCell::new();
            HOST_CTX.init(crate::host::KeyboardContext::new(keymap))
        };
        #[cfg(feature = "vial")]
        let mut host_service = crate::host::HostService::new(host_ctx, &rmk_config);

        // Build the USB transport with the user's extension callback
        // running on the embassy-usb Builder after RMK's HID classes
        // but before `Builder::build()`.
        let mut usb_transport =
            crate::usb::UsbTransport::new_with_extend(usb_driver, rmk_config.device_config, |b| usb_extend.extend(b));

        use crate::core_traits::Runnable;
        let matrix_fut = matrix.run();
        let storage_fut = storage.run();
        let split_fut = split_central_uart.run_peripheral_manager();
        let usb_fut = usb_transport.run();
        let keyboard_fut = keyboard.run();
        let input_devices_fut = input_devices.run();
        #[cfg(feature = "vial")]
        let host_fut = host_service.run();
        #[cfg(not(feature = "vial"))]
        let host_fut = core::future::pending::<()>();

        embassy_futures::join::join5(
            embassy_futures::join::join3(usb_fut, host_fut, storage_fut),
            matrix_fut,
            split_fut,
            keyboard_fut,
            input_devices_fut,
        )
        .await;

        unreachable!("USB run loop returned");
    }
}

// `.run()` for the USB + BLE path. Currently restricted to the no-split
// case — BLE+split needs different `run_peripheral_manager` arguments
// (addr table + stack ref) which need a different slot shape. Tracked
// for follow-up.
#[cfg(all(not(feature = "_no_usb"), feature = "_ble", feature = "host", feature = "storage",))]
impl<D, F, M, ID, const STG_ROW: usize, const STG_COL: usize, const NUM_LAYER: usize, const NUM_ENCODER: usize, X>
    Rmk<
        Usb<D>,
        StorageHeld<F, STG_ROW, STG_COL, NUM_LAYER, NUM_ENCODER>,
        Keymap,
        MatrixHeld<M>,
        NoSplitCentralUart,
        ID,
        X,
    >
where
    D: Driver<'static>,
    F: AsyncNorFlash,
    M: crate::core_traits::Runnable,
    ID: crate::core_traits::Runnable,
    X: UsbExtensionFn<'static, D>,
{
    pub async fn run<
        'b,
        C: Controller + ControllerCmdAsync<LeSetPhy> + ControllerCmdSync<LeReadLocalSupportedFeatures>,
    >(
        self,
        stack: &'b Stack<'b, C, DefaultPacketPool>,
    ) -> ! {
        let Rmk {
            rmk_config,
            usb: Usb(usb_driver),
            storage: StorageHeld(mut storage),
            keymap: Keymap(keymap),
            matrix: MatrixHeld(mut matrix),
            split_central_uart: NoSplitCentralUart,
            mut input_devices,
            usb_extend,
        } = self;

        let mut keyboard = crate::keyboard::Keyboard::new(keymap);
        // See the equivalent block in `Rmk::run` for why the context
        // lives in a function-local `StaticCell`.
        #[cfg(feature = "vial")]
        let host_ctx: &'static crate::host::KeyboardContext<'static> = {
            static HOST_CTX: static_cell::StaticCell<crate::host::KeyboardContext<'static>> =
                static_cell::StaticCell::new();
            HOST_CTX.init(crate::host::KeyboardContext::new(keymap))
        };
        #[cfg(feature = "vial")]
        let mut host_service = crate::host::HostService::new(host_ctx, &rmk_config);

        let device_config = rmk_config.device_config;
        let mut usb_transport =
            crate::usb::UsbTransport::new_with_extend(usb_driver, device_config, |b| usb_extend.extend(b));
        let mut ble_transport = crate::ble::BleTransport::new(stack, rmk_config).await;

        use crate::core_traits::Runnable;
        let matrix_fut = matrix.run();
        let storage_fut = storage.run();
        let usb_fut = usb_transport.run();
        let ble_fut = ble_transport.run();
        let keyboard_fut = keyboard.run();
        let input_devices_fut = input_devices.run();
        #[cfg(feature = "vial")]
        let host_fut = host_service.run();
        #[cfg(not(feature = "vial"))]
        let host_fut = core::future::pending::<()>();

        embassy_futures::join::join5(
            embassy_futures::join::join3(usb_fut, ble_fut, storage_fut),
            embassy_futures::join::join(host_fut, matrix_fut),
            keyboard_fut,
            input_devices_fut,
            core::future::pending::<()>(),
        )
        .await;

        unreachable!("BLE run loop returned");
    }
}

// `.run()` for the `_no_usb` BLE-only path. Same restriction on split as
// the USB+BLE impl.
#[cfg(all(feature = "_no_usb", feature = "_ble", feature = "host", feature = "storage",))]
impl<F, M, ID, const STG_ROW: usize, const STG_COL: usize, const NUM_LAYER: usize, const NUM_ENCODER: usize, X>
    Rmk<
        NoUsb,
        StorageHeld<F, STG_ROW, STG_COL, NUM_LAYER, NUM_ENCODER>,
        Keymap,
        MatrixHeld<M>,
        NoSplitCentralUart,
        ID,
        X,
    >
where
    F: AsyncNorFlash,
    M: crate::core_traits::Runnable,
    ID: crate::core_traits::Runnable,
{
    pub async fn run<
        'b,
        C: Controller + ControllerCmdAsync<LeSetPhy> + ControllerCmdSync<LeReadLocalSupportedFeatures>,
    >(
        self,
        stack: &'b Stack<'b, C, DefaultPacketPool>,
    ) -> ! {
        let Rmk {
            rmk_config,
            usb: NoUsb,
            storage: StorageHeld(mut storage),
            keymap: Keymap(keymap),
            matrix: MatrixHeld(mut matrix),
            split_central_uart: NoSplitCentralUart,
            mut input_devices,
            usb_extend,
        } = self;

        let mut keyboard = crate::keyboard::Keyboard::new(keymap);
        // See the equivalent block in `Rmk::run` for why the context
        // lives in a function-local `StaticCell`.
        #[cfg(feature = "vial")]
        let host_ctx: &'static crate::host::KeyboardContext<'static> = {
            static HOST_CTX: static_cell::StaticCell<crate::host::KeyboardContext<'static>> =
                static_cell::StaticCell::new();
            HOST_CTX.init(crate::host::KeyboardContext::new(keymap))
        };
        #[cfg(feature = "vial")]
        let mut host_service = crate::host::HostService::new(host_ctx, &rmk_config);

        // `usb_extend` is unused on the no-USB BLE-only path; bind to
        // suppress the unused warning without changing the slot type.
        let _ = usb_extend;
        let mut ble_transport = crate::ble::BleTransport::new(stack, rmk_config).await;

        use crate::core_traits::Runnable;
        let matrix_fut = matrix.run();
        let storage_fut = storage.run();
        let ble_fut = ble_transport.run();
        let keyboard_fut = keyboard.run();
        let input_devices_fut = input_devices.run();
        #[cfg(feature = "vial")]
        let host_fut = host_service.run();
        #[cfg(not(feature = "vial"))]
        let host_fut = core::future::pending::<()>();

        embassy_futures::join::join4(
            embassy_futures::join::join3(ble_fut, host_fut, storage_fut),
            matrix_fut,
            keyboard_fut,
            input_devices_fut,
        )
        .await;

        unreachable!("BLE run loop returned");
    }
}
