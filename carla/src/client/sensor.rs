use super::{Actor, ActorBase};
use crate::sensor::SensorData;
use autocxx::c_void;
use carla_sys::carla_rust::{
    client::{FfiActor, FfiSensor},
    sensor::FfiSensorData,
};
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;
use std::mem;

type Callback = dyn FnMut(SharedPtr<FfiSensorData>) + Send + 'static;

/// Represents a sensor in the simulation, corresponding to
/// `carla.Sensor` in Python API.
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Sensor {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiSensor>,
}

impl Sensor {
    pub fn stop(&self) {
        self.inner.Stop();
    }

    pub fn is_listening(&self) -> bool {
        self.inner.IsListening()
    }

    pub fn listen<F>(&self, mut callback: F)
    where
        F: FnMut(SensorData) + Send + 'static,
    {
        unsafe {
            let fn_ptr = {
                let fn_ = move |ptr: SharedPtr<FfiSensorData>| {
                    let data = SensorData::from_cxx(ptr);
                    (callback)(data);
                };
                // Double boxing pattern for FFI:
                // 1. Box<Callback> creates a trait object (fat pointer: data ptr + vtable ptr)
                // 2. Box<Box<Callback>> boxes the fat pointer itself, giving us a thin pointer
                //    to a heap location that contains the fat pointer
                // 3. Box::into_raw converts to *mut Box<Callback>, which is a thin raw pointer
                //    suitable for passing through C FFI (which cannot handle fat pointers)
                let fn_: Box<Callback> = Box::new(fn_);
                let fn_ = Box::new(fn_);
                let fn_: *mut Box<Callback> = Box::into_raw(fn_);
                fn_ as *mut c_void
            };

            let caller_ptr = caller as *mut c_void;
            let deleter_ptr = deleter as *mut c_void;

            self.inner.Listen(caller_ptr, fn_ptr, deleter_ptr);
        }
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiSensor>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl ActorBase for Sensor {
    fn cxx_actor(&self) -> SharedPtr<FfiActor> {
        self.inner.to_actor()
    }
}

impl TryFrom<Actor> for Sensor {
    type Error = Actor;

    fn try_from(value: Actor) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_sensor();
        Self::from_cxx(ptr).ok_or(value)
    }
}

unsafe extern "C" fn caller(fn_: *mut c_void, arg: *mut SharedPtr<FfiSensorData>) {
    // SAFETY: These pointers are provided by the C++ CARLA library through the Listen callback.
    // We validate they are non-null before dereferencing to prevent UB.
    if fn_.is_null() || arg.is_null() {
        eprintln!(
            "ERROR: Null pointer in sensor callback - fn_: {:p}, arg: {:p}",
            fn_, arg
        );
        return;
    }

    // Cast back to *mut Box<Callback> (the thin pointer we passed to C++)
    // We dereference it to get &Box<Callback>, which auto-derefs to &Callback
    let fn_ = fn_ as *mut Box<Callback>;
    let arg = (*arg).clone();
    (*fn_)(arg);
}

unsafe extern "C" fn deleter(fn_: *mut c_void) {
    // SAFETY: This pointer was created by Box::into_raw in the listen() method.
    // We validate it's non-null before attempting to reconstruct and drop the Box.
    if fn_.is_null() {
        eprintln!("ERROR: Null pointer in sensor deleter");
        return;
    }

    // Reconstruct the Box<Box<Callback>> to properly drop both layers:
    // 1. Cast back to *mut Box<Callback> (thin pointer)
    // 2. Box::from_raw reconstructs Box<Box<Callback>>
    // 3. Drop cleans up both the outer box and the inner trait object
    let fn_ = fn_ as *mut Box<Callback>;
    let fn_: Box<Box<Callback>> = Box::from_raw(fn_);
    mem::drop(fn_);
}

assert_impl_all!(Sensor: Send, Sync);
