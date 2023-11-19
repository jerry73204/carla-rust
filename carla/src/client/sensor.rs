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
                let fn_: Box<Callback> = Box::new(fn_); // Create a trait object ("fat" pointer)
                let fn_ = Box::new(fn_); // Create a "thin" pointer. The address is aliased
                let fn_: *mut Callback = Box::into_raw(fn_); // Convert to raw pointer
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
    let fn_ = fn_ as *mut Box<Callback>;
    let arg = (*arg).clone();
    (*fn_)(arg);
}

unsafe extern "C" fn deleter(fn_: *mut c_void) {
    let fn_ = fn_ as *mut Box<Callback>;
    let fn_: Box<Box<Callback>> = Box::from_raw(fn_);
    mem::drop(fn_);
}

assert_impl_all!(Sensor: Send, Sync);
