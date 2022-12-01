use super::ActorBase;
use crate::sensor::SensorData;
use autocxx::c_void;
use carla_sys::carla_rust::{
    client::{FfiActor, FfiSensor},
    sensor::FfiSensorData,
};
use cxx::SharedPtr;
use std::mem;

type Callback = dyn FnMut(SharedPtr<FfiSensorData>) + Send + 'static;

pub struct Sensor {
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
                let fn_: Box<Callback> = Box::new(fn_);
                let fn_: *mut Callback = Box::into_raw(fn_);
                fn_ as *mut c_void
            };

            let caller_ptr = caller as *mut c_void;
            let deleter_ptr = deleter as *mut c_void;

            self.inner.Listen(caller_ptr, fn_ptr, deleter_ptr);
        }
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiSensor>) -> Self {
        Self { inner: ptr }
    }
}

impl ActorBase for Sensor {
    fn cxx_actor(&self) -> SharedPtr<FfiActor> {
        self.inner.to_actor()
    }
}

unsafe extern "C" fn caller(fn_: *mut c_void, arg: *mut c_void) {
    let fn_: *mut Callback = mem::transmute_copy(&fn_);
    let fn_: &mut Callback = &mut *fn_;
    let arg: SharedPtr<FfiSensorData> = mem::transmute_copy(&arg);
    (fn_)(arg);
}

unsafe extern "C" fn deleter(ptr: *mut c_void) {
    let ptr: *mut Callback = mem::transmute_copy(&ptr);
    mem::drop(Box::from_raw(ptr));
}
