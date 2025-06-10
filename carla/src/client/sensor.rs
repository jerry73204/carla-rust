use super::{Actor, ActorBase};
use crate::sensor::SensorData;
use crate::utils::check_carla_error;
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::{ffi::c_void, ptr};

type Callback = dyn FnMut(SensorData) + Send + 'static;

/// Represents a sensor in the simulation, corresponding to
/// `carla.Sensor` in Python API.
#[derive(Clone, Debug)]
pub struct Sensor {
    inner: *mut carla_sensor_t,
}

impl Sensor {
    /// Create a Sensor from a raw C pointer.
    /// 
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_sensor_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null sensor pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn stop(&self) -> Result<()> {
        let error = unsafe { carla_sensor_stop(self.inner) };
        check_carla_error(error)
    }

    pub fn is_listening(&self) -> bool {
        unsafe { carla_sensor_is_listening(self.inner) }
    }

    pub fn listen<F>(&self, callback: F) -> Result<()>
    where
        F: FnMut(SensorData) + Send + 'static,
    {
        unsafe {
            let callback_ptr = {
                let callback: Box<Callback> = Box::new(callback);
                Box::into_raw(callback) as *mut c_void
            };

            let caller_ptr = sensor_callback_caller as *const c_void;
            let deleter_ptr = sensor_callback_deleter as *const c_void;

            let error = carla_sensor_listen(self.inner, caller_ptr, callback_ptr, deleter_ptr);
            check_carla_error(error)
        }
    }

}

impl ActorBase for Sensor {
    fn raw_ptr(&self) -> *mut carla_actor_t {
        self.inner
    }
}

impl TryFrom<Actor> for Sensor {
    type Error = Actor;

    fn try_from(value: Actor) -> std::result::Result<Self, Self::Error> {
        // Check if the actor is actually a sensor
        let is_sensor = unsafe { carla_actor_is_sensor(value.inner) };
        if is_sensor {
            let sensor_ptr = unsafe { carla_actor_as_sensor(value.inner) };
            if !sensor_ptr.is_null() {
                return Ok(Sensor { inner: sensor_ptr });
            }
        }
        Err(value)
    }
}

unsafe extern "C" fn sensor_callback_caller(callback_ptr: *mut c_void, sensor_data_ptr: *mut carla_sensor_data_t) {
    if callback_ptr.is_null() || sensor_data_ptr.is_null() {
        return;
    }
    
    let callback = callback_ptr as *mut Box<Callback>;
    let sensor_data = SensorData::from_raw_ptr(sensor_data_ptr);
    if let Ok(data) = sensor_data {
        (*callback)(data);
    }
}

unsafe extern "C" fn sensor_callback_deleter(callback_ptr: *mut c_void) {
    if !callback_ptr.is_null() {
        let callback = callback_ptr as *mut Box<Callback>;
        let _: Box<Box<Callback>> = Box::from_raw(callback);
        // Callback is automatically dropped here
    }
}

impl Drop for Sensor {
    fn drop(&mut self) {
        // Note: Sensor uses the same pointer as Actor, so we don't need to free it separately
        // The Actor destructor will handle the cleanup
        self.inner = ptr::null_mut();
    }
}

// SAFETY: Sensor wraps a thread-safe C API
unsafe impl Send for Sensor {}
unsafe impl Sync for Sensor {}
