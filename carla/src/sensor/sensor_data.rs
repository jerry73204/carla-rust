use crate::geom::TransformExt;
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::Isometry3;
use std::ptr;

/// The base trait for sensor data types.
pub trait SensorDataBase {
    /// Gets the raw C pointer to sensor data.
    fn raw_ptr(&self) -> *mut carla_sensor_data_t;

    /// Gets the frame number of the data.
    fn frame(&self) -> usize {
        unsafe { carla_sensor_data_get_frame(self.raw_ptr()) }
    }

    /// Gets the timestamp of the data.
    fn timestamp(&self) -> f64 {
        unsafe { carla_sensor_data_get_timestamp(self.raw_ptr()) }
    }

    /// Gets the transformation of the sensor where the data was
    /// perceived.
    fn sensor_transform(&self) -> Isometry3<f32> {
        let transform = unsafe { carla_sensor_data_get_transform(self.raw_ptr()) };
        transform.to_na()
    }

    /// Gets the sensor data type.
    fn data_type(&self) -> carla_sensor_data_type_t {
        unsafe { carla_sensor_data_get_type(self.raw_ptr()) }
    }
}

/// The base sensor data type.
#[derive(Clone, Debug)]
pub struct SensorData {
    pub(crate) inner: *mut carla_sensor_data_t,
}

impl SensorData {
    /// Create a SensorData from a raw C pointer.
    /// 
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_sensor_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null sensor data pointer"));
        }
        Ok(Self { inner: ptr })
    }
}

impl SensorDataBase for SensorData {
    fn raw_ptr(&self) -> *mut carla_sensor_data_t {
        self.inner
    }
}

impl Drop for SensorData {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_sensor_data_destroy(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

// SAFETY: SensorData wraps a thread-safe C API
unsafe impl Send for SensorData {}
unsafe impl Sync for SensorData {}
