use crate::{
    geom::Vector3DExt,
    sensor::{SensorData, SensorDataBase},
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use nalgebra::Vector3;
use std::ptr;

#[derive(Clone, Debug)]
pub struct ImuMeasurement {
    inner: *mut carla_imu_data_t,
}

impl ImuMeasurement {
    /// Create an ImuMeasurement from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_imu_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null IMU data pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn accelerometer(&self) -> Vector3<f32> {
        let accel = unsafe { carla_imu_data_get_accelerometer(self.inner) };
        accel.to_na()
    }

    pub fn compass(&self) -> f32 {
        unsafe { carla_imu_data_get_compass(self.inner) }
    }

    pub fn gyroscope(&self) -> Vector3<f32> {
        let gyro = unsafe { carla_imu_data_get_gyroscope(self.inner) };
        gyro.to_na()
    }
}

impl SensorDataBase for ImuMeasurement {
    fn raw_ptr(&self) -> *mut carla_sensor_data_t {
        self.inner as *mut carla_sensor_data_t
    }
}

impl TryFrom<SensorData> for ImuMeasurement {
    type Error = SensorData;

    fn try_from(value: SensorData) -> std::result::Result<Self, Self::Error> {
        // Check if the sensor data is actually IMU data
        let data_type = value.data_type();
        if data_type == carla_sensor_data_type_t_CARLA_SENSOR_DATA_IMU {
            let imu_ptr = unsafe { carla_sensor_data_as_imu(value.inner) };
            if !imu_ptr.is_null() {
                return Ok(ImuMeasurement { inner: imu_ptr });
            }
        }
        Err(value)
    }
}

impl Drop for ImuMeasurement {
    fn drop(&mut self) {
        // Note: IMU data is managed by the sensor data lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}

// SAFETY: ImuMeasurement wraps a thread-safe C API
unsafe impl Send for ImuMeasurement {}
unsafe impl Sync for ImuMeasurement {}
