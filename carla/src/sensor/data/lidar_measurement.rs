use super::LidarDetection;
use crate::sensor::{SensorData, SensorDataBase};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::{ptr, slice};

#[derive(Clone, Debug)]
pub struct LidarMeasurement {
    inner: *mut carla_lidar_data_t,
}

impl LidarMeasurement {
    /// Create a LidarMeasurement from a raw C pointer.
    /// 
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_lidar_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null LiDAR data pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn horizontal_angle(&self) -> f32 {
        unsafe { carla_lidar_data_get_horizontal_angle(self.inner) }
    }

    pub fn point_count(&self, channel: usize) -> Option<usize> {
        if channel < self.channel_count() {
            Some(unsafe { carla_lidar_data_get_point_count(self.inner, channel) })
        } else {
            None
        }
    }

    pub fn channel_count(&self) -> usize {
        unsafe { carla_lidar_data_get_channel_count(self.inner) }
    }

    pub fn as_slice(&self) -> &[LidarDetection] {
        let len = self.len();
        let data = unsafe { carla_lidar_data_get_points(self.inner) };
        if data.is_null() {
            &[]
        } else {
            unsafe { slice::from_raw_parts(data as *const LidarDetection, len) }
        }
    }

    pub fn len(&self) -> usize {
        unsafe { carla_lidar_data_get_size(self.inner) }
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

}

impl SensorDataBase for LidarMeasurement {
    fn raw_ptr(&self) -> *mut carla_sensor_data_t {
        self.inner as *mut carla_sensor_data_t
    }
}

impl TryFrom<SensorData> for LidarMeasurement {
    type Error = SensorData;

    fn try_from(value: SensorData) -> std::result::Result<Self, Self::Error> {
        // Check if the sensor data is actually LiDAR data
        let data_type = value.data_type();
        if data_type == carla_sensor_data_type_t_CARLA_SENSOR_DATA_LIDAR {
            let lidar_ptr = unsafe { carla_sensor_data_as_lidar(value.inner) };
            if !lidar_ptr.is_null() {
                return Ok(LidarMeasurement { inner: lidar_ptr });
            }
        }
        Err(value)
    }
}

impl Drop for LidarMeasurement {
    fn drop(&mut self) {
        // Note: LiDAR data is managed by the sensor data lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}

// SAFETY: LidarMeasurement wraps a thread-safe C API
unsafe impl Send for LidarMeasurement {}
unsafe impl Sync for LidarMeasurement {}
