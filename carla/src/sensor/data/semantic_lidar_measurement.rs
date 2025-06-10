use super::SemanticLidarDetection;
use crate::sensor::{SensorData, SensorDataBase};
use anyhow::{anyhow, Result};
use carla_sys::*;
use ndarray::ArrayView2;
use std::{ptr, slice};

#[derive(Clone, Debug)]
pub struct SemanticLidarMeasurement {
    inner: *mut carla_semantic_lidar_data_t,
}

impl SemanticLidarMeasurement {
    /// Create a SemanticLidarMeasurement from a raw C pointer.
    /// 
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_semantic_lidar_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null semantic LiDAR data pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn horizontal_angle(&self) -> f32 {
        unsafe { carla_semantic_lidar_data_get_horizontal_angle(self.inner) }
    }

    pub fn point_count(&self, channel: usize) -> Option<usize> {
        if channel < self.channel_count() {
            Some(unsafe { carla_semantic_lidar_data_get_point_count(self.inner, channel) })
        } else {
            None
        }
    }

    pub fn channel_count(&self) -> usize {
        unsafe { carla_semantic_lidar_data_get_channel_count(self.inner) }
    }

    pub fn as_slice(&self) -> &[SemanticLidarDetection] {
        let len = self.len();
        let data = unsafe { carla_semantic_lidar_data_get_points(self.inner) };
        if data.is_null() {
            &[]
        } else {
            unsafe { slice::from_raw_parts(data as *const SemanticLidarDetection, len) }
        }
    }

    pub fn as_array(&self) -> ArrayView2<'_, SemanticLidarDetection> {
        let len = self.len();
        let ih = self.channel_count();
        let iw = len / ih;
        assert!(ih * iw == len);
        ArrayView2::from_shape((iw, ih), self.as_slice()).unwrap()
    }

    pub fn len(&self) -> usize {
        unsafe { carla_semantic_lidar_data_get_size(self.inner) }
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

}

impl SensorDataBase for SemanticLidarMeasurement {
    fn raw_ptr(&self) -> *mut carla_sensor_data_t {
        self.inner as *mut carla_sensor_data_t
    }
}

impl TryFrom<SensorData> for SemanticLidarMeasurement {
    type Error = SensorData;

    fn try_from(value: SensorData) -> std::result::Result<Self, Self::Error> {
        // Check if the sensor data is actually semantic LiDAR data
        let data_type = value.data_type();
        if data_type == carla_sensor_data_type_t_CARLA_SENSOR_DATA_SEMANTIC_LIDAR {
            let semantic_lidar_ptr = unsafe { carla_sensor_data_as_semantic_lidar(value.inner) };
            if !semantic_lidar_ptr.is_null() {
                return Ok(SemanticLidarMeasurement { inner: semantic_lidar_ptr });
            }
        }
        Err(value)
    }
}

impl Drop for SemanticLidarMeasurement {
    fn drop(&mut self) {
        // Note: Semantic LiDAR data is managed by the sensor data lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}

// SAFETY: SemanticLidarMeasurement wraps a thread-safe C API
unsafe impl Send for SemanticLidarMeasurement {}
unsafe impl Sync for SemanticLidarMeasurement {}
