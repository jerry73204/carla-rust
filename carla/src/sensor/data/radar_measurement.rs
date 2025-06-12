use super::RadarDetection;
use crate::sensor::{SensorData, SensorDataBase};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::{ptr, slice};

#[derive(Clone, Debug)]
pub struct RadarMeasurement {
    inner: *mut carla_radar_data_t,
}

impl RadarMeasurement {
    /// Create a RadarMeasurement from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_radar_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null radar data pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn detection_amount(&self) -> usize {
        unsafe { carla_radar_data_get_detection_amount(self.inner) }
    }

    pub fn as_slice(&self) -> &[RadarDetection] {
        let len = self.len();
        let data = unsafe { carla_radar_data_get_detections(self.inner) };
        if data.is_null() {
            &[]
        } else {
            unsafe { slice::from_raw_parts(data as *const RadarDetection, len) }
        }
    }

    pub fn len(&self) -> usize {
        unsafe { carla_radar_data_get_size(self.inner) }
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl SensorDataBase for RadarMeasurement {
    fn raw_ptr(&self) -> *mut carla_sensor_data_t {
        self.inner as *mut carla_sensor_data_t
    }
}

impl TryFrom<SensorData> for RadarMeasurement {
    type Error = SensorData;

    fn try_from(value: SensorData) -> std::result::Result<Self, Self::Error> {
        // Check if the sensor data is actually radar data
        let data_type = value.data_type();
        if data_type == carla_sensor_data_type_t_CARLA_SENSOR_DATA_RADAR {
            let radar_ptr = unsafe { carla_sensor_data_as_radar(value.inner) };
            if !radar_ptr.is_null() {
                return Ok(RadarMeasurement { inner: radar_ptr });
            }
        }
        Err(value)
    }
}

impl Drop for RadarMeasurement {
    fn drop(&mut self) {
        // Note: Radar data is managed by the sensor data lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}

// SAFETY: RadarMeasurement wraps a thread-safe C API
unsafe impl Send for RadarMeasurement {}
unsafe impl Sync for RadarMeasurement {}
