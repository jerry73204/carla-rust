use crate::{geom::GeoLocation, sensor::{SensorData, SensorDataBase}};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ptr;

#[derive(Clone, Debug)]
pub struct GnssMeasurement {
    inner: *mut carla_gnss_data_t,
}

impl GnssMeasurement {
    /// Create a GnssMeasurement from a raw C pointer.
    /// 
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_gnss_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null GNSS data pointer"));
        }
        Ok(Self { inner: ptr })
    }

    pub fn geo_location(&self) -> GeoLocation {
        unsafe { carla_gnss_data_get_geo_location(self.inner) }
    }

    pub fn longitude(&self) -> f64 {
        unsafe { carla_gnss_data_get_longitude(self.inner) }
    }

    pub fn latitude(&self) -> f64 {
        unsafe { carla_gnss_data_get_latitude(self.inner) }
    }

    pub fn altitude(&self) -> f64 {
        unsafe { carla_gnss_data_get_altitude(self.inner) }
    }

}

impl SensorDataBase for GnssMeasurement {
    fn raw_ptr(&self) -> *mut carla_sensor_data_t {
        self.inner as *mut carla_sensor_data_t
    }
}

impl TryFrom<SensorData> for GnssMeasurement {
    type Error = SensorData;

    fn try_from(value: SensorData) -> std::result::Result<Self, Self::Error> {
        // Check if the sensor data is actually GNSS data
        let data_type = value.data_type();
        if data_type == carla_sensor_data_type_t_CARLA_SENSOR_DATA_GNSS {
            let gnss_ptr = unsafe { carla_sensor_data_as_gnss(value.inner) };
            if !gnss_ptr.is_null() {
                return Ok(GnssMeasurement { inner: gnss_ptr });
            }
        }
        Err(value)
    }
}

impl Drop for GnssMeasurement {
    fn drop(&mut self) {
        // Note: GNSS data is managed by the sensor data lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}

// SAFETY: GnssMeasurement wraps a thread-safe C API
unsafe impl Send for GnssMeasurement {}
unsafe impl Sync for GnssMeasurement {}
