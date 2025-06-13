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
        unsafe { carla_sensor_data_get_frame(self.raw_ptr()) as usize }
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

    /// Checks if this sensor data is valid.
    fn is_valid(&self) -> bool {
        !self.raw_ptr().is_null()
    }

    /// Gets the data size in bytes.
    fn data_size(&self) -> usize {
        // TODO: Implement when C API provides carla_sensor_data_get_size
        0
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

    /// Create a SensorData with validation.
    pub(crate) fn from_raw_ptr_validated(ptr: *mut carla_sensor_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null sensor data pointer"));
        }

        // Additional validation
        let data_type = unsafe { carla_sensor_data_get_type(ptr) };
        if data_type == 0 {
            return Err(anyhow!("Invalid sensor data type"));
        }

        // TODO: Add data size validation when C API provides carla_sensor_data_get_size
        // let data_size = unsafe { carla_sensor_data_get_size(ptr) };
        // if data_size == 0 {
        //     return Err(anyhow!("Zero-sized sensor data"));
        // }

        Ok(Self { inner: ptr })
    }

    /// Try to convert to specific sensor data type with type checking.
    pub fn try_as_image_data(&self) -> Result<crate::sensor::ImageData> {
        use crate::sensor::legacy_data::ImageData;
        if self.data_type() == carla_sensor_data_type_t_CARLA_SENSOR_DATA_IMAGE {
            ImageData::from_sensor_data(self)
        } else {
            Err(anyhow!("Sensor data is not image data"))
        }
    }

    /// Try to convert to LiDAR data with type checking.
    pub fn try_as_lidar_data(&self) -> Result<crate::sensor::LidarData> {
        use crate::sensor::legacy_data::LidarData;
        if self.data_type() == carla_sensor_data_type_t_CARLA_SENSOR_DATA_LIDAR {
            LidarData::from_sensor_data(self)
        } else {
            Err(anyhow!("Sensor data is not LiDAR data"))
        }
    }

    /// Try to convert to radar data with type checking.
    pub fn try_as_radar_data(&self) -> Result<crate::sensor::RadarData> {
        use crate::sensor::legacy_data::RadarData;
        if self.data_type() == carla_sensor_data_type_t_CARLA_SENSOR_DATA_RADAR {
            RadarData::from_sensor_data(self)
        } else {
            Err(anyhow!("Sensor data is not radar data"))
        }
    }

    /// Try to convert to IMU data with type checking.
    pub fn try_as_imu_data(&self) -> Result<crate::sensor::ImuData> {
        use crate::sensor::legacy_data::ImuData;
        if self.data_type() == carla_sensor_data_type_t_CARLA_SENSOR_DATA_IMU {
            ImuData::from_sensor_data(self)
        } else {
            Err(anyhow!("Sensor data is not IMU data"))
        }
    }

    /// Try to convert to GNSS data with type checking.
    pub fn try_as_gnss_data(&self) -> Result<crate::sensor::GnssData> {
        use crate::sensor::legacy_data::GnssData;
        if self.data_type() == carla_sensor_data_type_t_CARLA_SENSOR_DATA_GNSS {
            GnssData::from_sensor_data(self)
        } else {
            Err(anyhow!("Sensor data is not GNSS data"))
        }
    }

    /// Get a type-safe sensor data accessor pattern.
    pub fn data_accessor(&self) -> SensorDataAccessor {
        SensorDataAccessor { data: self }
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

/// Type-safe sensor data accessor for pattern matching.
pub struct SensorDataAccessor<'a> {
    data: &'a SensorData,
}

impl<'a> SensorDataAccessor<'a> {
    /// Access data as image data if possible.
    pub fn as_image(&self) -> Option<Result<crate::sensor::ImageData>> {
        if self.data.data_type() == carla_sensor_data_type_t_CARLA_SENSOR_DATA_IMAGE {
            Some(self.data.try_as_image_data())
        } else {
            None
        }
    }

    /// Access data as LiDAR data if possible.
    pub fn as_lidar(&self) -> Option<Result<crate::sensor::LidarData>> {
        if self.data.data_type() == carla_sensor_data_type_t_CARLA_SENSOR_DATA_LIDAR {
            Some(self.data.try_as_lidar_data())
        } else {
            None
        }
    }

    /// Access data as radar data if possible.
    pub fn as_radar(&self) -> Option<Result<crate::sensor::RadarData>> {
        if self.data.data_type() == carla_sensor_data_type_t_CARLA_SENSOR_DATA_RADAR {
            Some(self.data.try_as_radar_data())
        } else {
            None
        }
    }

    /// Access data as IMU data if possible.
    pub fn as_imu(&self) -> Option<Result<crate::sensor::ImuData>> {
        if self.data.data_type() == carla_sensor_data_type_t_CARLA_SENSOR_DATA_IMU {
            Some(self.data.try_as_imu_data())
        } else {
            None
        }
    }

    /// Access data as GNSS data if possible.
    pub fn as_gnss(&self) -> Option<Result<crate::sensor::GnssData>> {
        if self.data.data_type() == carla_sensor_data_type_t_CARLA_SENSOR_DATA_GNSS {
            Some(self.data.try_as_gnss_data())
        } else {
            None
        }
    }

    /// Get the underlying sensor data type.
    pub fn data_type(&self) -> carla_sensor_data_type_t {
        self.data.data_type()
    }

    /// Get data metadata.
    pub fn metadata(&self) -> SensorDataMetadata {
        SensorDataMetadata {
            frame: self.data.frame(),
            timestamp: self.data.timestamp(),
            data_type: self.data.data_type(),
            data_size: self.data.data_size(),
            is_valid: self.data.is_valid(),
        }
    }
}

/// Sensor data metadata for analysis.
#[derive(Debug, Clone)]
pub struct SensorDataMetadata {
    /// Frame number.
    pub frame: usize,
    /// Timestamp.
    pub timestamp: f64,
    /// Data type.
    pub data_type: carla_sensor_data_type_t,
    /// Data size in bytes.
    pub data_size: usize,
    /// Validity flag.
    pub is_valid: bool,
}

// SAFETY: SensorData wraps a thread-safe C API
unsafe impl Send for SensorData {}
unsafe impl Sync for SensorData {}
