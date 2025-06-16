//! GNSS sensor implementations.
//!
//! This module provides Rust bindings for CARLA's GNSS (GPS) sensor.
//! The API closely mirrors CARLA's C++ GnssMeasurement class which provides:
//! - Base SensorData fields (timestamp, transform, sensor info)
//! - Geographic coordinates (latitude, longitude, altitude) in standard formats
//! - Direct access to location data as provided by the sensor
//!
//! The GNSS sensor provides geographic coordinates converted from the actor's
//! local position using CARLA's geographic reference point.

use crate::{geom::Transform, sensor::SensorData, time::Timestamp};

/// GNSS (GPS) sensor data.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GNSSData {
    /// Sensor timestamp
    pub timestamp: Timestamp,
    /// Sensor transform when captured
    pub transform: Transform,
    /// Sensor ID
    pub sensor_id: u32,
    /// Latitude in degrees
    pub latitude: f64,
    /// Longitude in degrees
    pub longitude: f64,
    /// Altitude in meters
    pub altitude: f64,
}

impl SensorData for GNSSData {
    fn timestamp(&self) -> Timestamp {
        self.timestamp
    }

    fn transform(&self) -> Transform {
        self.transform
    }

    fn sensor_id(&self) -> u32 {
        self.sensor_id
    }

    fn size(&self) -> usize {
        std::mem::size_of::<Self>()
    }
}

impl GNSSData {
    /// Create GNSSData from carla-cxx GNSSData
    pub fn from_cxx(cxx_data: carla_cxx::sensor::GNSSData) -> Self {
        Self {
            timestamp: Timestamp::new(
                cxx_data.timestamp.frame,
                cxx_data.timestamp.elapsed_seconds,
                cxx_data.timestamp.delta_seconds,
                cxx_data.timestamp.platform_timestamp,
            ),
            transform: Transform::new(
                crate::geom::Location::new(
                    cxx_data.transform.location.x,
                    cxx_data.transform.location.y,
                    cxx_data.transform.location.z,
                ),
                crate::geom::Rotation::new(
                    cxx_data.transform.rotation.pitch as f32,
                    cxx_data.transform.rotation.yaw as f32,
                    cxx_data.transform.rotation.roll as f32,
                ),
            ),
            sensor_id: cxx_data.sensor_id,
            latitude: cxx_data.latitude,
            longitude: cxx_data.longitude,
            altitude: cxx_data.altitude,
        }
    }
}
