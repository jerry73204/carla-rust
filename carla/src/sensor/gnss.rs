//! GNSS sensor implementations.

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
