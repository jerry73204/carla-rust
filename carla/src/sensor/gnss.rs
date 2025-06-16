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

impl GNSSData {
    /// Create GNSSData from carla-cxx GNSSData
    pub fn from_cxx(cxx_data: carla_cxx::sensor::GNSSData) -> Self {
        Self {
            // TODO: Extract proper metadata from carla-cxx GNSSData structure
            // This requires adding timestamp, transform, and sensor_id fields to carla-cxx GNSSData
            timestamp: todo!("GNSSData::from_cxx timestamp extraction not yet implemented - missing FFI metadata"),
            transform: todo!("GNSSData::from_cxx transform extraction not yet implemented - missing FFI metadata"),
            sensor_id: todo!("GNSSData::from_cxx sensor_id extraction not yet implemented - missing FFI metadata"),
            latitude: cxx_data.latitude,
            longitude: cxx_data.longitude,
            altitude: cxx_data.altitude,
        }
    }

    /// Convert to Location in CARLA world coordinates.
    pub fn to_location(&self) -> crate::geom::Location {
        // This is a simplified conversion - in practice you'd need proper geo-referencing
        crate::geom::Location::new(self.longitude, self.latitude, self.altitude)
    }

    /// Calculate distance to another GNSS position (in meters, approximate).
    pub fn distance_to(&self, other: &GNSSData) -> f64 {
        let earth_radius = 6371000.0; // meters
        let lat1_rad = self.latitude.to_radians();
        let lat2_rad = other.latitude.to_radians();
        let delta_lat = (other.latitude - self.latitude).to_radians();
        let delta_lon = (other.longitude - self.longitude).to_radians();

        let a = (delta_lat / 2.0).sin().powi(2)
            + lat1_rad.cos() * lat2_rad.cos() * (delta_lon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

        earth_radius * c
    }
}
