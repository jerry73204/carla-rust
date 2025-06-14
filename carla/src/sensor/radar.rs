//! Radar sensor implementations.

use crate::{geom::Transform, sensor::SensorData, time::Timestamp};

/// Radar sensor data.
#[derive(Debug, Clone)]
pub struct RadarData {
    /// Sensor timestamp
    pub timestamp: Timestamp,
    /// Sensor transform when captured
    pub transform: Transform,
    /// Sensor ID
    pub sensor_id: u32,
    /// Radar detections
    pub detections: Vec<RadarDetection>,
}

/// Individual radar detection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RadarDetection {
    /// Velocity relative to sensor
    pub velocity: f32,
    /// Azimuth angle in radians
    pub azimuth: f32,
    /// Altitude angle in radians
    pub altitude: f32,
    /// Depth/distance from sensor
    pub depth: f32,
}

impl SensorData for RadarData {
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
        self.detections.len() * std::mem::size_of::<RadarDetection>()
    }
}
