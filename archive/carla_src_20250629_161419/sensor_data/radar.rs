//! Radar sensor implementations.

use crate::{geom::Transform, sensor_data::SensorData, time::Timestamp};

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

impl RadarData {
    /// Create RadarData from carla-sys RadarData
    pub fn from_cxx(cxx_data: carla_sys::sensor::RadarData) -> Self {
        let detections = cxx_data
            .detections
            .iter()
            .map(|d| RadarDetection {
                velocity: d.velocity,
                azimuth: d.azimuth,
                altitude: d.altitude,
                depth: d.depth,
            })
            .collect();

        Self {
            timestamp: cxx_data.timestamp.into(),
            transform: Transform::from(cxx_data.transform),
            sensor_id: cxx_data.sensor_id,
            detections,
        }
    }

    /// Filter detections by distance range.
    pub fn filter_by_distance(&self, min_distance: f32, max_distance: f32) -> Vec<RadarDetection> {
        self.detections
            .iter()
            .filter(|d| d.depth >= min_distance && d.depth <= max_distance)
            .copied()
            .collect()
    }

    /// Filter detections by velocity range.
    pub fn filter_by_velocity(&self, min_velocity: f32, max_velocity: f32) -> Vec<RadarDetection> {
        self.detections
            .iter()
            .filter(|d| d.velocity >= min_velocity && d.velocity <= max_velocity)
            .copied()
            .collect()
    }
}
