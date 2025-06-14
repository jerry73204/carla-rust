//! IMU sensor implementations.

use crate::{geom::Transform, sensor::SensorData, time::Timestamp};

/// IMU (Inertial Measurement Unit) sensor data.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IMUData {
    /// Sensor timestamp
    pub timestamp: Timestamp,
    /// Sensor transform when captured
    pub transform: Transform,
    /// Sensor ID
    pub sensor_id: u32,
    /// Accelerometer readings (m/s²)
    pub accelerometer: [f32; 3],
    /// Gyroscope readings (rad/s)
    pub gyroscope: [f32; 3],
    /// Compass reading (radians)
    pub compass: f32,
}

impl SensorData for IMUData {
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
