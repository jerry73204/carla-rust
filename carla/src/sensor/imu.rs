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

impl IMUData {
    /// Create IMUData from carla-cxx IMUData
    pub fn from_cxx(cxx_data: carla_cxx::sensor::IMUData) -> Self {
        Self {
            // TODO: Extract proper metadata from carla-cxx IMUData structure
            // This requires adding timestamp, transform, and sensor_id fields to carla-cxx IMUData
            timestamp: todo!(
                "IMUData::from_cxx timestamp extraction not yet implemented - missing FFI metadata"
            ),
            transform: todo!(
                "IMUData::from_cxx transform extraction not yet implemented - missing FFI metadata"
            ),
            sensor_id: todo!(
                "IMUData::from_cxx sensor_id extraction not yet implemented - missing FFI metadata"
            ),
            accelerometer: [
                cxx_data.accelerometer_x as f32,
                cxx_data.accelerometer_y as f32,
                cxx_data.accelerometer_z as f32,
            ],
            gyroscope: [
                cxx_data.gyroscope_x as f32,
                cxx_data.gyroscope_y as f32,
                cxx_data.gyroscope_z as f32,
            ],
            compass: cxx_data.compass as f32,
        }
    }

    /// Get total acceleration magnitude.
    pub fn acceleration_magnitude(&self) -> f32 {
        (self.accelerometer[0].powi(2)
            + self.accelerometer[1].powi(2)
            + self.accelerometer[2].powi(2))
        .sqrt()
    }

    /// Get total angular velocity magnitude.
    pub fn angular_velocity_magnitude(&self) -> f32 {
        (self.gyroscope[0].powi(2) + self.gyroscope[1].powi(2) + self.gyroscope[2].powi(2)).sqrt()
    }
}
