//! IMU sensor implementations.

use crate::{geom::Transform, sensor_data::SensorData, time::Timestamp};

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
    /// Create IMUData from carla-sys IMUData
    pub fn from_cxx(cxx_data: carla_sys::sensor::IMUData) -> Self {
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

    /// Get total acceleration magnitude (mathematical utility - Euclidean norm of accelerometer vector).
    /// This is a basic mathematical operation on CARLA-provided accelerometer data.
    pub fn acceleration_magnitude(&self) -> f32 {
        (self.accelerometer[0].powi(2)
            + self.accelerometer[1].powi(2)
            + self.accelerometer[2].powi(2))
        .sqrt()
    }

    /// Get total angular velocity magnitude (mathematical utility - Euclidean norm of gyroscope vector).
    /// This is a basic mathematical operation on CARLA-provided gyroscope data.
    pub fn angular_velocity_magnitude(&self) -> f32 {
        (self.gyroscope[0].powi(2) + self.gyroscope[1].powi(2) + self.gyroscope[2].powi(2)).sqrt()
    }
}
