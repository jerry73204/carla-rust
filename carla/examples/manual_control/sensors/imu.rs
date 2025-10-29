//! IMU (Inertial Measurement Unit) sensor
//!
//! Provides accelerometer, gyroscope, and compass data.
//!
//! ## Phase 13 Implementation
//!
//! - Phase 5.1: IMU data tracking with value clamping
//! - Phase 5.1: Compass heading with cardinal directions

use carla::client::Sensor;
use eyre::Result;

/// IMU sensor
///
/// Tracks accelerometer, gyroscope, and compass
pub struct IMUSensor {
    pub sensor: Option<Sensor>,
    pub accelerometer: (f32, f32, f32),
    pub gyroscope: (f32, f32, f32),
    pub compass: f32, // degrees
}

impl IMUSensor {
    /// Create a new IMU sensor
    pub fn new() -> Self {
        Self {
            sensor: None,
            accelerometer: (0.0, 0.0, 0.0),
            gyroscope: (0.0, 0.0, 0.0),
            compass: 0.0,
        }
    }

    /// Spawn the IMU sensor
    ///
    /// TODO Phase 5.1: Implement sensor spawning and listener
    pub fn spawn(&mut self, _world: &crate::world::World) -> Result<()> {
        // TODO: Get blueprint for sensor.other.imu
        // TODO: Spawn sensor attached to player vehicle
        // TODO: Set up listener to update IMU data
        Ok(())
    }

    /// Get compass heading as cardinal directions
    ///
    /// Returns: "N", "NE", "E", "SE", "S", "SW", "W", "NW", or combinations
    pub fn get_heading(&self) -> String {
        let c = self.compass;
        let mut heading = String::new();

        if c > 270.5 || c < 89.5 {
            heading.push('N');
        }
        if (90.5..269.5).contains(&c) {
            heading.push('S');
        }
        if (0.5..179.5).contains(&c) {
            heading.push('E');
        }
        if (180.5..359.5).contains(&c) {
            heading.push('W');
        }

        heading
    }

    /// Update IMU data
    ///
    /// TODO Phase 5.1: Called from sensor listener
    fn on_imu_data(&mut self, _accel: (f32, f32, f32), _gyro: (f32, f32, f32), _compass: f32) {
        // TODO: Clamp accelerometer/gyroscope to ±99.9
        // TODO: Update compass (convert radians to degrees)
    }
}

impl Default for IMUSensor {
    fn default() -> Self {
        Self::new()
    }
}
