//! IMU (Inertial Measurement Unit) sensor
//!
//! Provides accelerometer, gyroscope, and compass data.
//!
//! ## Phase 13 Implementation
//!
//! - Phase 5.1: IMU data tracking with value clamping
//! - Phase 5.1: Compass heading with cardinal directions

use carla::{client::Sensor, rpc::AttachmentType, sensor::data::ImuMeasurement};
use eyre::{eyre, Result};
use std::sync::{Arc, Mutex};
use tracing::info;

/// IMU sensor
///
/// Tracks accelerometer, gyroscope, and compass
#[allow(dead_code)]
pub struct IMUSensor {
    pub sensor: Option<Sensor>,
    data: Arc<Mutex<ImuData>>,
}

/// IMU sensor data (shared with listener)
#[derive(Debug, Clone)]
struct ImuData {
    pub accelerometer: (f32, f32, f32),
    pub gyroscope: (f32, f32, f32),
    pub compass: f32, // degrees
}

impl IMUSensor {
    /// Create a new IMU sensor
    pub fn new() -> Self {
        Self {
            sensor: None,
            data: Arc::new(Mutex::new(ImuData {
                accelerometer: (0.0, 0.0, 0.0),
                gyroscope: (0.0, 0.0, 0.0),
                compass: 0.0,
            })),
        }
    }

    /// Spawn the IMU sensor
    ///
    /// ✅ Subphase 12.5.1: Spawn IMU and set up listener
    pub fn spawn(&mut self, world: &mut crate::world::World) -> Result<()> {
        let player = world
            .player
            .as_ref()
            .ok_or_else(|| eyre!("No player vehicle available"))?;

        // Get blueprint for sensor.other.imu
        let blueprint_library = world.world.blueprint_library();
        let imu_bp = blueprint_library
            .find("sensor.other.imu")
            .ok_or_else(|| eyre!("sensor.other.imu blueprint not found"))?;

        info!("Spawning IMU sensor");

        // Spawn IMU sensor attached to vehicle
        let imu_actor = world
            .world
            .spawn_actor_opt(
                &imu_bp,
                &nalgebra::Isometry3::identity(),
                Some(player),
                AttachmentType::Rigid,
            )
            .map_err(|e| eyre!("Failed to spawn IMU sensor: {:?}", e))?;

        let imu_sensor =
            Sensor::try_from(imu_actor).map_err(|_| eyre!("Failed to convert to Sensor"))?;

        // Set up listener to receive IMU data
        let data_clone = Arc::clone(&self.data);
        imu_sensor.listen(move |sensor_data| {
            if let Ok(imu_measurement) = ImuMeasurement::try_from(sensor_data) {
                let accel = imu_measurement.accelerometer();
                let gyro = imu_measurement.gyroscope();
                let compass = imu_measurement.compass();

                // Clamp accelerometer and gyroscope to ±99.9
                let clamp = |v: f32| v.clamp(-99.9, 99.9);
                let accel = (clamp(accel.x), clamp(accel.y), clamp(accel.z));
                let gyro = (clamp(gyro.x), clamp(gyro.y), clamp(gyro.z));

                // Convert compass from radians to degrees
                let compass_degrees = compass.to_degrees();

                // Update shared data
                if let Ok(mut data) = data_clone.lock() {
                    data.accelerometer = accel;
                    data.gyroscope = gyro;
                    data.compass = compass_degrees;
                }
            }
        });

        self.sensor = Some(imu_sensor);
        info!("✓ IMU sensor spawned and listening");

        Ok(())
    }

    /// Get accelerometer data
    pub fn accelerometer(&self) -> (f32, f32, f32) {
        self.data.lock().unwrap().accelerometer
    }

    /// Get gyroscope data
    pub fn gyroscope(&self) -> (f32, f32, f32) {
        self.data.lock().unwrap().gyroscope
    }

    /// Get compass (degrees)
    pub fn compass(&self) -> f32 {
        self.data.lock().unwrap().compass
    }

    /// Get compass heading as cardinal directions
    ///
    /// Returns: "N", "NE", "E", "SE", "S", "SW", "W", "NW", or combinations
    pub fn get_heading(&self) -> String {
        let c = self.compass();
        let mut heading = String::new();

        if !(89.5..=270.5).contains(&c) {
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
}

impl Default for IMUSensor {
    fn default() -> Self {
        Self::new()
    }
}
