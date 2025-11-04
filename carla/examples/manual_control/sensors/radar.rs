//! Radar sensor
//!
//! Detects objects with velocity information, visualized as 3D debug points.
//!
//! ## Phase 13 Implementation
//!
//! - ✅ Phase 11.1: Radar detection with debug visualization
//! - ✅ Phase 11.1: Velocity-based color coding (red = approaching, blue = receding)

use carla::{
    client::Sensor,
    geom::{Location, Rotation},
    rpc::{AttachmentType, Color},
    sensor::data::RadarMeasurement,
};
use eyre::{eyre, Result};
use std::sync::{Arc, Mutex};
use tracing::info;

/// Radar sensor
///
/// Detects objects and draws debug visualization with velocity-based color coding
pub struct RadarSensor {
    pub sensor: Option<Sensor>,
    pub enabled: Arc<Mutex<bool>>,
    pub velocity_range: f32, // m/s
}

impl RadarSensor {
    /// Create a new radar sensor
    pub fn new() -> Self {
        Self {
            sensor: None,
            enabled: Arc::new(Mutex::new(false)),
            velocity_range: 7.5,
        }
    }

    /// Spawn the radar sensor
    ///
    /// ✅ Phase 11.1: Spawn radar and set up listener with debug visualization
    pub fn spawn(&mut self, world: &mut crate::world::World) -> Result<()> {
        let player = world
            .player
            .as_ref()
            .ok_or_else(|| eyre!("No player vehicle available"))?;

        // Get blueprint for sensor.other.radar
        let blueprint_library = world.world.blueprint_library();
        let mut radar_bp = blueprint_library
            .find("sensor.other.radar")
            .ok_or_else(|| eyre!("sensor.other.radar blueprint not found"))?;

        // Set radar attributes
        let _ = radar_bp.set_attribute("horizontal_fov", "35");
        let _ = radar_bp.set_attribute("vertical_fov", "20");
        let _ = radar_bp.set_attribute("points_per_second", "1500");

        info!("Spawning radar sensor with FOV 35°x20°");

        // Position radar at the front of the vehicle
        let radar_transform = carla::geom::Transform {
            location: Location::new(2.5, 0.0, 1.0),
            rotation: Rotation::new(0.0, 0.0, 0.0),
        };

        // Spawn radar sensor attached to vehicle
        let radar_actor = world
            .world
            .spawn_actor_opt(
                &radar_bp,
                &radar_transform,
                Some(player),
                AttachmentType::Rigid,
            )
            .map_err(|e| eyre!("Failed to spawn radar sensor: {:?}", e))?;

        let radar_sensor =
            Sensor::try_from(radar_actor).map_err(|_| eyre!("Failed to convert to Sensor"))?;

        // Set up listener to draw debug points
        let enabled_clone = Arc::clone(&self.enabled);
        let velocity_range = self.velocity_range;
        let mut carla_world = world.world.clone();

        radar_sensor.listen(move |sensor_data| {
            // Check if radar visualization is enabled
            if let Ok(enabled) = enabled_clone.lock() {
                if !*enabled {
                    return;
                }
            }

            // Convert sensor data to radar measurement
            if let Ok(radar_data) = RadarMeasurement::try_from(sensor_data) {
                let debug = carla_world.debug();
                let detections = radar_data.as_slice();

                // Draw each detection as a debug point
                for detection in detections {
                    // Convert spherical coordinates (depth, azimuth, altitude) to Cartesian
                    let azimuth_rad = detection.azimuth.to_radians();
                    let altitude_rad = detection.altitude.to_radians();

                    let x = detection.depth * altitude_rad.cos() * azimuth_rad.cos();
                    let y = detection.depth * altitude_rad.cos() * azimuth_rad.sin();
                    let z = detection.depth * altitude_rad.sin();

                    // Get current radar sensor location
                    let location = Location::new(x, y, z);

                    // Calculate color based on velocity
                    // Negative velocity = approaching (red), positive = receding (blue)
                    let norm_velocity = (detection.velocity / velocity_range).clamp(-1.0, 1.0);
                    let color = if norm_velocity < 0.0 {
                        // Approaching: interpolate from white to red
                        let intensity = (-norm_velocity * 255.0) as u8;
                        Color::new(255, 255 - intensity, 255 - intensity)
                    } else {
                        // Receding: interpolate from white to blue
                        let intensity = (norm_velocity * 255.0) as u8;
                        Color::new(255 - intensity, 255 - intensity, 255)
                    };

                    // Draw debug point (size 0.1m, lifetime 0.1s)
                    debug.draw_point(location, 0.1, color, 0.1, false);
                }
            }
        });

        self.sensor = Some(radar_sensor);
        info!("✓ Radar sensor spawned and listening");

        Ok(())
    }

    /// Toggle radar visualization
    ///
    /// ✅ Phase 11.1: G key functionality
    pub fn toggle(&mut self) {
        if let Ok(mut enabled) = self.enabled.lock() {
            *enabled = !*enabled;
            info!(
                "Radar visualization: {}",
                if *enabled { "ON" } else { "OFF" }
            );
        }
    }

    /// Check if radar is enabled
    #[allow(dead_code)]
    pub fn is_enabled(&self) -> bool {
        self.enabled.lock().map(|e| *e).unwrap_or(false)
    }
}

impl Default for RadarSensor {
    fn default() -> Self {
        Self::new()
    }
}
