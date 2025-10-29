//! Radar sensor
//!
//! Detects objects with velocity information, visualized as 3D debug points.
//!
//! ## Phase 13 Implementation
//!
//! - Phase 11.1: Radar detection with debug visualization
//! - Phase 11.1: Velocity-based color coding (red = approaching, blue = receding)

use carla::client::Sensor;
use eyre::Result;

/// Radar sensor
///
/// Detects objects and draws debug visualization
pub struct RadarSensor {
    pub sensor: Option<Sensor>,
    pub enabled: bool,
    pub velocity_range: f32, // m/s
}

impl RadarSensor {
    /// Create a new radar sensor
    pub fn new() -> Self {
        Self {
            sensor: None,
            enabled: false,
            velocity_range: 7.5,
        }
    }

    /// Spawn the radar sensor
    ///
    /// TODO Phase 11.1: Implement sensor spawning and listener
    pub fn spawn(&mut self, _world: &crate::world::World) -> Result<()> {
        // TODO: Get blueprint for sensor.other.radar
        // TODO: Set attributes (horizontal_fov=35, vertical_fov=20)
        // TODO: Position based on vehicle bounding box
        // TODO: Spawn sensor attached to player vehicle
        // TODO: Set up listener to draw debug points
        Ok(())
    }

    /// Toggle radar visualization
    ///
    /// TODO Phase 11.1: G key functionality
    pub fn toggle(&mut self) {
        self.enabled = !self.enabled;
    }

    /// Draw radar detections as debug points
    ///
    /// TODO Phase 11.1: Called from sensor listener
    fn on_radar_data(&self, _detections: Vec<RadarDetection>) {
        // TODO: For each detection:
        //   - Calculate color based on velocity (red = negative, blue = positive)
        //   - Draw debug point at detection location
        //   - Use world.debug.draw_point()
    }
}

/// Radar detection
#[derive(Debug, Clone)]
pub struct RadarDetection {
    pub depth: f32,
    pub azimuth: f32,
    pub altitude: f32,
    pub velocity: f32,
}

impl Default for RadarSensor {
    fn default() -> Self {
        Self::new()
    }
}
