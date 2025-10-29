//! HUD (Heads-Up Display) module
//!
//! The HUD struct is responsible for:
//! - Real-time telemetry display (FPS, speed, location, compass)
//! - IMU data (accelerometer, gyroscope)
//! - Vehicle control state (throttle, steer, brake, gear)
//! - Collision history graph (200 frame rolling window)
//! - Nearby vehicles list (sorted by distance)
//! - Notification integration
//!
//! ## Phase 13 Implementation
//!
//! - Phase 4: Basic telemetry (FPS, speed, location, vehicle/map name)
//! - Phase 5: Sensor data (IMU compass, accelerometer, gyroscope, GNSS, collision graph)
//! - Phase 6: Control state (throttle/brake/steer bars, gear, nearby vehicles)
//! - Phase 7: Ackermann controller info

use eyre::Result;
use macroquad::prelude::*;

/// HUD display manager
///
/// Renders all on-screen telemetry and information
pub struct HUD {
    pub width: u32,
    pub height: u32,
    pub show_info: bool,
    // TODO Phase 4.1: Add FPS tracking
    // pub server_fps: f32,
    // pub client_fps: f32,

    // TODO Phase 4.2: Add telemetry text buffer
    // info_text: Vec<String>,

    // TODO Phase 7.3: Add Ackermann display state
    // show_ackermann_info: bool,
}

impl HUD {
    /// Create a new HUD
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            show_info: true,
        }
    }

    /// Update HUD state
    ///
    /// TODO Phase 4-6: Gather telemetry from world and sensors
    pub fn update(&mut self, _world: &crate::world::World, _delta_time: f32) {
        // TODO Phase 4: Update FPS counters
        // TODO Phase 5: Gather sensor data
        // TODO Phase 6: Calculate nearby vehicles
    }

    /// Render HUD to screen
    ///
    /// TODO Phase 4-6: Render all telemetry
    pub fn render(&self) -> Result<()> {
        if !self.show_info {
            return Ok(());
        }

        // TODO Phase 4.1: Render semi-transparent overlay (220x720, alpha 100)
        // TODO Phase 4.1: Render FPS counters
        // TODO Phase 4.2: Render vehicle telemetry
        // TODO Phase 5: Render sensor data (IMU, GNSS, collision graph)
        // TODO Phase 6.1: Render control state bars
        // TODO Phase 6.2: Render nearby vehicles list
        // TODO Phase 7.3: Render Ackermann info if enabled

        Ok(())
    }

    /// Toggle HUD info display
    ///
    /// TODO Phase 12.3: F1 key functionality
    pub fn toggle_info(&mut self) {
        self.show_info = !self.show_info;
    }

    /// Show/hide Ackermann controller info
    ///
    /// TODO Phase 7.3: Called when F key pressed
    pub fn show_ackermann_info(&mut self, _enabled: bool) {
        // TODO: self.show_ackermann_info = enabled;
    }
}

impl Default for HUD {
    fn default() -> Self {
        Self::new(1280, 720)
    }
}
