//! Lane invasion sensor
//!
//! Detects when the vehicle crosses lane markings.
//!
//! ## Phase 13 Implementation
//!
//! - Phase 5.4: Lane marking crossing detection
//! - Phase 5.4: Lane type identification and notification

use carla::client::Sensor;
use eyre::Result;

/// Lane invasion sensor
///
/// Detects lane marking crossings
pub struct LaneInvasionSensor {
    pub sensor: Option<Sensor>,
}

impl LaneInvasionSensor {
    /// Create a new lane invasion sensor
    pub fn new() -> Self {
        Self { sensor: None }
    }

    /// Spawn the lane invasion sensor
    ///
    /// TODO Phase 5.4: Implement sensor spawning and listener
    pub fn spawn(&mut self, _world: &crate::world::World) -> Result<()> {
        // TODO: Get blueprint for sensor.other.lane_invasion
        // TODO: Spawn sensor attached to player vehicle
        // TODO: Set up listener to show notifications
        Ok(())
    }

    /// Handle lane invasion event
    ///
    /// TODO Phase 5.4: Called from sensor listener
    fn on_invasion(&mut self, _lane_types: Vec<String>) {
        // TODO: Format lane types (Solid, Broken, etc.)
        // TODO: Show notification: "Crossed line X and Y"
    }
}

impl Default for LaneInvasionSensor {
    fn default() -> Self {
        Self::new()
    }
}
