//! GNSS (GPS) sensor
//!
//! Provides GPS coordinates (latitude, longitude).
//!
//! ## Phase 13 Implementation
//!
//! - Phase 5.2: GPS coordinate tracking
//! - Phase 5.2: Display in HUD with 6 decimal precision

use carla::client::Sensor;
use eyre::Result;

/// GNSS sensor
///
/// Tracks GPS coordinates
pub struct GnssSensor {
    pub sensor: Option<Sensor>,
    pub latitude: f64,
    pub longitude: f64,
}

impl GnssSensor {
    /// Create a new GNSS sensor
    pub fn new() -> Self {
        Self {
            sensor: None,
            latitude: 0.0,
            longitude: 0.0,
        }
    }

    /// Spawn the GNSS sensor
    ///
    /// TODO Phase 5.2: Implement sensor spawning and listener
    pub fn spawn(&mut self, _world: &crate::world::World) -> Result<()> {
        // TODO: Get blueprint for sensor.other.gnss
        // TODO: Set sensor location (x=1.0, z=2.8)
        // TODO: Spawn sensor attached to player vehicle
        // TODO: Set up listener to update lat/lon
        Ok(())
    }

    /// Update GPS coordinates
    ///
    /// TODO Phase 5.2: Called from sensor listener
    fn on_gnss_event(&mut self, _latitude: f64, _longitude: f64) {
        // TODO: Update coordinates
    }
}

impl Default for GnssSensor {
    fn default() -> Self {
        Self::new()
    }
}
