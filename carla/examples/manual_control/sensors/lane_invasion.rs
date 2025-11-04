//! Lane invasion sensor
//!
//! Detects when the vehicle crosses lane markings.
//!
//! ## Phase 13 Implementation
//!
//! - Phase 5.4: Lane marking crossing detection
//! - Phase 5.4: Lane type identification and notification

use carla::{
    client::Sensor,
    geom::{Location, Rotation, Transform},
    rpc::AttachmentType,
    sensor::data::LaneInvasionEvent,
};
use eyre::{eyre, Result};
use std::sync::{Arc, Mutex};
use tracing::{info, warn};

/// Lane invasion sensor
///
/// Detects lane marking crossings
#[allow(dead_code)]
pub struct LaneInvasionSensor {
    pub sensor: Option<Sensor>,
    last_invasion: Arc<Mutex<Option<String>>>,
}

impl LaneInvasionSensor {
    /// Create a new lane invasion sensor
    pub fn new() -> Self {
        Self {
            sensor: None,
            last_invasion: Arc::new(Mutex::new(None)),
        }
    }

    /// Spawn the lane invasion sensor
    ///
    /// ✅ Subphase 12.5.4: Spawn lane invasion sensor and set up listener
    pub fn spawn(&mut self, world: &mut crate::world::World) -> Result<()> {
        let player = world
            .player
            .as_ref()
            .ok_or_else(|| eyre!("No player vehicle available"))?;

        // Get blueprint for sensor.other.lane_invasion
        let blueprint_library = world.world.blueprint_library();
        let lane_invasion_bp = blueprint_library
            .find("sensor.other.lane_invasion")
            .ok_or_else(|| eyre!("sensor.other.lane_invasion blueprint not found"))?;

        info!("Spawning lane invasion sensor");

        // Spawn lane invasion sensor attached to vehicle
        let lane_invasion_actor = world
            .world
            .spawn_actor_opt(
                &lane_invasion_bp,
                &Transform {
                    location: Location::new(0.0, 0.0, 0.0),
                    rotation: Rotation::new(0.0, 0.0, 0.0),
                },
                Some(player),
                AttachmentType::Rigid,
            )
            .map_err(|e| eyre!("Failed to spawn lane invasion sensor: {:?}", e))?;

        let lane_invasion_sensor = Sensor::try_from(lane_invasion_actor)
            .map_err(|_| eyre!("Failed to convert to Sensor"))?;

        // Set up listener to receive lane invasion events
        let last_invasion_clone = Arc::clone(&self.last_invasion);
        lane_invasion_sensor.listen(move |sensor_data| {
            if let Ok(lane_invasion_event) = LaneInvasionEvent::try_from(sensor_data) {
                let markings = lane_invasion_event.crossed_lane_markings();

                if !markings.is_empty() {
                    // Simple lane invasion message
                    let count = markings.len();
                    let message = if count == 1 {
                        "Crossed lane marking".to_string()
                    } else {
                        format!("Crossed {} lane markings", count)
                    };

                    warn!("{}", message);

                    // Store last invasion for potential HUD display
                    if let Ok(mut last) = last_invasion_clone.lock() {
                        *last = Some(message);
                    }
                }
            }
        });

        self.sensor = Some(lane_invasion_sensor);
        info!("✓ Lane invasion sensor spawned and listening");

        Ok(())
    }

    /// Get and clear the last invasion message
    #[allow(dead_code)]
    pub fn take_last_invasion(&self) -> Option<String> {
        self.last_invasion.lock().unwrap().take()
    }
}

impl Default for LaneInvasionSensor {
    fn default() -> Self {
        Self::new()
    }
}
