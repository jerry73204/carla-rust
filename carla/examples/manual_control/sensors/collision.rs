//! Collision sensor
//!
//! Detects collisions and maintains a history for the collision graph in the HUD.
//!
//! ## Phase 13 Implementation
//!
//! - Phase 5.3: Collision detection and history tracking
//! - Phase 5.3: Collision intensity graph (200 frame rolling window)

use carla::client::Sensor;
use eyre::Result;
use std::collections::VecDeque;

/// Collision history entry
#[derive(Debug, Clone)]
pub struct CollisionEvent {
    pub frame: usize,
    pub intensity: f32,
}

/// Collision sensor
///
/// Tracks collision events and maintains a history for display
pub struct CollisionSensor {
    pub sensor: Option<Sensor>,
    pub history: VecDeque<CollisionEvent>,
}

impl CollisionSensor {
    /// Create a new collision sensor
    pub fn new() -> Self {
        Self {
            sensor: None,
            history: VecDeque::with_capacity(4000),
        }
    }

    /// Spawn the collision sensor
    ///
    /// TODO Phase 5.3: Implement sensor spawning and listener
    pub fn spawn(&mut self, _world: &crate::world::World) -> Result<()> {
        // TODO: Get blueprint for sensor.other.collision
        // TODO: Spawn sensor attached to player vehicle
        // TODO: Set up listener to record collision events
        Ok(())
    }

    /// Get collision history for graphing
    ///
    /// Returns normalized collision intensities for the last 200 frames
    pub fn get_collision_history(&self, _current_frame: usize) -> Vec<f32> {
        // TODO Phase 5.3: Extract last 200 frames, normalize by max
        vec![]
    }

    /// Record a collision event
    ///
    /// TODO Phase 5.3: Called from sensor listener
    fn on_collision(&mut self, _frame: usize, _intensity: f32) {
        // TODO: Add to history
        // TODO: Trim history to 4000 entries
        // TODO: Show notification via HUD
    }
}

impl Default for CollisionSensor {
    fn default() -> Self {
        Self::new()
    }
}
