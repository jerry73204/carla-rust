//! Collision sensor
//!
//! Detects collisions and maintains a history for the collision graph in the HUD.
//!
//! ## Phase 13 Implementation
//!
//! - Phase 5.3: Collision detection and history tracking
//! - Phase 5.3: Collision intensity graph (200 frame rolling window)

use carla::{
    client::Sensor, prelude::*, rpc::AttachmentType,
    sensor::data::CollisionEvent as CarlaCollisionEvent,
};
use eyre::{eyre, Result};
use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
};
use tracing::{info, warn};

/// Collision history entry
#[derive(Debug, Clone)]
pub struct CollisionEntry {
    pub frame: u64,
    pub intensity: f32,
}

/// Collision sensor
///
/// Tracks collision events and maintains a history for display
#[allow(dead_code)]
pub struct CollisionSensor {
    pub sensor: Option<Sensor>,
    history: Arc<Mutex<VecDeque<CollisionEntry>>>,
    frame_count: Arc<Mutex<u64>>,
}

impl CollisionSensor {
    /// Create a new collision sensor
    pub fn new() -> Self {
        Self {
            sensor: None,
            history: Arc::new(Mutex::new(VecDeque::with_capacity(4000))),
            frame_count: Arc::new(Mutex::new(0)),
        }
    }

    /// Spawn the collision sensor
    ///
    /// ✅ Subphase 12.5.3: Spawn collision sensor and set up listener
    pub fn spawn(&mut self, world: &mut crate::world::World) -> Result<()> {
        let player = world
            .player
            .as_ref()
            .ok_or_else(|| eyre!("No player vehicle available"))?;

        // Get blueprint for sensor.other.collision
        let blueprint_library = world.world.blueprint_library();
        let collision_bp = blueprint_library
            .find("sensor.other.collision")
            .ok_or_else(|| eyre!("sensor.other.collision blueprint not found"))?;

        info!("Spawning collision sensor");

        // Spawn collision sensor attached to vehicle
        let collision_actor = world
            .world
            .spawn_actor_opt(
                &collision_bp,
                &nalgebra::Isometry3::identity(),
                Some(player),
                AttachmentType::Rigid,
            )
            .map_err(|e| eyre!("Failed to spawn collision sensor: {:?}", e))?;

        let collision_sensor =
            Sensor::try_from(collision_actor).map_err(|_| eyre!("Failed to convert to Sensor"))?;

        // Set up listener to receive collision events
        let history_clone = Arc::clone(&self.history);
        let frame_count_clone = Arc::clone(&self.frame_count);
        collision_sensor.listen(move |sensor_data| {
            if let Ok(collision_event) = CarlaCollisionEvent::try_from(sensor_data) {
                let impulse = collision_event.normal_impulse();
                let intensity =
                    (impulse.x * impulse.x + impulse.y * impulse.y + impulse.z * impulse.z).sqrt();

                let frame = *frame_count_clone.lock().unwrap();

                // Log collision
                let other_actor = collision_event.other_actor();
                if let Some(other) = other_actor {
                    warn!(
                        "Collision with actor ID {} (intensity: {:.1})",
                        other.id(),
                        intensity
                    );
                } else {
                    warn!("Collision with static object (intensity: {:.1})", intensity);
                }

                // Add to history
                if let Ok(mut history) = history_clone.lock() {
                    history.push_back(CollisionEntry { frame, intensity });
                    // Trim history to 4000 entries
                    while history.len() > 4000 {
                        history.pop_front();
                    }
                }
            }
        });

        self.sensor = Some(collision_sensor);
        info!("✓ Collision sensor spawned and listening");

        Ok(())
    }

    /// Update frame count (call every frame)
    pub fn update_frame(&self) {
        if let Ok(mut frame) = self.frame_count.lock() {
            *frame += 1;
        }
    }

    /// Get collision history for graphing
    ///
    /// Returns normalized collision intensities for the last 200 frames
    pub fn get_collision_history(&self, current_frame: u64) -> Vec<f32> {
        let history = self.history.lock().unwrap();

        if history.is_empty() {
            return vec![0.0; 200];
        }

        // Get entries from last 200 frames
        let start_frame = current_frame.saturating_sub(200);
        let recent: Vec<&CollisionEntry> =
            history.iter().filter(|e| e.frame >= start_frame).collect();

        if recent.is_empty() {
            return vec![0.0; 200];
        }

        // Find max intensity for normalization
        let max_intensity = recent.iter().map(|e| e.intensity).fold(0.0f32, f32::max);

        // Build array of 200 frames
        let mut result = vec![0.0; 200];
        for entry in recent {
            let frame_index = (entry.frame - start_frame) as usize;
            if frame_index < 200 {
                let normalized = if max_intensity > 0.0 {
                    entry.intensity / max_intensity
                } else {
                    0.0
                };
                result[frame_index] = normalized;
            }
        }

        result
    }

    /// Get current frame count
    pub fn frame_count(&self) -> u64 {
        *self.frame_count.lock().unwrap()
    }
}

impl Default for CollisionSensor {
    fn default() -> Self {
        Self::new()
    }
}
