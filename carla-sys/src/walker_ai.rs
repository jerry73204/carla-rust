//! Walker AI Controller implementation for CARLA.

use crate::ffi::{self, Actor, SimpleWalkerDestination, WalkerAIController};
use anyhow::Result;
use cxx::SharedPtr;

/// High-level wrapper for CARLA Walker AI Controller
pub struct WalkerAIControllerWrapper {
    inner: SharedPtr<WalkerAIController>,
}

impl WalkerAIControllerWrapper {
    /// Create a WalkerAIControllerWrapper from an Actor (performs cast)
    pub fn from_actor(actor_ptr: SharedPtr<Actor>) -> Option<Self> {
        let controller_ptr = ffi::Actor_CastToWalkerAIController(actor_ptr);
        if controller_ptr.is_null() {
            None
        } else {
            Some(Self {
                inner: controller_ptr,
            })
        }
    }

    /// Start the AI controller
    pub fn start(&self) -> Result<()> {
        ffi::WalkerAIController_Start(&self.inner);
        Ok(())
    }

    /// Stop the AI controller
    pub fn stop(&self) -> Result<()> {
        ffi::WalkerAIController_Stop(&self.inner);
        Ok(())
    }

    /// Set maximum walking speed for the AI
    pub fn set_max_speed(&self, max_speed: f32) -> Result<()> {
        ffi::WalkerAIController_SetMaxSpeed(&self.inner, max_speed);
        Ok(())
    }

    /// Make the walker go to a specific location
    pub fn go_to_location(&self, location: Location) -> Result<()> {
        let destination = SimpleWalkerDestination {
            x: location.x,
            y: location.y,
            z: location.z,
        };
        ffi::WalkerAIController_GoToLocation(&self.inner, &destination);
        Ok(())
    }

    /// Get a random location for the walker to navigate to
    pub fn get_random_location(&self) -> Option<Location> {
        let dest = ffi::WalkerAIController_GetRandomLocation(&self.inner);
        // Check if location is valid (not 0,0,0)
        if dest.x == 0.0 && dest.y == 0.0 && dest.z == 0.0 {
            None
        } else {
            Some(Location {
                x: dest.x,
                y: dest.y,
                z: dest.z,
            })
        }
    }

    /// Check if the controller has a valid destination
    pub fn has_valid_destination(&self) -> bool {
        ffi::WalkerAIController_HasValidDestination(&self.inner)
    }
}

/// 3D location for walker navigation
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Location {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Location {
    /// Create a new location
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Get distance to another location
    pub fn distance_to(&self, other: &Location) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// Walker AI behavior configuration
#[derive(Debug, Clone)]
pub struct WalkerAIBehavior {
    /// Maximum walking speed in m/s
    pub max_speed: f32,
    /// Whether the walker should cross roads
    pub cross_roads: bool,
    /// Probability of crossing at non-crossing locations (0.0-1.0)
    pub random_crossing_probability: f32,
}

impl Default for WalkerAIBehavior {
    fn default() -> Self {
        Self {
            max_speed: 1.4, // Average walking speed
            cross_roads: true,
            random_crossing_probability: 0.1,
        }
    }
}
