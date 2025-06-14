use super::Walker;
use crate::{
    geom::{Location, Vector3D},
    utils::check_carla_error,
};
use anyhow::{anyhow, Result};
use carla_sys::*;

/// Walker navigation system for pathfinding and route planning.
///
/// This provides higher-level navigation capabilities for walkers,
/// including route planning, obstacle avoidance, and pathfinding.
#[derive(Clone, Debug)]
pub struct WalkerNavigation {
    /// The walker being controlled.
    walker: Walker,
    /// Current destination if set.
    destination: Option<Vector3D>,
    /// Navigation state.
    state: NavigationState,
}

impl WalkerNavigation {
    /// Create a new walker navigation system for the given walker.
    pub fn new(walker: Walker) -> Self {
        Self {
            walker,
            destination: None,
            state: NavigationState::Idle,
        }
    }

    /// Get the walker being controlled.
    pub fn walker(&self) -> &Walker {
        &self.walker
    }

    /// Set a destination for the walker to navigate to.
    pub fn set_destination(&mut self, destination: Vector3D) -> Result<()> {
        self.destination = Some(destination);
        self.state = NavigationState::Planning;
        Ok(())
    }

    /// Get the current destination if set.
    pub fn destination(&self) -> Option<Vector3D> {
        self.destination
    }

    /// Clear the current destination.
    pub fn clear_destination(&mut self) {
        self.destination = None;
        self.state = NavigationState::Idle;
    }

    /// Get the current navigation state.
    pub fn state(&self) -> NavigationState {
        self.state
    }

    /// Calculate the distance to the destination.
    pub fn distance_to_destination(&self) -> Option<f32> {
        if let Some(dest) = self.destination {
            let current_loc = self.walker.actor().get_location();
            let diff = Vector3D {
                x: dest.x - current_loc.x,
                y: dest.y - current_loc.y,
                z: dest.z - current_loc.z,
            };
            Some((diff.x * diff.x + diff.y * diff.y + diff.z * diff.z).sqrt())
        } else {
            None
        }
    }

    /// Check if the walker has reached the destination (within tolerance).
    pub fn has_reached_destination(&self, tolerance: f32) -> bool {
        if let Some(distance) = self.distance_to_destination() {
            distance <= tolerance
        } else {
            false
        }
    }

    /// Update navigation state based on current position.
    pub fn update(&mut self) -> Result<()> {
        match self.state {
            NavigationState::Planning => {
                if let Some(dest) = self.destination {
                    // Start moving towards destination
                    let current_loc = self.walker.actor().get_location();
                    let direction = Vector3D {
                        x: dest.x - current_loc.x,
                        y: dest.y - current_loc.y,
                        z: dest.z - current_loc.z,
                    };

                    // Normalize direction
                    let length = (direction.x * direction.x
                        + direction.y * direction.y
                        + direction.z * direction.z)
                        .sqrt();
                    if length > 0.0 {
                        let normalized = Vector3D {
                            x: direction.x / length,
                            y: direction.y / length,
                            z: direction.z / length,
                        };

                        // Apply control to walker
                        let control = super::WalkerControl::new().direction(normalized).speed(1.0);
                        self.walker.apply_control(&control)?;
                        self.state = NavigationState::Moving;
                    }
                }
            }
            NavigationState::Moving => {
                // Check if we've reached the destination
                if self.has_reached_destination(1.0) {
                    self.state = NavigationState::Arrived;
                    // Stop the walker
                    let control = super::WalkerControl::new().speed(0.0);
                    self.walker.apply_control(&control)?;
                }
            }
            NavigationState::Idle | NavigationState::Arrived => {
                // Nothing to do
            }
        }
        Ok(())
    }

    /// Get the current walking speed.
    pub fn get_speed(&self) -> f32 {
        self.walker.get_control().speed
    }

    /// Check if the walker is currently moving.
    pub fn is_moving(&self) -> bool {
        self.get_speed() > 0.01
    }

    /// Get the current walking direction.
    pub fn get_direction(&self) -> Vector3D {
        self.walker.get_control().direction
    }

    /// Stop the walker immediately.
    pub fn stop(&self) -> Result<()> {
        let control = super::WalkerControl::new().speed(0.0);
        self.walker.apply_control(&control)
    }

    /// Get the walker's current velocity from physics.
    pub fn get_velocity(&self) -> Vector3D {
        self.walker.get_velocity()
    }
}

/// Navigation state for walker pathfinding.
#[derive(Clone, Debug, Copy, PartialEq)]
pub enum NavigationState {
    /// Walker is idle with no destination.
    Idle,
    /// Walker is planning a route to the destination.
    Planning,
    /// Walker is actively moving towards the destination.
    Moving,
    /// Walker has arrived at the destination.
    Arrived,
}

impl Default for NavigationState {
    fn default() -> Self {
        Self::Idle
    }
}

/// Route planning result for walker navigation.
#[derive(Clone, Debug)]
pub struct WalkerRoute {
    /// Waypoints along the route.
    pub waypoints: Vec<Vector3D>,
    /// Total estimated distance.
    pub distance: f32,
    /// Estimated travel time in seconds.
    pub estimated_time: f32,
}

impl WalkerRoute {
    /// Create a new route.
    pub fn new(waypoints: Vec<Vector3D>) -> Self {
        let distance = Self::calculate_distance(&waypoints);
        let estimated_time = distance / 1.4; // Average walking speed ~1.4 m/s

        Self {
            waypoints,
            distance,
            estimated_time,
        }
    }

    /// Calculate total distance along waypoints.
    fn calculate_distance(waypoints: &[Vector3D]) -> f32 {
        if waypoints.len() < 2 {
            return 0.0;
        }

        let mut total = 0.0;
        for i in 1..waypoints.len() {
            let prev = &waypoints[i - 1];
            let curr = &waypoints[i];
            let diff = Vector3D {
                x: curr.x - prev.x,
                y: curr.y - prev.y,
                z: curr.z - prev.z,
            };
            total += (diff.x * diff.x + diff.y * diff.y + diff.z * diff.z).sqrt();
        }
        total
    }

    /// Check if the route is empty.
    pub fn is_empty(&self) -> bool {
        self.waypoints.is_empty()
    }

    /// Get the next waypoint in the route.
    pub fn next_waypoint(&self) -> Option<Vector3D> {
        self.waypoints.first().copied()
    }

    /// Get the final destination.
    pub fn destination(&self) -> Option<Vector3D> {
        self.waypoints.last().copied()
    }
}
