//! Local planner for waypoint following and vehicle control.

use super::{controller::VehiclePIDController, pid::PIDParams, types::RoadOption};
use crate::{
    agents::tools::get_speed,
    client::{ActorBase, Vehicle, Waypoint},
    rpc::VehicleControl,
};
use anyhow::Result;
use std::collections::VecDeque;

/// Configuration options for LocalPlanner.
#[derive(Debug, Clone)]
pub struct LocalPlannerConfig {
    /// Simulation timestep
    pub dt: f32,
    /// Target speed in km/h
    pub target_speed: f32,
    /// Waypoint look-ahead distance
    pub sampling_radius: f32,
    /// Lateral PID parameters
    pub lateral_control_dict: PIDParams,
    /// Longitudinal PID parameters
    pub longitudinal_control_dict: PIDParams,
    /// Maximum throttle
    pub max_throttle: f32,
    /// Maximum brake
    pub max_brake: f32,
    /// Maximum steering
    pub max_steering: f32,
    /// Lateral offset from lane center
    pub offset: f32,
}

impl Default for LocalPlannerConfig {
    fn default() -> Self {
        Self {
            dt: 1.0 / 20.0,
            target_speed: 20.0,
            sampling_radius: 2.0,
            lateral_control_dict: PIDParams {
                k_p: 1.95,
                k_i: 1.4,
                k_d: 0.01,
            },
            longitudinal_control_dict: PIDParams {
                k_p: 1.0,
                k_i: 1.0,
                k_d: 0.0,
            },
            max_throttle: 0.75,
            max_brake: 0.3,
            max_steering: 0.8,
            offset: 0.0,
        }
    }
}

/// Local planner for autonomous waypoint following using PID control.
///
/// The `LocalPlanner` manages a queue of waypoints and uses a PID controller to
/// generate vehicle control commands that smoothly follow the planned path. It handles:
/// - Waypoint queue management (adding, removing passed waypoints)
/// - Speed control (target speed or speed limits)
/// - Lateral control (steering to follow path)
/// - Distance-based waypoint pruning
///
/// This corresponds to the Python API's local planner functionality used in the
/// basic agent implementations.
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     agents::navigation::{LocalPlanner, LocalPlannerConfig},
///     client::Client,
///     prelude::*,
/// };
///
/// # fn main() -> anyhow::Result<()> {
/// let client = Client::default();
/// let world = client.world();
/// let map = world.map();
///
/// // Spawn vehicle
/// let blueprint = world
///     .blueprint_library()
///     .filter("vehicle.tesla.model3")?
///     .next()
///     .expect("Tesla Model 3 blueprint");
/// let spawn_point = world.map().recommended_spawn_points()[0];
/// let vehicle = world.spawn_actor(&blueprint, &spawn_point)?.vehicle()?;
///
/// // Create local planner
/// let config = LocalPlannerConfig::default();
/// let mut planner = LocalPlanner::new(vehicle, config);
///
/// // Set target speed
/// planner.set_speed(30.0); // 30 km/h
///
/// // Main control loop
/// loop {
///     let control = planner.run_step(false)?;
///     // Apply control to vehicle...
///     if planner.done() {
///         break; // Reached destination
///     }
/// }
/// # Ok(())
/// # }
/// ```
pub struct LocalPlanner {
    vehicle: Vehicle,
    controller: VehiclePIDController,
    waypoints_queue: VecDeque<(Waypoint, RoadOption)>,
    target_speed: f32,
    _sampling_radius: f32,
    follow_speed_limits: bool,
    base_min_distance: f32,
    distance_ratio: f32,
}

impl LocalPlanner {
    /// Creates a new `LocalPlanner` for the given vehicle.
    ///
    /// Initializes the PID controller with the provided configuration parameters.
    ///
    /// # Arguments
    ///
    /// * `vehicle` - The vehicle to control
    /// * `config` - Configuration parameters for planning and control
    pub fn new(vehicle: Vehicle, config: LocalPlannerConfig) -> Self {
        let controller = VehiclePIDController::new(
            config.lateral_control_dict,
            config.longitudinal_control_dict,
            config.offset,
            config.max_throttle,
            config.max_brake,
            config.max_steering,
            config.dt,
        );

        Self {
            vehicle,
            controller,
            waypoints_queue: VecDeque::new(),
            target_speed: config.target_speed,
            _sampling_radius: config.sampling_radius,
            follow_speed_limits: false,
            base_min_distance: 3.0,
            distance_ratio: 0.5,
        }
    }

    /// Sets the target speed in km/h.
    pub fn set_speed(&mut self, speed: f32) {
        self.target_speed = speed;
    }

    /// Enables or disables speed limit following.
    pub fn follow_speed_limits(&mut self, value: bool) {
        self.follow_speed_limits = value;
    }

    /// Sets the lateral offset.
    pub fn set_offset(&mut self, offset: f32) {
        self.controller.set_offset(offset);
    }

    /// Sets a global plan (list of waypoints with road options).
    ///
    /// This replaces or extends the current waypoint queue with a new path to follow.
    ///
    /// # Arguments
    ///
    /// * `plan` - Vector of (waypoint, road_option) tuples defining the path
    /// * `stop_waypoint_creation` - Currently unused (reserved for future use)
    /// * `clean_queue` - If true, clears existing waypoints before adding new ones
    pub fn set_global_plan(
        &mut self,
        plan: Vec<(Waypoint, RoadOption)>,
        stop_waypoint_creation: bool,
        clean_queue: bool,
    ) {
        if clean_queue {
            self.waypoints_queue.clear();
        }

        for waypoint_pair in plan {
            self.waypoints_queue.push_back(waypoint_pair);
        }

        // If not stopping waypoint creation, we could add more waypoints here
        // For now, we just use the provided plan
        let _ = stop_waypoint_creation; // Suppress warning
    }

    /// Returns the current waypoint queue.
    pub fn get_plan(&self) -> Vec<(Waypoint, RoadOption)> {
        self.waypoints_queue.iter().cloned().collect()
    }

    /// Gets the incoming waypoint and direction.
    pub fn get_incoming_waypoint_and_direction(
        &self,
        steps: usize,
    ) -> Option<(Waypoint, RoadOption)> {
        if steps >= self.waypoints_queue.len() {
            self.waypoints_queue.back().cloned()
        } else {
            self.waypoints_queue.get(steps).cloned()
        }
    }

    /// Checks if the destination has been reached.
    pub fn done(&self) -> bool {
        self.waypoints_queue.is_empty()
    }

    /// Executes one planning step and returns vehicle control.
    ///
    /// This is the main method called each frame to:
    /// 1. Remove passed waypoints from the queue
    /// 2. Get the current target waypoint
    /// 3. Run the PID controller to compute steering, throttle, and brake
    ///
    /// # Arguments
    ///
    /// * `debug` - If true, prints debug information about planning state
    ///
    /// # Returns
    ///
    /// Vehicle control commands to follow the path, or an error if control computation fails.
    ///
    /// # Behavior
    ///
    /// - Returns brake=1.0 when waypoint queue is empty (destination reached)
    /// - Automatically removes waypoints as the vehicle passes them
    /// - Uses distance-based pruning that scales with vehicle speed
    pub fn run_step(&mut self, debug: bool) -> Result<VehicleControl> {
        // Purge waypoints queue of obsolete waypoints (passed waypoints)
        let vehicle_location = self.vehicle.transform().location;
        let vehicle_speed = get_speed(&self.vehicle) / 3.6; // Convert km/h to m/s
        let min_distance = self.base_min_distance + self.distance_ratio * vehicle_speed;

        let mut num_waypoint_removed = 0;
        for (waypoint, _) in &self.waypoints_queue {
            // Keep at least one waypoint (last waypoint requires distance < 1m to remove)
            let min_dist_threshold = if self.waypoints_queue.len() - num_waypoint_removed == 1 {
                1.0
            } else {
                min_distance
            };

            let waypoint_location = waypoint.transform().location;
            let distance = ((vehicle_location.x - waypoint_location.x).powi(2)
                + (vehicle_location.y - waypoint_location.y).powi(2))
            .sqrt();

            if distance < min_dist_threshold {
                num_waypoint_removed += 1;
            } else {
                break;
            }
        }

        // Remove obsolete waypoints
        for _ in 0..num_waypoint_removed {
            self.waypoints_queue.pop_front();
        }

        if self.waypoints_queue.is_empty() {
            // No more waypoints, stop the vehicle
            return Ok(VehicleControl {
                throttle: 0.0,
                steer: 0.0,
                brake: 1.0,
                hand_brake: false,
                reverse: false,
                manual_gear_shift: false,
                gear: 0,
            });
        }

        // Get current target waypoint
        let (target_waypoint, _) = &self.waypoints_queue[0];
        let target_location = target_waypoint.transform().location;

        // Determine target speed
        let target_speed = self.target_speed;

        // TODO: Implement speed limit following when we have waypoint.lane_type() API
        // For now, just use configured target speed

        if debug {
            println!(
                "LocalPlanner: target_speed={:.1}, queue_len={}",
                target_speed,
                self.waypoints_queue.len()
            );
        }

        // Run PID controller
        let control = self
            .controller
            .run_step(&self.vehicle, target_speed, &target_location)?;

        Ok(control)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_local_planner_config_default() {
        let config = LocalPlannerConfig::default();
        assert_eq!(config.dt, 1.0 / 20.0);
        assert_eq!(config.target_speed, 20.0);
        assert_eq!(config.sampling_radius, 2.0);
    }

    #[test]
    fn test_set_speed() {
        // Note: This test can't run without a real vehicle, so we test the config
        let config = LocalPlannerConfig {
            target_speed: 50.0,
            ..Default::default()
        };
        assert_eq!(config.target_speed, 50.0);
    }
}
