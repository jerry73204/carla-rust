//! Local planner for waypoint following and vehicle control.

use super::{controller::VehiclePIDController, pid::PIDParams, types::RoadOption};
use crate::{
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

/// Local planner that follows a waypoint queue using PID control.
pub struct LocalPlanner {
    vehicle: Vehicle,
    controller: VehiclePIDController,
    waypoints_queue: VecDeque<(Waypoint, RoadOption)>,
    target_speed: f32,
    sampling_radius: f32,
    follow_speed_limits: bool,
}

impl LocalPlanner {
    /// Creates a new LocalPlanner.
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
            sampling_radius: config.sampling_radius,
            follow_speed_limits: false,
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
    pub fn run_step(&mut self, debug: bool) -> Result<VehicleControl> {
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

        // Check if we need to advance to next waypoint
        let vehicle_isometry = self.vehicle.transform();
        let vehicle_location =
            crate::geom::Location::from_na_translation(&vehicle_isometry.translation);

        let waypoint_isometry = target_waypoint.transform();
        let waypoint_location =
            crate::geom::Location::from_na_translation(&waypoint_isometry.translation);

        let distance = ((vehicle_location.x - waypoint_location.x).powi(2)
            + (vehicle_location.y - waypoint_location.y).powi(2)
            + (vehicle_location.z - waypoint_location.z).powi(2))
        .sqrt();

        if distance < self.sampling_radius {
            // Move to next waypoint
            self.waypoints_queue.pop_front();

            if self.waypoints_queue.is_empty() {
                // Reached destination
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
        }

        // Get target waypoint (might be updated after pop)
        let (target_waypoint, _) = &self.waypoints_queue[0];
        let target_isometry = target_waypoint.transform();
        let target_location =
            crate::geom::Location::from_na_translation(&target_isometry.translation);

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
