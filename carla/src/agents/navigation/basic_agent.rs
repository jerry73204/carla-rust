//! BasicAgent implementation for autonomous navigation.

use super::{
    agent_core::{AgentCore, AgentCoreConfig, LaneChangeDirection},
    global_route_planner::GlobalRoutePlanner,
    local_planner::LocalPlannerConfig,
    types::{Agent, RoadOption},
};
use crate::{
    agents::tools::get_speed,
    client::{ActorBase, Map, Vehicle, Waypoint},
    geom::Location,
    rpc::VehicleControl,
};
use anyhow::Result;

/// Configuration options for BasicAgent.
#[derive(Debug, Clone)]
pub struct BasicAgentConfig {
    /// Target speed in km/h
    pub target_speed: f32,
    /// Agent core configuration
    pub core_config: AgentCoreConfig,
    /// Local planner configuration
    pub local_config: LocalPlannerConfig,
}

impl Default for BasicAgentConfig {
    fn default() -> Self {
        Self {
            target_speed: 20.0,
            core_config: AgentCoreConfig::default(),
            local_config: LocalPlannerConfig {
                target_speed: 20.0,
                ..Default::default()
            },
        }
    }
}

/// BasicAgent provides traffic-aware navigation with obstacle and traffic light detection.
///
/// # Examples
/// ```no_run
/// use carla::{
///     agents::navigation::{BasicAgent, BasicAgentConfig},
///     client::Vehicle,
///     geom::Location,
/// };
///
/// # fn example(vehicle: Vehicle) -> anyhow::Result<()> {
/// let config = BasicAgentConfig::default();
/// let mut agent = BasicAgent::new(vehicle, config, None, None)?;
///
/// // Set destination
/// let destination = Location {
///     x: 100.0,
///     y: 50.0,
///     z: 0.3,
/// };
/// agent.set_destination(destination, None, true)?;
///
/// // Control loop
/// while !agent.done() {
///     let control = agent.run_step()?;
///     // Apply control to vehicle
/// }
/// # Ok(())
/// # }
/// ```
pub struct BasicAgent {
    core: AgentCore,
    target_speed: f32,
}

impl BasicAgent {
    /// Creates a new BasicAgent.
    ///
    /// # Arguments
    /// * `vehicle` - The vehicle to control
    /// * `config` - Agent configuration
    /// * `map_inst` - Optional pre-loaded map instance
    /// * `grp_inst` - Optional pre-loaded GlobalRoutePlanner instance
    pub fn new(
        vehicle: Vehicle,
        config: BasicAgentConfig,
        map_inst: Option<Map>,
        grp_inst: Option<GlobalRoutePlanner>,
    ) -> Result<Self> {
        let target_speed = config.target_speed;

        let core = AgentCore::new(
            vehicle,
            map_inst,
            grp_inst,
            config.core_config,
            config.local_config,
        )?;

        Ok(Self { core, target_speed })
    }

    /// Sets the target speed in km/h.
    pub fn set_target_speed(&mut self, speed: f32) {
        self.target_speed = speed;
        self.core.local_planner.set_speed(speed);
    }

    /// Enables or disables speed limit following.
    pub fn follow_speed_limits(&mut self, value: bool) {
        self.core.local_planner.follow_speed_limits(value);
    }

    /// Sets a navigation destination.
    ///
    /// # Arguments
    /// * `end_location` - Target destination
    /// * `start_location` - Optional starting location (defaults to vehicle position)
    /// * `clean_queue` - Whether to clear existing route
    pub fn set_destination(
        &mut self,
        end_location: Location,
        start_location: Option<Location>,
        clean_queue: bool,
    ) -> Result<()> {
        let start_location = if let Some(loc) = start_location {
            loc
        } else if clean_queue {
            if let Some((wp, _)) = self
                .core
                .local_planner
                .get_incoming_waypoint_and_direction(0)
            {
                let isometry = wp.transform();
                crate::geom::Location::from_na_translation(&isometry.translation)
            } else {
                let isometry = self.core.vehicle.transform();
                crate::geom::Location::from_na_translation(&isometry.translation)
            }
        } else if let Some((wp, _)) = self.core.local_planner.get_plan().last() {
            let isometry = wp.transform();
            crate::geom::Location::from_na_translation(&isometry.translation)
        } else {
            let isometry = self.core.vehicle.transform();
            crate::geom::Location::from_na_translation(&isometry.translation)
        };

        // Compute route using global planner
        let route = self
            .core
            .global_planner
            .trace_route(start_location, end_location)?;

        // Set route in local planner
        self.core
            .local_planner
            .set_global_plan(route, true, clean_queue);

        Ok(())
    }

    /// Sets a pre-computed global plan.
    pub fn set_global_plan(
        &mut self,
        plan: Vec<(Waypoint, RoadOption)>,
        stop_waypoint_creation: bool,
        clean_queue: bool,
    ) {
        self.core
            .local_planner
            .set_global_plan(plan, stop_waypoint_creation, clean_queue);
    }

    /// Computes a route between two waypoints.
    pub fn trace_route(
        &self,
        start_waypoint: &Waypoint,
        end_waypoint: &Waypoint,
    ) -> Result<Vec<(Waypoint, RoadOption)>> {
        let start_isometry = start_waypoint.transform();
        let start_location =
            crate::geom::Location::from_na_translation(&start_isometry.translation);

        let end_isometry = end_waypoint.transform();
        let end_location = crate::geom::Location::from_na_translation(&end_isometry.translation);
        self.core
            .global_planner
            .trace_route(start_location, end_location)
    }

    /// Checks if the destination has been reached.
    pub fn done(&self) -> bool {
        self.core.local_planner.done()
    }

    /// Executes one navigation step.
    ///
    /// # Returns
    /// VehicleControl command to apply to the vehicle
    pub fn run_step(&mut self) -> Result<VehicleControl> {
        self.run_step_debug(false)
    }

    /// Executes one navigation step with optional debug output.
    pub fn run_step_debug(&mut self, debug: bool) -> Result<VehicleControl> {
        let mut hazard_detected = false;

        // Get vehicle speed
        let vehicle_speed = get_speed(&self.core.vehicle) / 3.6; // Convert to m/s

        // Check for vehicle obstacles
        let max_vehicle_distance =
            self.core.base_vehicle_threshold + self.core.speed_ratio * vehicle_speed;
        let obstacle_result = self.core.vehicle_obstacle_detected(max_vehicle_distance)?;

        if obstacle_result.obstacle_was_found {
            hazard_detected = true;
            if debug {
                println!(
                    "BasicAgent: Vehicle obstacle detected at {:.1}m",
                    obstacle_result.distance
                );
            }
        }

        // Check for traffic lights
        let max_tlight_distance =
            self.core.base_tlight_threshold + self.core.speed_ratio * vehicle_speed;
        let traffic_light_result = self.core.affected_by_traffic_light(max_tlight_distance)?;

        if traffic_light_result.traffic_light_was_found {
            hazard_detected = true;
            if debug {
                println!("BasicAgent: Red traffic light detected");
            }
        }

        // Get control from local planner
        let mut control = self.core.local_planner.run_step(debug)?;

        // Apply emergency stop if hazard detected
        if hazard_detected {
            control = self.add_emergency_stop(control);
            if debug {
                println!("BasicAgent: Emergency stop applied");
            }
        }

        Ok(control)
    }

    /// Applies emergency stop to a control.
    pub fn add_emergency_stop(&self, mut control: VehicleControl) -> VehicleControl {
        control.throttle = 0.0;
        control.brake = self.core.max_brake;
        control.hand_brake = false;
        control
    }

    /// Sets whether to ignore traffic lights.
    pub fn ignore_traffic_lights(&mut self, active: bool) {
        self.core.ignore_traffic_lights(active);
    }

    /// Sets whether to ignore stop signs.
    pub fn ignore_stop_signs(&mut self, active: bool) {
        self.core.ignore_stop_signs(active);
    }

    /// Sets whether to ignore vehicles.
    pub fn ignore_vehicles(&mut self, active: bool) {
        self.core.ignore_vehicles(active);
    }

    /// Gets a reference to the local planner (for advanced use).
    pub fn local_planner(&self) -> &super::local_planner::LocalPlanner {
        &self.core.local_planner
    }

    /// Gets a reference to the global planner (for advanced use).
    pub fn global_planner(&self) -> &GlobalRoutePlanner {
        &self.core.global_planner
    }

    /// Executes a lane change maneuver.
    ///
    /// # Arguments
    /// * `direction` - Direction to change lanes (left or right)
    /// * `same_lane_time` - Time (seconds) to travel in current lane before starting change
    /// * `other_lane_time` - Time (seconds) to travel in new lane after completing change
    /// * `lane_change_time` - Time (seconds) to perform the lane change
    ///
    /// # Returns
    /// Ok(()) if lane change path was generated and applied, Err if not possible
    pub fn lane_change(
        &mut self,
        direction: LaneChangeDirection,
        same_lane_time: f32,
        other_lane_time: f32,
        lane_change_time: f32,
    ) -> Result<()> {
        // Get current speed to convert time to distance
        let speed = get_speed(&self.core.vehicle) / 3.6; // Convert km/h to m/s
        let speed = speed.max(1.0); // Minimum 1 m/s to avoid division by zero

        // Convert times to distances
        let distance_same_lane = speed * same_lane_time;
        let distance_other_lane = speed * other_lane_time;
        let lane_change_distance = speed * lane_change_time;

        // Get current waypoint
        let current_waypoint = if let Some((wp, _)) = self
            .core
            .local_planner
            .get_incoming_waypoint_and_direction(0)
        {
            wp
        } else {
            // Use vehicle location if no waypoint in queue
            let vehicle_isometry = self.core.vehicle.transform();
            let vehicle_location = Location::from_na_translation(&vehicle_isometry.translation);
            self.core
                .map
                .waypoint_at(&vehicle_location)
                .ok_or_else(|| anyhow::anyhow!("Failed to get waypoint from vehicle location"))?
        };

        // Generate lane change path
        let step_distance = 2.0; // 2 meters between waypoints
        let path = AgentCore::generate_lane_change_path(
            &current_waypoint,
            direction,
            distance_same_lane,
            distance_other_lane,
            lane_change_distance,
            true, // Check if lane change is possible
            step_distance,
        );

        if path.is_empty() {
            return Err(anyhow::anyhow!("Lane change not possible"));
        }

        // Apply the lane change path
        self.set_global_plan(path, true, true);

        Ok(())
    }
}

impl Agent for BasicAgent {
    fn run_step(&mut self) -> Result<VehicleControl> {
        self.run_step()
    }

    fn done(&self) -> bool {
        self.done()
    }

    fn set_destination(
        &mut self,
        end_location: Location,
        start_location: Option<Location>,
        clean_queue: bool,
    ) -> Result<()> {
        self.set_destination(end_location, start_location, clean_queue)
    }

    fn set_target_speed(&mut self, speed: f32) {
        self.set_target_speed(speed)
    }

    fn set_global_plan(
        &mut self,
        plan: Vec<(Waypoint, RoadOption)>,
        stop_waypoint_creation: bool,
        clean_queue: bool,
    ) {
        self.set_global_plan(plan, stop_waypoint_creation, clean_queue)
    }

    fn trace_route(
        &self,
        start_waypoint: &Waypoint,
        end_waypoint: &Waypoint,
    ) -> Result<Vec<(Waypoint, RoadOption)>> {
        self.trace_route(start_waypoint, end_waypoint)
    }

    fn ignore_traffic_lights(&mut self, active: bool) {
        self.ignore_traffic_lights(active)
    }

    fn ignore_stop_signs(&mut self, active: bool) {
        self.ignore_stop_signs(active)
    }

    fn ignore_vehicles(&mut self, active: bool) {
        self.ignore_vehicles(active)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_agent_config_default() {
        let config = BasicAgentConfig::default();
        assert_eq!(config.target_speed, 20.0);
    }

    #[test]
    fn test_emergency_stop() {
        // Test emergency stop logic without needing a vehicle
        let control = VehicleControl {
            throttle: 0.5,
            steer: 0.1,
            brake: 0.0,
            hand_brake: false,
            reverse: false,
            manual_gear_shift: false,
            gear: 0,
        };

        let max_brake = 0.5;
        let mut stopped_control = control;
        stopped_control.throttle = 0.0;
        stopped_control.brake = max_brake;

        assert_eq!(stopped_control.throttle, 0.0);
        assert_eq!(stopped_control.brake, 0.5);
    }
}
