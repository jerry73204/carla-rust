//! ConstantVelocityAgent for testing and simple constant-speed scenarios.

use super::{
    agent_core::{AgentCore, AgentCoreConfig},
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

/// Configuration options for ConstantVelocityAgent.
#[derive(Debug, Clone)]
pub struct ConstantVelocityAgentConfig {
    /// Target constant speed in m/s
    pub target_speed: f32,
    /// Whether to use basic behavior when not in constant velocity mode
    pub use_basic_behavior: bool,
    /// Agent core configuration
    pub core_config: AgentCoreConfig,
    /// Local planner configuration
    pub local_config: LocalPlannerConfig,
}

impl Default for ConstantVelocityAgentConfig {
    fn default() -> Self {
        let target_speed_mps = 5.0; // 5 m/s = 18 km/h
        Self {
            target_speed: target_speed_mps,
            use_basic_behavior: false,
            core_config: AgentCoreConfig::default(),
            local_config: LocalPlannerConfig {
                target_speed: target_speed_mps * 3.6, // Convert to km/h for planner
                ..Default::default()
            },
        }
    }
}

/// ConstantVelocityAgent maintains constant velocity for testing.
///
/// This agent is primarily used for testing scenarios where a vehicle needs
/// to maintain constant speed regardless of traffic conditions. It simplifies
/// hazard detection compared to BasicAgent and BehaviorAgent.
///
/// # Examples
/// ```no_run
/// use carla::{
///     agents::navigation::{ConstantVelocityAgent, ConstantVelocityAgentConfig},
///     client::Vehicle,
///     geom::Location,
/// };
///
/// # fn example(vehicle: Vehicle) -> anyhow::Result<()> {
/// let config = ConstantVelocityAgentConfig {
///     target_speed: 10.0, // 10 m/s
///     ..Default::default()
/// };
/// let mut agent = ConstantVelocityAgent::new(vehicle, config, None, None)?;
///
/// // Set destination
/// let destination = Location::new(100.0, 50.0, 0.3);
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
pub struct ConstantVelocityAgent {
    core: AgentCore,
    use_basic_behavior: bool,
    target_speed: f32,  // m/s
    current_speed: f32, // m/s
    is_constant_velocity_active: bool,
}

impl ConstantVelocityAgent {
    /// Creates a new ConstantVelocityAgent.
    ///
    /// # Arguments
    /// * `vehicle` - The vehicle to control
    /// * `config` - Agent configuration
    /// * `map_inst` - Optional pre-loaded map instance
    /// * `grp_inst` - Optional pre-loaded GlobalRoutePlanner instance
    pub fn new(
        vehicle: Vehicle,
        config: ConstantVelocityAgentConfig,
        map_inst: Option<Map>,
        grp_inst: Option<GlobalRoutePlanner>,
    ) -> Result<Self> {
        let target_speed = config.target_speed;
        let use_basic_behavior = config.use_basic_behavior;

        let core = AgentCore::new(
            vehicle,
            map_inst,
            grp_inst,
            config.core_config,
            config.local_config,
        )?;

        Ok(Self {
            core,
            use_basic_behavior,
            target_speed,
            current_speed: 0.0,
            is_constant_velocity_active: true,
        })
    }

    /// Sets the constant velocity in m/s.
    pub fn set_constant_velocity(&mut self, speed: f32) {
        self.target_speed = speed;
        // Convert m/s to km/h for local planner
        self.core.local_planner.set_speed(speed * 3.6);
        self.is_constant_velocity_active = true;
    }

    /// Sets the target speed (alias for set_constant_velocity with km/h input).
    pub fn set_target_speed(&mut self, speed_kmh: f32) {
        self.set_constant_velocity(speed_kmh / 3.6);
    }

    /// Enables or disables speed limit following.
    pub fn follow_speed_limits(&mut self, value: bool) {
        self.core.local_planner.follow_speed_limits(value);
    }

    /// Sets a navigation destination.
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
                wp.transform().location
            } else {
                self.core.vehicle.transform().location
            }
        } else if let Some((wp, _)) = self.core.local_planner.get_plan().last() {
            wp.transform().location
        } else {
            self.core.vehicle.transform().location
        };

        let route = self
            .core
            .global_planner
            .trace_route(start_location, end_location)?;

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
        let start_location = start_waypoint.transform().location;
        let end_location = end_waypoint.transform().location;

        self.core
            .global_planner
            .trace_route(start_location, end_location)
    }

    /// Checks if the destination has been reached.
    pub fn done(&self) -> bool {
        self.core.local_planner.done()
    }

    /// Executes one navigation step.
    pub fn run_step(&mut self) -> Result<VehicleControl> {
        self.run_step_with_debug(false)
    }

    /// Executes one navigation step with optional debug output.
    pub fn run_step_with_debug(&mut self, debug: bool) -> Result<VehicleControl> {
        // Update current speed
        self.current_speed = get_speed(&self.core.vehicle) / 3.6; // Convert to m/s

        if !self.is_constant_velocity_active && self.use_basic_behavior {
            // Use basic behavior (simplified hazard detection)
            return self.run_basic_behavior(debug);
        }

        // Constant velocity mode - simplified hazard check
        let mut hazard_speed = self.target_speed * 3.6; // Convert to km/h

        // Simple obstacle check (optional in constant velocity mode)
        if !self.core.ignore_vehicles {
            let max_distance = 10.0; // Fixed detection distance
            let result = self.core.vehicle_obstacle_detected(max_distance)?;

            if result.obstacle_was_found {
                // Slow down but maintain some speed
                hazard_speed = (self.target_speed * 0.5) * 3.6; // Half speed in km/h
                if debug {
                    println!("ConstantVelocityAgent: Obstacle detected, reducing speed");
                }
            }
        }

        self.core.local_planner.set_speed(hazard_speed);
        self.core.local_planner.run_step(debug)
    }

    /// Runs with basic behavior (similar to BasicAgent).
    fn run_basic_behavior(&mut self, debug: bool) -> Result<VehicleControl> {
        let mut hazard_detected = false;

        // Check for vehicle obstacles
        let vehicle_speed = get_speed(&self.core.vehicle) / 3.6; // m/s
        let max_vehicle_distance =
            self.core.base_vehicle_threshold + self.core.speed_ratio * vehicle_speed;
        let obstacle_result = self.core.vehicle_obstacle_detected(max_vehicle_distance)?;

        if obstacle_result.obstacle_was_found {
            hazard_detected = true;
            if debug {
                println!(
                    "ConstantVelocityAgent: Vehicle obstacle at {:.1}m",
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
                println!("ConstantVelocityAgent: Red traffic light detected");
            }
        }

        // Get control from local planner
        let mut control = self.core.local_planner.run_step(debug)?;

        // Apply emergency stop if hazard detected
        if hazard_detected {
            control.throttle = 0.0;
            control.brake = self.core.max_brake;
            if debug {
                println!("ConstantVelocityAgent: Emergency stop applied");
            }
        }

        Ok(control)
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
}

impl Agent for ConstantVelocityAgent {
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
    fn test_constant_velocity_agent_config_default() {
        let config = ConstantVelocityAgentConfig::default();
        assert_eq!(config.target_speed, 5.0); // 5 m/s
        assert!(!config.use_basic_behavior);
    }

    #[test]
    fn test_speed_conversion() {
        let speed_mps = 10.0;
        let speed_kmh = speed_mps * 3.6;
        assert_eq!(speed_kmh, 36.0);
    }
}
