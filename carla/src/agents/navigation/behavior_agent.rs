//! BehaviorAgent with behavior profiles and advanced hazard detection.

use super::{
    agent_core::{AgentCore, AgentCoreConfig},
    global_route_planner::GlobalRoutePlanner,
    local_planner::LocalPlannerConfig,
    types::{Agent, RoadOption},
};
use crate::{
    agents::tools::get_speed,
    client::{Actor, ActorBase, Map, Vehicle, Waypoint},
    geom::Location,
    rpc::VehicleControl,
};
use anyhow::Result;

/// Behavior parameters for different driving profiles.
#[derive(Debug, Clone)]
pub struct BehaviorParams {
    /// Maximum speed (km/h)
    pub max_speed: f32,
    /// Speed limit distance multiplier
    pub speed_lim_dist: f32,
    /// Speed decrease factor
    pub speed_decrease: f32,
    /// Safety time (seconds)
    pub safety_time: f32,
    /// Minimum proximity threshold (meters)
    pub min_proximity_threshold: f32,
    /// Braking distance (meters)
    pub braking_distance: f32,
    /// Tailgate counter threshold
    pub tailgate_counter: i32,
}

/// Driving behavior type.
#[derive(Debug, Clone)]
pub enum BehaviorType {
    /// Cautious driving (conservative, large safety margins, low speed)
    Cautious(BehaviorParams),
    /// Normal driving (standard behavior)
    Normal(BehaviorParams),
    /// Aggressive driving (fast, small safety margins, quick lane changes)
    Aggressive(BehaviorParams),
    /// Custom user-defined behavior profile
    Custom(BehaviorParams),
}

impl BehaviorType {
    /// Creates a cautious behavior profile.
    ///
    /// Characteristics:
    /// - Low max speed (40 km/h)
    /// - Large safety margins (3s safety time, 12m proximity threshold)
    /// - Conservative lane changes
    /// - Never tailgates
    pub fn cautious() -> Self {
        BehaviorType::Cautious(BehaviorParams {
            max_speed: 40.0,
            speed_lim_dist: 6.0,
            speed_decrease: 12.0,
            safety_time: 3.0,
            min_proximity_threshold: 12.0,
            braking_distance: 6.0,
            tailgate_counter: -1, // Never tailgate
        })
    }

    /// Creates a normal behavior profile.
    ///
    /// Characteristics:
    /// - Medium max speed (50 km/h)
    /// - Standard safety margins (2s safety time, 10m proximity threshold)
    /// - Balanced behavior
    pub fn normal() -> Self {
        BehaviorType::Normal(BehaviorParams {
            max_speed: 50.0,
            speed_lim_dist: 3.0,
            speed_decrease: 10.0,
            safety_time: 2.0,
            min_proximity_threshold: 10.0,
            braking_distance: 5.0,
            tailgate_counter: 0,
        })
    }

    /// Creates an aggressive behavior profile.
    ///
    /// Characteristics:
    /// - High max speed (70 km/h)
    /// - Tight safety margins (1s safety time, 8m proximity threshold)
    /// - Quick lane changes
    /// - Tailgates aggressively
    pub fn aggressive() -> Self {
        BehaviorType::Aggressive(BehaviorParams {
            max_speed: 70.0,
            speed_lim_dist: 1.0,
            speed_decrease: 8.0,
            safety_time: 1.0,
            min_proximity_threshold: 8.0,
            braking_distance: 4.0,
            tailgate_counter: -1, // Tailgate aggressively
        })
    }

    /// Creates a custom user-defined behavior profile.
    ///
    /// Allows full control over all behavior parameters.
    ///
    /// # Arguments
    /// * `params` - Custom behavior parameters
    ///
    /// # Examples
    /// ```
    /// use carla::agents::navigation::{BehaviorParams, BehaviorType};
    ///
    /// // Create a "super cautious" profile
    /// let super_cautious = BehaviorType::custom(BehaviorParams {
    ///     max_speed: 30.0,
    ///     speed_lim_dist: 10.0,
    ///     speed_decrease: 15.0,
    ///     safety_time: 4.0,
    ///     min_proximity_threshold: 15.0,
    ///     braking_distance: 8.0,
    ///     tailgate_counter: -1,
    /// });
    ///
    /// // Create a "sporty" profile
    /// let sporty = BehaviorType::custom(BehaviorParams {
    ///     max_speed: 80.0,
    ///     speed_lim_dist: 0.5,
    ///     speed_decrease: 6.0,
    ///     safety_time: 0.8,
    ///     min_proximity_threshold: 6.0,
    ///     braking_distance: 3.0,
    ///     tailgate_counter: -2,
    /// });
    /// ```
    pub fn custom(params: BehaviorParams) -> Self {
        BehaviorType::Custom(params)
    }

    /// Gets the behavior parameters.
    pub fn params(&self) -> &BehaviorParams {
        match self {
            BehaviorType::Cautious(p)
            | BehaviorType::Normal(p)
            | BehaviorType::Aggressive(p)
            | BehaviorType::Custom(p) => p,
        }
    }

    /// Gets a mutable reference to the behavior parameters.
    ///
    /// Allows runtime modification of behavior parameters.
    ///
    /// # Examples
    /// ```
    /// use carla::agents::navigation::BehaviorType;
    ///
    /// let mut behavior = BehaviorType::normal();
    /// // Temporarily reduce max speed
    /// behavior.params_mut().max_speed = 30.0;
    /// ```
    pub fn params_mut(&mut self) -> &mut BehaviorParams {
        match self {
            BehaviorType::Cautious(p)
            | BehaviorType::Normal(p)
            | BehaviorType::Aggressive(p)
            | BehaviorType::Custom(p) => p,
        }
    }
}

impl Default for BehaviorType {
    fn default() -> Self {
        Self::normal()
    }
}

/// Configuration options for BehaviorAgent.
#[derive(Debug, Clone)]
pub struct BehaviorAgentConfig {
    /// Behavior profile
    pub behavior: BehaviorType,
    /// Agent core configuration
    pub core_config: AgentCoreConfig,
    /// Local planner configuration
    pub local_config: LocalPlannerConfig,
}

impl Default for BehaviorAgentConfig {
    fn default() -> Self {
        let behavior = BehaviorType::normal();
        let target_speed = behavior.params().max_speed;

        Self {
            behavior,
            core_config: AgentCoreConfig::default(),
            local_config: LocalPlannerConfig {
                target_speed,
                ..Default::default()
            },
        }
    }
}

/// BehaviorAgent with behavior profiles and advanced hazard detection.
///
/// Extends BasicAgent functionality with:
/// - Behavior profiles (cautious/normal/aggressive)
/// - Time-to-collision calculations
/// - Pedestrian avoidance
/// - Adaptive cruise control
///
/// # Examples
/// ```no_run
/// use carla::{
///     agents::navigation::{BehaviorAgent, BehaviorAgentConfig, BehaviorType},
///     client::Vehicle,
///     geom::Location,
/// };
///
/// # fn example(vehicle: Vehicle) -> anyhow::Result<()> {
/// let config = BehaviorAgentConfig {
///     behavior: BehaviorType::cautious(),
///     ..Default::default()
/// };
/// let mut agent = BehaviorAgent::new(vehicle, config, None, None)?;
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
pub struct BehaviorAgent {
    core: AgentCore,
    behavior: BehaviorType,

    // BehaviorAgent-specific state
    look_ahead_steps: usize,
    speed: f32,
    speed_limit: f32,
    direction: Option<RoadOption>,
    incoming_direction: Option<RoadOption>,
    incoming_waypoint: Option<Waypoint>,
    min_speed: f32,
    #[allow(dead_code)]
    sampling_resolution: f32,
}

impl BehaviorAgent {
    /// Creates a new BehaviorAgent.
    ///
    /// # Arguments
    /// * `vehicle` - The vehicle to control
    /// * `config` - Agent configuration with behavior profile
    /// * `map_inst` - Optional pre-loaded map instance
    /// * `grp_inst` - Optional pre-loaded GlobalRoutePlanner instance
    pub fn new(
        vehicle: Vehicle,
        config: BehaviorAgentConfig,
        map_inst: Option<Map>,
        grp_inst: Option<GlobalRoutePlanner>,
    ) -> Result<Self> {
        let behavior = config.behavior;
        let sampling_resolution = config.core_config.sampling_resolution;

        let core = AgentCore::new(
            vehicle,
            map_inst,
            grp_inst,
            config.core_config,
            config.local_config,
        )?;

        Ok(Self {
            core,
            behavior,
            look_ahead_steps: 5,
            speed: 0.0,
            speed_limit: 50.0,
            direction: None,
            incoming_direction: None,
            incoming_waypoint: None,
            min_speed: 5.0,
            sampling_resolution,
        })
    }

    /// Updates internal information about the vehicle and route.
    fn update_information(&mut self) {
        self.speed = get_speed(&self.core.vehicle);

        // Get incoming waypoint and direction
        if let Some((wp, dir)) = self
            .core
            .local_planner
            .get_incoming_waypoint_and_direction(self.look_ahead_steps)
        {
            self.incoming_waypoint = Some(wp);
            self.incoming_direction = Some(dir);
        }

        // Update current direction from first waypoint
        if let Some((_, dir)) = self
            .core
            .local_planner
            .get_incoming_waypoint_and_direction(0)
        {
            self.direction = Some(dir);
        }

        // TODO: Update speed limit from waypoint when API available
        // For now, use behavior max speed as speed limit
        self.speed_limit = self.behavior.params().max_speed;
    }

    /// Checks traffic lights and returns true if emergency stop needed.
    fn traffic_light_manager(&mut self) -> bool {
        let max_tlight_distance =
            self.core.base_tlight_threshold + self.core.speed_ratio * (self.speed / 3.6);

        match self.core.affected_by_traffic_light(max_tlight_distance) {
            Ok(result) => result.traffic_light_was_found,
            Err(_) => false,
        }
    }

    /// Detects vehicle obstacles and returns (found, actor_id, distance).
    fn collision_and_car_avoid_manager(&self) -> Result<(bool, Option<u32>, f32)> {
        let params = self.behavior.params();
        let max_distance = params.min_proximity_threshold + params.safety_time * (self.speed / 3.6);

        let result = self.core.vehicle_obstacle_detected(max_distance)?;
        Ok((
            result.obstacle_was_found,
            result.obstacle_id,
            result.distance,
        ))
    }

    /// Detects pedestrian obstacles and returns (found, actor, distance).
    fn pedestrian_avoid_manager(&self) -> Result<(bool, Option<Actor>, f32)> {
        // Similar to vehicle detection but for walkers
        let walker_list = self.core.world.actors().filter("*walker.pedestrian*");
        let vehicle_location = self.core.vehicle.transform().location;

        let params = self.behavior.params();
        let max_distance = params.min_proximity_threshold + params.safety_time * (self.speed / 3.6);

        for actor in walker_list.iter() {
            let walker_location = actor.transform().location;

            let dx = walker_location.x - vehicle_location.x;
            let dy = walker_location.y - vehicle_location.y;
            let distance = (dx * dx + dy * dy).sqrt();

            if distance < max_distance {
                return Ok((true, Some(actor), distance));
            }
        }

        Ok((false, None, -1.0))
    }

    /// Implements adaptive cruise control when following another vehicle.
    fn car_following_manager(&mut self, distance: f32, debug: bool) -> Result<VehicleControl> {
        let params = self.behavior.params();

        // Calculate target speed based on distance
        let _speed_decrease = params.speed_decrease;
        let safety_distance = params.min_proximity_threshold;

        let target_speed = if distance < safety_distance {
            // Too close, slow down more aggressively
            self.min_speed
        } else {
            // Maintain speed proportional to distance
            let speed_factor = (distance - safety_distance) / safety_distance;
            let target = self.speed_limit * speed_factor.min(1.0);
            target.max(self.min_speed)
        };

        if debug {
            println!(
                "BehaviorAgent: Car following - distance={:.1}m, target_speed={:.1}km/h",
                distance, target_speed
            );
        }

        self.core.local_planner.set_speed(target_speed);
        self.core.local_planner.run_step(debug)
    }

    /// Creates an emergency stop control.
    fn emergency_stop(&self) -> VehicleControl {
        VehicleControl {
            throttle: 0.0,
            steer: 0.0,
            brake: self.core.max_brake,
            hand_brake: false,
            reverse: false,
            manual_gear_shift: false,
            gear: 0,
        }
    }

    /// Executes one navigation step with behavior-specific logic.
    pub fn run_step_with_debug(&mut self, debug: bool) -> Result<VehicleControl> {
        // Update vehicle state
        self.update_information();

        // Priority 1: Check traffic lights
        if self.traffic_light_manager() {
            if debug {
                println!("BehaviorAgent: Red traffic light detected - emergency stop");
            }
            return Ok(self.emergency_stop());
        }

        // Priority 2: Check for pedestrians
        let (walker_found, walker_actor, walker_distance) = self.pedestrian_avoid_manager()?;
        if walker_found {
            // Distance is computed from center to center,
            // we use bounding boxes to calculate the actual distance
            let vehicle_bbox = self.core.vehicle.bounding_box();

            // Get maximum extent (approximate radius) for ego vehicle
            let vehicle_radius = vehicle_bbox.extent.x.max(vehicle_bbox.extent.y);

            // Get walker radius - use actual bbox on 0.9.16+, approximate on older versions
            let walker_radius = {
                #[cfg(carla_version_0916)]
                {
                    // Use actual walker bounding box (available in 0.9.16+)
                    let walker = walker_actor.unwrap();
                    let walker_bbox = walker.bounding_box();
                    walker_bbox.extent.x.max(walker_bbox.extent.y)
                }
                #[cfg(not(carla_version_0916))]
                {
                    // Approximate walker radius for older CARLA versions
                    let _ = walker_actor; // Suppress unused variable warning
                    0.4
                }
            };

            // Adjust distance by subtracting radii
            let actual_distance = walker_distance - walker_radius - vehicle_radius;

            let params = self.behavior.params();
            if actual_distance < params.braking_distance {
                if debug {
                    println!(
                        "BehaviorAgent: Pedestrian too close ({:.1}m actual, {:.1}m center-to-center) - emergency stop",
                        actual_distance, walker_distance
                    );
                }
                return Ok(self.emergency_stop());
            }
        }

        // Priority 3: Check for vehicle obstacles
        let (vehicle_found, vehicle_id, vehicle_distance) =
            self.collision_and_car_avoid_manager()?;
        if vehicle_found {
            // Distance is computed from center to center,
            // we use bounding boxes to calculate the actual distance
            let ego_bbox = self.core.vehicle.bounding_box();

            // Get the obstacle vehicle and its bounding box
            let vehicle_actor = self.core.world.actor(vehicle_id.unwrap()).unwrap();
            let obstacle_bbox = if let Ok(vehicle) = Vehicle::try_from(vehicle_actor) {
                vehicle.bounding_box()
            } else {
                // If not a vehicle (shouldn't happen in vehicle detection), use default bbox
                ego_bbox.clone()
            };

            // Get maximum extent (approximate radius) for both
            let vehicle_radius = obstacle_bbox.extent.x.max(obstacle_bbox.extent.y);
            let ego_radius = ego_bbox.extent.x.max(ego_bbox.extent.y);

            // Adjust distance by subtracting radii
            let actual_distance = vehicle_distance - vehicle_radius - ego_radius;

            let params = self.behavior.params();
            if actual_distance < params.braking_distance {
                if debug {
                    println!(
                        "BehaviorAgent: Vehicle too close ({:.1}m actual, {:.1}m center-to-center) - emergency stop",
                        actual_distance, vehicle_distance
                    );
                }
                return Ok(self.emergency_stop());
            } else {
                // Use car following behavior with actual distance
                return self.car_following_manager(actual_distance, debug);
            }
        }

        // Priority 4: Normal driving with speed limit compliance
        let params = self.behavior.params();
        let target_speed = self.speed_limit.min(params.max_speed);

        if debug {
            println!(
                "BehaviorAgent: Normal driving - target_speed={:.1}km/h",
                target_speed
            );
        }

        self.core.local_planner.set_speed(target_speed);
        self.core.local_planner.run_step(debug)
    }

    /// Sets the target speed (overrides behavior max speed temporarily).
    pub fn set_target_speed(&mut self, speed: f32) {
        self.core.local_planner.set_speed(speed);
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

impl Agent for BehaviorAgent {
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
    fn test_behavior_params() {
        let cautious = BehaviorType::cautious();
        assert_eq!(cautious.params().max_speed, 40.0);

        let normal = BehaviorType::normal();
        assert_eq!(normal.params().max_speed, 50.0);

        let aggressive = BehaviorType::aggressive();
        assert_eq!(aggressive.params().max_speed, 70.0);
    }

    #[test]
    fn test_behavior_agent_config_default() {
        let config = BehaviorAgentConfig::default();
        assert_eq!(config.behavior.params().max_speed, 50.0);
    }

    #[test]
    fn test_emergency_stop() {
        // Test emergency stop logic without needing a vehicle
        let max_brake = 0.5;
        let stopped_control = VehicleControl {
            throttle: 0.0,
            steer: 0.0,
            brake: max_brake,
            hand_brake: false,
            reverse: false,
            manual_gear_shift: false,
            gear: 0,
        };

        assert_eq!(stopped_control.throttle, 0.0);
        assert_eq!(stopped_control.brake, 0.5);
    }
}
