//! Traffic manager specific types.

use crate::actor::ActorId;

/// Traffic manager configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficManagerConfig {
    /// Global speed difference percentage [-100.0, 100.0]
    pub global_speed_percentage_difference: f32,
    /// Global lane offset in meters
    pub global_lane_offset: f32,
    /// Global distance to leading vehicle in meters
    pub global_distance_to_leading_vehicle: f32,
    /// Enable synchronous mode
    pub synchronous_mode: bool,
    /// Synchronous mode timeout in milliseconds
    pub synchronous_mode_timeout: f64,
    /// Enable OSM mode
    pub osm_mode: bool,
    /// Enable hybrid physics mode
    pub hybrid_physics_mode: bool,
    /// Hybrid physics radius
    pub hybrid_physics_radius: f32,
    /// Random device seed
    pub random_device_seed: u64,
    /// Enable respawn dormant vehicles
    pub respawn_dormant_vehicles: bool,
    /// Respawn lower boundary
    pub respawn_lower_bound: f32,
    /// Respawn upper boundary  
    pub respawn_upper_bound: f32,
    /// Max lower boundary
    pub max_lower_bound: f32,
    /// Max upper boundary
    pub max_upper_bound: f32,
}

impl Default for TrafficManagerConfig {
    fn default() -> Self {
        Self {
            global_speed_percentage_difference: 30.0,
            global_lane_offset: 0.0,
            global_distance_to_leading_vehicle: 5.0,
            synchronous_mode: false,
            synchronous_mode_timeout: 10.0,
            osm_mode: false,
            hybrid_physics_mode: false,
            hybrid_physics_radius: 70.0,
            random_device_seed: 0,
            respawn_dormant_vehicles: false,
            respawn_lower_bound: 25.0,
            respawn_upper_bound: 50.0,
            max_lower_bound: 10.0,
            max_upper_bound: 90.0,
        }
    }
}

/// Vehicle-specific traffic manager configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficManagerVehicleConfig {
    /// Vehicle actor ID
    pub actor_id: ActorId,
    /// Speed percentage difference for this vehicle
    pub speed_percentage: f32,
    /// Desired speed for this vehicle
    pub desired_speed: f32,
    /// Lane offset for this vehicle
    pub lane_offset: f32,
    /// Auto lane change enabled for this vehicle
    pub auto_lane_change: bool,
    /// Distance to leading vehicle for this vehicle
    pub distance_to_leading_vehicle: f32,
    /// Percentage of running red lights
    pub percentage_running_light: f32,
    /// Percentage of running stop signs
    pub percentage_running_sign: f32,
    /// Percentage of ignoring walkers
    pub percentage_ignore_walkers: f32,
    /// Percentage of ignoring vehicles
    pub percentage_ignore_vehicles: f32,
    /// Keep right percentage
    pub keep_right_percentage: f32,
    /// Random left lane change percentage
    pub random_left_lane_change_percentage: f32,
    /// Random right lane change percentage
    pub random_right_lane_change_percentage: f32,
    /// Update vehicle lights
    pub update_vehicle_lights: bool,
}

impl Default for TrafficManagerVehicleConfig {
    fn default() -> Self {
        Self {
            actor_id: 0,
            speed_percentage: 30.0,
            desired_speed: 30.0,
            lane_offset: 0.0,
            auto_lane_change: true,
            distance_to_leading_vehicle: 5.0,
            percentage_running_light: 0.0,
            percentage_running_sign: 0.0,
            percentage_ignore_walkers: 0.0,
            percentage_ignore_vehicles: 0.0,
            keep_right_percentage: 70.0,
            random_left_lane_change_percentage: 10.0,
            random_right_lane_change_percentage: 10.0,
            update_vehicle_lights: false,
        }
    }
}

/// Traffic manager statistics.
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficManagerStats {
    /// Number of registered vehicles
    pub registered_vehicles_count: usize,
    /// Number of active vehicles
    pub active_vehicles_count: usize,
    /// Number of dormant vehicles
    pub dormant_vehicles_count: usize,
    /// Number of destroyed vehicles
    pub destroyed_vehicles_count: usize,
    /// Average vehicle speed
    pub average_vehicle_speed: f32,
    /// Total distance traveled by all vehicles
    pub total_distance_traveled: f64,
}

impl Default for TrafficManagerStats {
    fn default() -> Self {
        Self {
            registered_vehicles_count: 0,
            active_vehicles_count: 0,
            dormant_vehicles_count: 0,
            destroyed_vehicles_count: 0,
            average_vehicle_speed: 0.0,
            total_distance_traveled: 0.0,
        }
    }
}

/// Traffic manager action for a specific vehicle.
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficManagerAction {
    /// Vehicle actor ID
    pub actor_id: ActorId,
    /// Current action type
    pub action_type: ActionType,
    /// Target speed
    pub target_speed: f32,
    /// Target lane ID
    pub target_lane_id: i32,
    /// Distance to target
    pub distance_to_target: f32,
}

/// Types of actions that can be performed by traffic manager.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ActionType {
    /// Vehicle is following the lane
    LaneFollow,
    /// Vehicle is changing lanes to the left
    LaneChangeLeft,
    /// Vehicle is changing lanes to the right
    LaneChangeRight,
    /// Vehicle is stopping for a red light
    StopRedLight,
    /// Vehicle is stopping for a stop sign
    StopSign,
    /// Vehicle is yielding to pedestrians
    YieldPedestrian,
    /// Vehicle is yielding to other vehicles
    YieldVehicle,
    /// Vehicle is parked or dormant
    Parked,
}

// ========================================
// FFI Conversion Implementations
// ========================================

impl TrafficManagerConfig {
    /// Create from carla-cxx TrafficManagerConfig.
    pub fn from_cxx(config: carla_cxx::TrafficManagerConfig) -> Self {
        Self {
            global_speed_percentage_difference: config.global_speed_percentage_difference,
            global_lane_offset: config.global_lane_offset,
            global_distance_to_leading_vehicle: config.global_distance_to_leading_vehicle,
            synchronous_mode: config.synchronous_mode,
            synchronous_mode_timeout: config.synchronous_mode_timeout_ms,
            osm_mode: config.osm_mode,
            hybrid_physics_mode: config.hybrid_physics_mode,
            hybrid_physics_radius: config.hybrid_physics_radius,
            random_device_seed: config.random_device_seed,
            respawn_dormant_vehicles: config.respawn_dormant_vehicles,
            respawn_lower_bound: config.respawn_lower_bound,
            respawn_upper_bound: config.respawn_upper_bound,
            // These fields are not in the FFI struct, use defaults
            max_lower_bound: 10.0,
            max_upper_bound: 90.0,
        }
    }
}

impl TrafficManagerVehicleConfig {
    /// Create from carla-cxx TrafficManagerVehicleConfig.
    pub fn from_cxx(config: carla_cxx::TrafficManagerVehicleConfig) -> Self {
        Self {
            actor_id: 0, // Not available in FFI struct
            speed_percentage: config.speed_percentage_difference,
            desired_speed: config.desired_speed,
            lane_offset: config.lane_offset,
            auto_lane_change: config.auto_lane_change,
            distance_to_leading_vehicle: config.distance_to_leading_vehicle,
            percentage_running_light: config.percentage_running_light,
            percentage_running_sign: config.percentage_running_sign,
            percentage_ignore_walkers: config.percentage_ignore_walkers,
            percentage_ignore_vehicles: config.percentage_ignore_vehicles,
            keep_right_percentage: config.keep_right_percentage,
            random_left_lane_change_percentage: config.random_left_lane_change_percentage,
            random_right_lane_change_percentage: config.random_right_lane_change_percentage,
            update_vehicle_lights: config.update_vehicle_lights,
        }
    }
}

impl TrafficManagerStats {
    /// Create from carla-cxx TrafficManagerStats.
    pub fn from_cxx(stats: carla_cxx::TrafficManagerStats) -> Self {
        Self {
            registered_vehicles_count: stats.total_registered_vehicles as usize,
            active_vehicles_count: stats.active_vehicle_count as usize,
            dormant_vehicles_count: 0,   // Not available in FFI struct
            destroyed_vehicles_count: 0, // Not available in FFI struct
            average_vehicle_speed: 0.0,  // Not available in FFI struct
            total_distance_traveled: stats.total_simulation_time_seconds,
        }
    }
}

impl TrafficManagerAction {
    /// Create from carla-cxx TrafficManagerAction.
    pub fn from_cxx(action: carla_cxx::TrafficManagerAction) -> Self {
        Self {
            actor_id: 0, // Not available in FFI struct
            action_type: ActionType::from_road_option(action.road_option),
            target_speed: 0.0,       // Not available in FFI struct
            target_lane_id: 0,       // Not available in FFI struct
            distance_to_target: 0.0, // Not available in FFI struct
        }
    }
}

impl ActionType {
    /// Create from carla-cxx RoadOption.
    pub fn from_road_option(road_option: carla_cxx::RoadOption) -> Self {
        match road_option {
            carla_cxx::RoadOption::LaneFollow => ActionType::LaneFollow,
            carla_cxx::RoadOption::ChangeLaneLeft => ActionType::LaneChangeLeft,
            carla_cxx::RoadOption::ChangeLaneRight => ActionType::LaneChangeRight,
            carla_cxx::RoadOption::Left => ActionType::LaneChangeLeft,
            carla_cxx::RoadOption::Right => ActionType::LaneChangeRight,
            carla_cxx::RoadOption::Straight => ActionType::LaneFollow,
            carla_cxx::RoadOption::RoadEnd => ActionType::Parked,
            carla_cxx::RoadOption::Void => ActionType::LaneFollow,
        }
    }

    /// Create from carla-cxx action type integer.
    pub fn from_cxx(action_type: u8) -> Self {
        match action_type {
            0 => ActionType::LaneFollow,
            1 => ActionType::LaneChangeLeft,
            2 => ActionType::LaneChangeRight,
            3 => ActionType::StopRedLight,
            4 => ActionType::StopSign,
            5 => ActionType::YieldPedestrian,
            6 => ActionType::YieldVehicle,
            7 => ActionType::Parked,
            _ => ActionType::LaneFollow, // Default fallback
        }
    }
}
