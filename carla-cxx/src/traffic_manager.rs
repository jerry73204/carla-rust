//! Traffic management functionality for CARLA simulator.
//!
//! This module provides traffic manager capabilities for controlling
//! AI vehicle behavior in the simulation.

use crate::ffi::{
    bridge, Client, SimpleTrafficManagerAction, SimpleTrafficManagerConfig,
    SimpleTrafficManagerStats, SimpleTrafficManagerVehicleConfig, TrafficManager, Vehicle,
};
use cxx::SharedPtr;

/// High-level wrapper for CARLA Traffic Manager
pub struct TrafficManagerWrapper {
    inner: SharedPtr<TrafficManager>,
    port: u16,
}

impl TrafficManagerWrapper {
    /// Create a new TrafficManagerWrapper from a SharedPtr<TrafficManager>
    pub fn new(tm: SharedPtr<TrafficManager>, port: u16) -> Self {
        Self { inner: tm, port }
    }

    /// Get traffic manager instance for specified port
    pub fn get_instance(client: &Client, port: u16) -> Self {
        let tm = bridge::TrafficManager_GetInstance(client, port);
        Self::new(tm, port)
    }

    /// Get default traffic manager instance (port 8000)
    pub fn get_default(client: &Client) -> Self {
        Self::get_instance(client, 8000)
    }

    /// Get the port this traffic manager is running on
    pub fn get_port(&self) -> u16 {
        self.port
    }

    // === Vehicle Registration ===

    /// Register vehicles with the traffic manager
    pub fn register_vehicles(&self, vehicles: &[&Vehicle]) {
        let vehicle_ptrs: Vec<*const Vehicle> =
            vehicles.iter().map(|v| *v as *const Vehicle).collect();
        bridge::TrafficManager_RegisterVehicles(&self.inner, &vehicle_ptrs);
    }

    /// Register a single vehicle with the traffic manager
    pub fn register_vehicle(&self, vehicle: &Vehicle) {
        self.register_vehicles(&[vehicle]);
    }

    /// Unregister vehicles from the traffic manager
    pub fn unregister_vehicles(&self, vehicles: &[&Vehicle]) {
        let vehicle_ptrs: Vec<*const Vehicle> =
            vehicles.iter().map(|v| *v as *const Vehicle).collect();
        bridge::TrafficManager_UnregisterVehicles(&self.inner, &vehicle_ptrs);
    }

    /// Unregister a single vehicle from the traffic manager
    pub fn unregister_vehicle(&self, vehicle: &Vehicle) {
        self.unregister_vehicles(&[vehicle]);
    }

    /// Check if a vehicle is registered with the traffic manager
    pub fn is_vehicle_registered(&self, vehicle: &Vehicle) -> bool {
        bridge::TrafficManager_IsVehicleRegistered(&self.inner, vehicle)
    }

    // === Synchronous Mode ===

    /// Set synchronous mode for the traffic manager
    pub fn set_synchronous_mode(&self, mode: bool) {
        bridge::TrafficManager_SetSynchronousMode(&self.inner, mode);
    }

    /// Perform a synchronous tick and return whether successful
    pub fn synchronous_tick(&self) -> bool {
        bridge::TrafficManager_SynchronousTick(&self.inner)
    }

    /// Set timeout for synchronous operations in milliseconds
    pub fn set_synchronous_mode_timeout(&self, timeout_ms: f64) {
        bridge::TrafficManager_SetSynchronousModeTimeout(&self.inner, timeout_ms);
    }

    // === Global Configuration ===

    /// Set global speed percentage difference (-100 to 100%)
    /// Positive values decrease speed, negative values increase speed
    pub fn set_global_speed_percentage(&self, percentage: f32) {
        bridge::TrafficManager_SetGlobalSpeedPercentage(&self.inner, percentage);
    }

    /// Set global lane offset for all vehicles (positive = right, negative = left)
    pub fn set_global_lane_offset(&self, offset: f32) {
        bridge::TrafficManager_SetGlobalLaneOffset(&self.inner, offset);
    }

    /// Set global distance to leading vehicle for all vehicles
    pub fn set_global_distance_to_leading_vehicle(&self, distance: f32) {
        bridge::TrafficManager_SetGlobalDistanceToLeadingVehicle(&self.inner, distance);
    }

    /// Set random device seed for reproducible behavior
    pub fn set_random_device_seed(&self, seed: u64) {
        bridge::TrafficManager_SetRandomDeviceSeed(&self.inner, seed);
    }

    /// Enable/disable OpenStreetMap mode
    pub fn set_osm_mode(&self, mode: bool) {
        bridge::TrafficManager_SetOSMMode(&self.inner, mode);
    }

    // === Vehicle-Specific Speed Control ===

    /// Set speed percentage difference for specific vehicle (-100 to 100%)
    /// Positive values decrease speed, negative values increase speed
    pub fn set_vehicle_speed_percentage(&self, vehicle: &Vehicle, percentage: f32) {
        bridge::TrafficManager_SetVehicleSpeedPercentage(&self.inner, vehicle, percentage);
    }

    /// Set desired speed for specific vehicle in m/s
    pub fn set_vehicle_desired_speed(&self, vehicle: &Vehicle, speed: f32) {
        bridge::TrafficManager_SetVehicleDesiredSpeed(&self.inner, vehicle, speed);
    }

    // === Vehicle-Specific Lane Behavior ===

    /// Set lane offset for specific vehicle (positive = right, negative = left)
    pub fn set_vehicle_lane_offset(&self, vehicle: &Vehicle, offset: f32) {
        bridge::TrafficManager_SetVehicleLaneOffset(&self.inner, vehicle, offset);
    }

    /// Enable/disable automatic lane changes for vehicle
    pub fn set_vehicle_auto_lane_change(&self, vehicle: &Vehicle, enable: bool) {
        bridge::TrafficManager_SetVehicleAutoLaneChange(&self.inner, vehicle, enable);
    }

    /// Force lane change for vehicle (true = left, false = right)
    pub fn force_vehicle_lane_change(&self, vehicle: &Vehicle, direction: bool) {
        bridge::TrafficManager_ForceVehicleLaneChange(&self.inner, vehicle, direction);
    }

    /// Set distance to leading vehicle for specific vehicle
    pub fn set_vehicle_distance_to_leading_vehicle(&self, vehicle: &Vehicle, distance: f32) {
        bridge::TrafficManager_SetVehicleDistanceToLeadingVehicle(&self.inner, vehicle, distance);
    }

    // === Traffic Rule Compliance ===

    /// Set percentage of running red lights (0-100%)
    pub fn set_vehicle_percentage_running_light(&self, vehicle: &Vehicle, percentage: f32) {
        bridge::TrafficManager_SetVehiclePercentageRunningLight(&self.inner, vehicle, percentage);
    }

    /// Set percentage of ignoring stop signs (0-100%)
    pub fn set_vehicle_percentage_running_sign(&self, vehicle: &Vehicle, percentage: f32) {
        bridge::TrafficManager_SetVehiclePercentageRunningSign(&self.inner, vehicle, percentage);
    }

    /// Set percentage of ignoring pedestrians (0-100%)
    pub fn set_vehicle_percentage_ignore_walkers(&self, vehicle: &Vehicle, percentage: f32) {
        bridge::TrafficManager_SetVehiclePercentageIgnoreWalkers(&self.inner, vehicle, percentage);
    }

    /// Set percentage of ignoring other vehicles (0-100%)
    pub fn set_vehicle_percentage_ignore_vehicles(&self, vehicle: &Vehicle, percentage: f32) {
        bridge::TrafficManager_SetVehiclePercentageIgnoreVehicles(&self.inner, vehicle, percentage);
    }

    // === Advanced Features ===

    /// Enable/disable hybrid physics mode for performance optimization
    pub fn set_hybrid_physics_mode(&self, mode: bool) {
        bridge::TrafficManager_SetHybridPhysicsMode(&self.inner, mode);
    }

    /// Set hybrid physics radius
    pub fn set_hybrid_physics_radius(&self, radius: f32) {
        bridge::TrafficManager_SetHybridPhysicsRadius(&self.inner, radius);
    }

    /// Enable/disable collision detection between two vehicles
    pub fn set_collision_detection(&self, vehicle1: &Vehicle, vehicle2: &Vehicle, detect: bool) {
        bridge::TrafficManager_SetCollisionDetection(&self.inner, vehicle1, vehicle2, detect);
    }

    /// Enable/disable automatic vehicle light updates
    pub fn set_vehicle_update_lights(&self, vehicle: &Vehicle, update: bool) {
        bridge::TrafficManager_SetVehicleUpdateLights(&self.inner, vehicle, update);
    }

    // === Lane Behavior Percentages ===

    /// Set keep right percentage for vehicle (0-100%)
    pub fn set_vehicle_keep_right_percentage(&self, vehicle: &Vehicle, percentage: f32) {
        bridge::TrafficManager_SetVehicleKeepRightPercentage(&self.inner, vehicle, percentage);
    }

    /// Set random left lane change percentage (0-100%)
    pub fn set_vehicle_random_left_lane_change_percentage(
        &self,
        vehicle: &Vehicle,
        percentage: f32,
    ) {
        bridge::TrafficManager_SetVehicleRandomLeftLaneChangePercentage(
            &self.inner,
            vehicle,
            percentage,
        );
    }

    /// Set random right lane change percentage (0-100%)
    pub fn set_vehicle_random_right_lane_change_percentage(
        &self,
        vehicle: &Vehicle,
        percentage: f32,
    ) {
        bridge::TrafficManager_SetVehicleRandomRightLaneChangePercentage(
            &self.inner,
            vehicle,
            percentage,
        );
    }

    // === Respawn Configuration ===

    /// Enable/disable respawning of dormant vehicles
    pub fn set_respawn_dormant_vehicles(&self, enable: bool) {
        bridge::TrafficManager_SetRespawnDormantVehicles(&self.inner, enable);
    }

    /// Set boundaries for respawning dormant vehicles
    pub fn set_respawn_boundaries(&self, lower_bound: f32, upper_bound: f32) {
        bridge::TrafficManager_SetRespawnBoundaries(&self.inner, lower_bound, upper_bound);
    }

    /// Set maximum boundaries for traffic manager operation
    pub fn set_max_boundaries(&self, lower: f32, upper: f32) {
        bridge::TrafficManager_SetMaxBoundaries(&self.inner, lower, upper);
    }

    // === Statistics and Monitoring ===

    /// Get current traffic manager configuration
    pub fn get_config(&self) -> TrafficManagerConfig {
        let config = bridge::TrafficManager_GetConfig(&self.inner);
        TrafficManagerConfig::from_simple(config)
    }

    /// Get vehicle-specific configuration
    pub fn get_vehicle_config(&self, vehicle: &Vehicle) -> TrafficManagerVehicleConfig {
        let config = bridge::TrafficManager_GetVehicleConfig(&self.inner, vehicle);
        TrafficManagerVehicleConfig::from_simple(config)
    }

    /// Get traffic manager statistics
    pub fn get_stats(&self) -> TrafficManagerStats {
        let stats = bridge::TrafficManager_GetStats(&self.inner);
        TrafficManagerStats::from_simple(stats)
    }

    /// Get next action for vehicle
    pub fn get_next_action(&self, vehicle: &Vehicle) -> TrafficManagerAction {
        let action = bridge::TrafficManager_GetNextAction(&self.inner, vehicle);
        TrafficManagerAction::from_simple(action)
    }

    // === Lifecycle Management ===

    /// Shutdown this traffic manager instance
    pub fn shutdown(&self) {
        bridge::TrafficManager_Shutdown(&self.inner);
    }

    /// Reset all traffic manager instances to default state
    pub fn reset() {
        bridge::TrafficManager_Reset();
    }

    /// Release all traffic manager instances
    pub fn release() {
        bridge::TrafficManager_Release();
    }
}

/// Traffic manager global configuration
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficManagerConfig {
    pub global_speed_percentage_difference: f32,
    pub global_lane_offset: f32,
    pub global_distance_to_leading_vehicle: f32,
    pub synchronous_mode: bool,
    pub synchronous_mode_timeout_ms: f64,
    pub hybrid_physics_mode: bool,
    pub hybrid_physics_radius: f32,
    pub respawn_dormant_vehicles: bool,
    pub respawn_lower_bound: f32,
    pub respawn_upper_bound: f32,
    pub random_device_seed: u64,
    pub osm_mode: bool,
    pub port: u16,
}

impl TrafficManagerConfig {
    fn from_simple(simple: SimpleTrafficManagerConfig) -> Self {
        Self {
            global_speed_percentage_difference: simple.global_speed_percentage_difference,
            global_lane_offset: simple.global_lane_offset,
            global_distance_to_leading_vehicle: simple.global_distance_to_leading_vehicle,
            synchronous_mode: simple.synchronous_mode,
            synchronous_mode_timeout_ms: simple.synchronous_mode_timeout_ms,
            hybrid_physics_mode: simple.hybrid_physics_mode,
            hybrid_physics_radius: simple.hybrid_physics_radius,
            respawn_dormant_vehicles: simple.respawn_dormant_vehicles,
            respawn_lower_bound: simple.respawn_lower_bound,
            respawn_upper_bound: simple.respawn_upper_bound,
            random_device_seed: simple.random_device_seed,
            osm_mode: simple.osm_mode,
            port: simple.port,
        }
    }
}

/// Traffic manager vehicle-specific configuration
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficManagerVehicleConfig {
    pub speed_percentage_difference: f32,
    pub desired_speed: f32,
    pub lane_offset: f32,
    pub distance_to_leading_vehicle: f32,
    pub auto_lane_change: bool,
    pub force_lane_change_direction: bool,
    pub force_lane_change_active: bool,
    pub keep_right_percentage: f32,
    pub random_left_lane_change_percentage: f32,
    pub random_right_lane_change_percentage: f32,
    pub percentage_running_light: f32,
    pub percentage_running_sign: f32,
    pub percentage_ignore_walkers: f32,
    pub percentage_ignore_vehicles: f32,
    pub update_vehicle_lights: bool,
    pub collision_detection_enabled: bool,
}

impl TrafficManagerVehicleConfig {
    fn from_simple(simple: SimpleTrafficManagerVehicleConfig) -> Self {
        Self {
            speed_percentage_difference: simple.speed_percentage_difference,
            desired_speed: simple.desired_speed,
            lane_offset: simple.lane_offset,
            distance_to_leading_vehicle: simple.distance_to_leading_vehicle,
            auto_lane_change: simple.auto_lane_change,
            force_lane_change_direction: simple.force_lane_change_direction,
            force_lane_change_active: simple.force_lane_change_active,
            keep_right_percentage: simple.keep_right_percentage,
            random_left_lane_change_percentage: simple.random_left_lane_change_percentage,
            random_right_lane_change_percentage: simple.random_right_lane_change_percentage,
            percentage_running_light: simple.percentage_running_light,
            percentage_running_sign: simple.percentage_running_sign,
            percentage_ignore_walkers: simple.percentage_ignore_walkers,
            percentage_ignore_vehicles: simple.percentage_ignore_vehicles,
            update_vehicle_lights: simple.update_vehicle_lights,
            collision_detection_enabled: simple.collision_detection_enabled,
        }
    }
}

/// Traffic manager statistics
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficManagerStats {
    pub total_registered_vehicles: u32,
    pub active_vehicle_count: u32,
    pub total_ticks: u32,
    pub average_tick_time_ms: f64,
    pub collision_count: u32,
    pub lane_change_count: u32,
    pub traffic_light_violations: u32,
    pub stop_sign_violations: u32,
    pub total_simulation_time_seconds: f64,
}

impl TrafficManagerStats {
    fn from_simple(simple: SimpleTrafficManagerStats) -> Self {
        Self {
            total_registered_vehicles: simple.total_registered_vehicles,
            active_vehicle_count: simple.active_vehicle_count,
            total_ticks: simple.total_ticks,
            average_tick_time_ms: simple.average_tick_time_ms,
            collision_count: simple.collision_count,
            lane_change_count: simple.lane_change_count,
            traffic_light_violations: simple.traffic_light_violations,
            stop_sign_violations: simple.stop_sign_violations,
            total_simulation_time_seconds: simple.total_simulation_time_seconds,
        }
    }
}

/// Road options for navigation decisions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum RoadOption {
    Void = 0,
    Left = 1,
    Right = 2,
    Straight = 3,
    LaneFollow = 4,
    ChangeLaneLeft = 5,
    ChangeLaneRight = 6,
    RoadEnd = 7,
}

impl RoadOption {
    fn from_u32(value: u32) -> Self {
        match value {
            0 => RoadOption::Void,
            1 => RoadOption::Left,
            2 => RoadOption::Right,
            3 => RoadOption::Straight,
            4 => RoadOption::LaneFollow,
            5 => RoadOption::ChangeLaneLeft,
            6 => RoadOption::ChangeLaneRight,
            7 => RoadOption::RoadEnd,
            _ => RoadOption::Void,
        }
    }
}

/// Traffic manager action (road option + waypoint)
#[derive(Debug, Clone, PartialEq)]
pub struct TrafficManagerAction {
    pub road_option: RoadOption,
    pub waypoint_id: u64, // 0 if no waypoint
}

impl TrafficManagerAction {
    fn from_simple(simple: SimpleTrafficManagerAction) -> Self {
        Self {
            road_option: RoadOption::from_u32(simple.road_option),
            waypoint_id: simple.waypoint_id,
        }
    }
}
