//! Traffic manager specific types.

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
    /// Auto lane change enabled
    pub auto_lane_change: bool,
    /// Distance to leading vehicle for lane change
    pub distance_to_leading_vehicle_for_lane_change: f32,
    /// Distance to oncoming vehicle for lane change
    pub distance_to_oncoming_vehicle_for_lane_change: f32,
    /// Hybrid physics mode enabled
    pub hybrid_physics_mode: bool,
    /// Hybrid physics radius
    pub hybrid_physics_radius: f32,
    /// Random device seed
    pub random_device_seed: u64,
}

impl Default for TrafficManagerConfig {
    fn default() -> Self {
        Self {
            global_speed_percentage_difference: 30.0,
            global_lane_offset: 0.0,
            global_distance_to_leading_vehicle: 5.0,
            synchronous_mode: false,
            auto_lane_change: true,
            distance_to_leading_vehicle_for_lane_change: 10.0,
            distance_to_oncoming_vehicle_for_lane_change: 10.0,
            hybrid_physics_mode: false,
            hybrid_physics_radius: 70.0,
            random_device_seed: 0,
        }
    }
}
