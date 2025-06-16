//! Traffic manager implementation.

use super::{
    TrafficManagerAction, TrafficManagerConfig, TrafficManagerStats, TrafficManagerVehicleConfig,
};
use crate::{actor::Vehicle, client::Client, error::CarlaResult};
use carla_cxx::TrafficManagerWrapper;

/// Traffic manager for controlling autonomous vehicle behavior.
#[derive(Debug)]
pub struct TrafficManager {
    /// Internal wrapper for FFI calls
    pub(crate) inner: TrafficManagerWrapper,
    /// Port this traffic manager is running on
    port: u16,
}

impl TrafficManager {
    /// Get or create a traffic manager instance on the specified port.
    pub fn get_instance(client: &Client, port: u16) -> CarlaResult<Self> {
        let inner = TrafficManagerWrapper::get_instance(client.inner().get_client(), port);
        Ok(Self { inner, port })
    }

    /// Get the port this traffic manager is running on.
    pub fn get_port(&self) -> u16 {
        self.inner.get_port()
    }

    // ========================================
    // Vehicle Registration
    // ========================================

    /// Register vehicles with this traffic manager.
    pub fn register_vehicles(&self, vehicles: &[&Vehicle]) -> CarlaResult<()> {
        let ffi_vehicles: Vec<&carla_cxx::ffi::Vehicle> = vehicles
            .iter()
            .map(|v| v.inner().inner().as_ref().unwrap())
            .collect();
        self.inner.register_vehicles(&ffi_vehicles);
        Ok(())
    }

    /// Register a single vehicle with this traffic manager.
    pub fn register_vehicle(&self, vehicle: &Vehicle) -> CarlaResult<()> {
        self.register_vehicles(&[vehicle])
    }

    /// Unregister vehicles from this traffic manager.
    pub fn unregister_vehicles(&self, vehicles: &[&Vehicle]) -> CarlaResult<()> {
        let ffi_vehicles: Vec<&carla_cxx::ffi::Vehicle> = vehicles
            .iter()
            .map(|v| v.inner().inner().as_ref().unwrap())
            .collect();
        self.inner.unregister_vehicles(&ffi_vehicles);
        Ok(())
    }

    /// Unregister a single vehicle from this traffic manager.
    pub fn unregister_vehicle(&self, vehicle: &Vehicle) -> CarlaResult<()> {
        self.unregister_vehicles(&[vehicle])
    }

    /// Check if a vehicle is registered with this traffic manager.
    pub fn is_vehicle_registered(&self, vehicle: &Vehicle) -> bool {
        self.inner
            .is_vehicle_registered(vehicle.inner().inner().as_ref().unwrap())
    }

    // ========================================
    // Synchronous Mode
    // ========================================

    /// Enable/disable synchronous mode.
    pub fn set_synchronous_mode(&self, enabled: bool) -> CarlaResult<()> {
        self.inner.set_synchronous_mode(enabled);
        Ok(())
    }

    /// Perform a synchronous tick (when in synchronous mode).
    /// Returns true if the tick was successful.
    pub fn synchronous_tick(&self) -> CarlaResult<bool> {
        let result = self.inner.synchronous_tick();
        Ok(result)
    }

    /// Set synchronous mode timeout in milliseconds.
    pub fn set_synchronous_mode_timeout(&self, timeout_ms: f64) -> CarlaResult<()> {
        self.inner.set_synchronous_mode_timeout(timeout_ms);
        Ok(())
    }

    // ========================================
    // Global Configuration
    // ========================================

    /// Set global speed percentage for all registered vehicles.
    /// Percentage relative to the speed limit (e.g., 50.0 means 50% of speed limit).
    pub fn set_global_speed_percentage(&self, percentage: f32) -> CarlaResult<()> {
        self.inner.set_global_speed_percentage(percentage);
        Ok(())
    }

    /// Set global lane offset for all registered vehicles.
    /// Positive values move vehicles to the right, negative to the left.
    pub fn set_global_lane_offset(&self, offset: f32) -> CarlaResult<()> {
        self.inner.set_global_lane_offset(offset);
        Ok(())
    }

    /// Set global distance to leading vehicle for all registered vehicles.
    pub fn set_global_distance_to_leading_vehicle(&self, distance: f32) -> CarlaResult<()> {
        self.inner.set_global_distance_to_leading_vehicle(distance);
        Ok(())
    }

    /// Set random device seed for reproducible behavior.
    pub fn set_random_device_seed(&self, seed: u64) -> CarlaResult<()> {
        self.inner.set_random_device_seed(seed);
        Ok(())
    }

    /// Enable/disable OSM (OpenStreetMap) mode.
    pub fn set_osm_mode(&self, enabled: bool) -> CarlaResult<()> {
        self.inner.set_osm_mode(enabled);
        Ok(())
    }

    // ========================================
    // Vehicle-Specific Configuration
    // ========================================

    /// Set speed percentage for a specific vehicle.
    pub fn set_vehicle_speed_percentage(
        &self,
        vehicle: &Vehicle,
        percentage: f32,
    ) -> CarlaResult<()> {
        self.inner
            .set_vehicle_speed_percentage(vehicle.inner().inner(), percentage);
        Ok(())
    }

    /// Set desired speed for a specific vehicle.
    pub fn set_vehicle_desired_speed(&self, vehicle: &Vehicle, speed: f32) -> CarlaResult<()> {
        self.inner
            .set_vehicle_desired_speed(vehicle.inner().inner(), speed);
        Ok(())
    }

    /// Set lane offset for a specific vehicle.
    pub fn set_vehicle_lane_offset(&self, vehicle: &Vehicle, offset: f32) -> CarlaResult<()> {
        self.inner
            .set_vehicle_lane_offset(vehicle.inner().inner(), offset);
        Ok(())
    }

    /// Enable/disable auto lane change for a specific vehicle.
    pub fn set_vehicle_auto_lane_change(
        &self,
        vehicle: &Vehicle,
        enabled: bool,
    ) -> CarlaResult<()> {
        self.inner
            .set_vehicle_auto_lane_change(vehicle.inner().inner(), enabled);
        Ok(())
    }

    /// Force a vehicle to change lanes.
    /// direction: true for left, false for right.
    pub fn force_vehicle_lane_change(&self, vehicle: &Vehicle, direction: bool) -> CarlaResult<()> {
        self.inner
            .force_vehicle_lane_change(vehicle.inner().inner(), direction);
        Ok(())
    }

    /// Set distance to leading vehicle for a specific vehicle.
    pub fn set_vehicle_distance_to_leading_vehicle(
        &self,
        vehicle: &Vehicle,
        distance: f32,
    ) -> CarlaResult<()> {
        self.inner
            .set_vehicle_distance_to_leading_vehicle(vehicle.inner().inner(), distance);
        Ok(())
    }

    // ========================================
    // Traffic Rule Compliance
    // ========================================

    /// Set percentage of running red lights for a specific vehicle.
    pub fn set_vehicle_percentage_running_light(
        &self,
        vehicle: &Vehicle,
        percentage: f32,
    ) -> CarlaResult<()> {
        self.inner
            .set_vehicle_percentage_running_light(vehicle.inner().inner(), percentage);
        Ok(())
    }

    /// Set percentage of running stop signs for a specific vehicle.
    pub fn set_vehicle_percentage_running_sign(
        &self,
        vehicle: &Vehicle,
        percentage: f32,
    ) -> CarlaResult<()> {
        self.inner
            .set_vehicle_percentage_running_sign(vehicle.inner().inner(), percentage);
        Ok(())
    }

    /// Set percentage of ignoring walkers for a specific vehicle.
    pub fn set_vehicle_percentage_ignore_walkers(
        &self,
        vehicle: &Vehicle,
        percentage: f32,
    ) -> CarlaResult<()> {
        self.inner
            .set_vehicle_percentage_ignore_walkers(vehicle.inner().inner(), percentage);
        Ok(())
    }

    /// Set percentage of ignoring other vehicles for a specific vehicle.
    pub fn set_vehicle_percentage_ignore_vehicles(
        &self,
        vehicle: &Vehicle,
        percentage: f32,
    ) -> CarlaResult<()> {
        self.inner
            .set_vehicle_percentage_ignore_vehicles(vehicle.inner().inner(), percentage);
        Ok(())
    }

    // ========================================
    // Advanced Features
    // ========================================

    /// Enable/disable hybrid physics mode.
    pub fn set_hybrid_physics_mode(&self, enabled: bool) -> CarlaResult<()> {
        self.inner.set_hybrid_physics_mode(enabled);
        Ok(())
    }

    /// Set hybrid physics radius.
    pub fn set_hybrid_physics_radius(&self, radius: f32) -> CarlaResult<()> {
        self.inner.set_hybrid_physics_radius(radius);
        Ok(())
    }

    /// Set collision detection between two vehicles.
    pub fn set_collision_detection(
        &self,
        vehicle1: &Vehicle,
        vehicle2: &Vehicle,
        detect: bool,
    ) -> CarlaResult<()> {
        self.inner.set_collision_detection(
            vehicle1.inner().inner(),
            vehicle2.inner().inner(),
            detect,
        );
        Ok(())
    }

    /// Enable/disable vehicle light updates for a specific vehicle.
    pub fn set_vehicle_update_lights(&self, vehicle: &Vehicle, update: bool) -> CarlaResult<()> {
        self.inner
            .set_vehicle_update_lights(vehicle.inner().inner(), update);
        Ok(())
    }

    // ========================================
    // Lane Behavior Percentages
    // ========================================

    /// Set keep right percentage for a specific vehicle.
    pub fn set_vehicle_keep_right_percentage(
        &self,
        vehicle: &Vehicle,
        percentage: f32,
    ) -> CarlaResult<()> {
        self.inner
            .set_vehicle_keep_right_percentage(vehicle.inner().inner(), percentage);
        Ok(())
    }

    /// Set random left lane change percentage for a specific vehicle.
    pub fn set_vehicle_random_left_lane_change_percentage(
        &self,
        vehicle: &Vehicle,
        percentage: f32,
    ) -> CarlaResult<()> {
        self.inner
            .set_vehicle_random_left_lane_change_percentage(vehicle.inner().inner(), percentage);
        Ok(())
    }

    /// Set random right lane change percentage for a specific vehicle.
    pub fn set_vehicle_random_right_lane_change_percentage(
        &self,
        vehicle: &Vehicle,
        percentage: f32,
    ) -> CarlaResult<()> {
        self.inner
            .set_vehicle_random_right_lane_change_percentage(vehicle.inner().inner(), percentage);
        Ok(())
    }

    // ========================================
    // Respawn Configuration
    // ========================================

    /// Enable/disable respawning of dormant vehicles.
    pub fn set_respawn_dormant_vehicles(&self, enabled: bool) -> CarlaResult<()> {
        self.inner.set_respawn_dormant_vehicles(enabled);
        Ok(())
    }

    /// Set respawn boundaries for dormant vehicles.
    pub fn set_respawn_boundaries(&self, lower_bound: f32, upper_bound: f32) -> CarlaResult<()> {
        self.inner.set_respawn_boundaries(lower_bound, upper_bound);
        Ok(())
    }

    /// Set maximum boundaries for vehicle spawning.
    pub fn set_max_boundaries(&self, lower: f32, upper: f32) -> CarlaResult<()> {
        self.inner.set_max_boundaries(lower, upper);
        Ok(())
    }

    // ========================================
    // Statistics and Monitoring
    // ========================================

    /// Get current traffic manager configuration.
    pub fn get_config(&self) -> CarlaResult<TrafficManagerConfig> {
        let config = self.inner.get_config();
        Ok(TrafficManagerConfig::from_cxx(config))
    }

    /// Get vehicle-specific configuration.
    pub fn get_vehicle_config(
        &self,
        vehicle: &Vehicle,
    ) -> CarlaResult<TrafficManagerVehicleConfig> {
        let config = self.inner.get_vehicle_config(vehicle.inner().inner());
        Ok(TrafficManagerVehicleConfig::from_cxx(config))
    }

    /// Get traffic manager statistics.
    pub fn get_stats(&self) -> CarlaResult<TrafficManagerStats> {
        let stats = self.inner.get_stats();
        Ok(TrafficManagerStats::from_cxx(stats))
    }

    /// Get next action for a specific vehicle.
    pub fn get_next_action(&self, vehicle: &Vehicle) -> CarlaResult<TrafficManagerAction> {
        let action = self.inner.get_next_action(vehicle.inner().inner());
        Ok(TrafficManagerAction::from_cxx(action))
    }

    // ========================================
    // Lifecycle Management
    // ========================================

    /// Shutdown this traffic manager instance.
    pub fn shutdown(&self) -> CarlaResult<()> {
        self.inner.shutdown();
        Ok(())
    }

    /// Reset all traffic managers to default state.
    pub fn reset_all() -> CarlaResult<()> {
        TrafficManagerWrapper::reset();
        Ok(())
    }

    /// Release all traffic manager resources.
    pub fn release_all() -> CarlaResult<()> {
        TrafficManagerWrapper::release();
        Ok(())
    }

    /// Apply configuration to traffic manager.
    pub fn apply_config(&self, config: &TrafficManagerConfig) -> CarlaResult<()> {
        self.set_global_speed_percentage(config.global_speed_percentage_difference)?;
        self.set_global_lane_offset(config.global_lane_offset)?;
        self.set_global_distance_to_leading_vehicle(config.global_distance_to_leading_vehicle)?;
        self.set_synchronous_mode(config.synchronous_mode)?;
        self.set_synchronous_mode_timeout(config.synchronous_mode_timeout)?;
        self.set_osm_mode(config.osm_mode)?;
        self.set_hybrid_physics_mode(config.hybrid_physics_mode)?;
        self.set_hybrid_physics_radius(config.hybrid_physics_radius)?;
        self.set_random_device_seed(config.random_device_seed)?;
        self.set_respawn_dormant_vehicles(config.respawn_dormant_vehicles)?;
        self.set_respawn_boundaries(config.respawn_lower_bound, config.respawn_upper_bound)?;
        self.set_max_boundaries(config.max_lower_bound, config.max_upper_bound)?;
        Ok(())
    }

    /// Apply vehicle-specific configuration.
    pub fn apply_vehicle_config(
        &self,
        vehicle: &Vehicle,
        config: &TrafficManagerVehicleConfig,
    ) -> CarlaResult<()> {
        self.set_vehicle_speed_percentage(vehicle, config.speed_percentage)?;
        self.set_vehicle_desired_speed(vehicle, config.desired_speed)?;
        self.set_vehicle_lane_offset(vehicle, config.lane_offset)?;
        self.set_vehicle_auto_lane_change(vehicle, config.auto_lane_change)?;
        self.set_vehicle_distance_to_leading_vehicle(vehicle, config.distance_to_leading_vehicle)?;
        self.set_vehicle_percentage_running_light(vehicle, config.percentage_running_light)?;
        self.set_vehicle_percentage_running_sign(vehicle, config.percentage_running_sign)?;
        self.set_vehicle_percentage_ignore_walkers(vehicle, config.percentage_ignore_walkers)?;
        self.set_vehicle_percentage_ignore_vehicles(vehicle, config.percentage_ignore_vehicles)?;
        self.set_vehicle_keep_right_percentage(vehicle, config.keep_right_percentage)?;
        self.set_vehicle_random_left_lane_change_percentage(
            vehicle,
            config.random_left_lane_change_percentage,
        )?;
        self.set_vehicle_random_right_lane_change_percentage(
            vehicle,
            config.random_right_lane_change_percentage,
        )?;
        self.set_vehicle_update_lights(vehicle, config.update_vehicle_lights)?;
        Ok(())
    }

    /// Get access to the underlying TrafficManagerWrapper for direct FFI calls.
    pub(crate) fn inner(&self) -> &TrafficManagerWrapper {
        &self.inner
    }
}

impl Drop for TrafficManager {
    fn drop(&mut self) {
        // Automatically shutdown when dropped
        let _ = self.shutdown();
    }
}
