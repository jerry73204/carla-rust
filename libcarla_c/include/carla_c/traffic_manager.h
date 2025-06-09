#ifndef CARLA_C_TRAFFIC_MANAGER_H
#define CARLA_C_TRAFFIC_MANAGER_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Traffic Manager lifecycle management

// Create and get traffic manager instance for specified port
carla_traffic_manager_t *
carla_traffic_manager_get_instance(carla_client_t *client, uint16_t port);

// Get default traffic manager instance (port 8000)
carla_traffic_manager_t *
carla_traffic_manager_get_default(carla_client_t *client);

// Release all traffic manager instances
void carla_traffic_manager_release_all(void);

// Reset all traffic manager instances to default state
void carla_traffic_manager_reset_all(void);

// Shutdown specific traffic manager instance
void carla_traffic_manager_shutdown(carla_traffic_manager_t *tm);

// Get traffic manager instance information
carla_error_t
carla_traffic_manager_get_info(carla_traffic_manager_t *tm,
                               carla_traffic_manager_info_t *info);

// Vehicle registration and management

// Register vehicles with traffic manager
carla_error_t carla_traffic_manager_register_vehicles(
    carla_traffic_manager_t *tm, carla_actor_t **actors, size_t actor_count);

// Register single vehicle with traffic manager
carla_error_t
carla_traffic_manager_register_vehicle(carla_traffic_manager_t *tm,
                                       carla_actor_t *actor);

// Unregister vehicles from traffic manager
carla_error_t carla_traffic_manager_unregister_vehicles(
    carla_traffic_manager_t *tm, carla_actor_t **actors, size_t actor_count);

// Unregister single vehicle from traffic manager
carla_error_t
carla_traffic_manager_unregister_vehicle(carla_traffic_manager_t *tm,
                                         carla_actor_t *actor);

// Synchronous execution control

// Set synchronous mode for traffic manager
carla_error_t
carla_traffic_manager_set_synchronous_mode(carla_traffic_manager_t *tm,
                                           bool mode);

// Perform synchronous tick and return whether successful
bool carla_traffic_manager_synchronous_tick(carla_traffic_manager_t *tm);

// Set timeout for synchronous operations in milliseconds
carla_error_t
carla_traffic_manager_set_synchronous_timeout(carla_traffic_manager_t *tm,
                                              double timeout_ms);

// Global configuration

// Set global speed percentage difference (-100 to 100%)
carla_error_t
carla_traffic_manager_set_global_speed_percentage(carla_traffic_manager_t *tm,
                                                  float percentage);

// Set global lane offset for all vehicles
carla_error_t
carla_traffic_manager_set_global_lane_offset(carla_traffic_manager_t *tm,
                                             float offset);

// Set global distance to leading vehicle
carla_error_t carla_traffic_manager_set_global_distance_to_leading_vehicle(
    carla_traffic_manager_t *tm, float distance);

// Set random device seed for reproducible behavior
carla_error_t
carla_traffic_manager_set_random_device_seed(carla_traffic_manager_t *tm,
                                             uint64_t seed);

// Enable/disable OpenStreetMap mode
carla_error_t carla_traffic_manager_set_osm_mode(carla_traffic_manager_t *tm,
                                                 bool osm_mode);

// Per-vehicle speed control

// Set speed percentage difference for specific vehicle (-100 to 100%)
carla_error_t carla_traffic_manager_set_vehicle_speed_percentage(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage);

// Set desired speed for specific vehicle in m/s
carla_error_t carla_traffic_manager_set_vehicle_desired_speed(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float speed);

// Per-vehicle lane behavior

// Set lane offset for specific vehicle
carla_error_t carla_traffic_manager_set_vehicle_lane_offset(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float offset);

// Enable/disable automatic lane changes for vehicle
carla_error_t carla_traffic_manager_set_vehicle_auto_lane_change(
    carla_traffic_manager_t *tm, carla_actor_t *actor, bool enable);

// Force lane change for vehicle (true = left, false = right)
carla_error_t carla_traffic_manager_force_vehicle_lane_change(
    carla_traffic_manager_t *tm, carla_actor_t *actor, bool direction);

// Set keep right percentage for vehicle (0-100%)
carla_error_t carla_traffic_manager_set_vehicle_keep_right_percentage(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage);

// Set random left lane change percentage (0-100%)
carla_error_t
carla_traffic_manager_set_vehicle_random_left_lane_change_percentage(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage);

// Set random right lane change percentage (0-100%)
carla_error_t
carla_traffic_manager_set_vehicle_random_right_lane_change_percentage(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage);

// Per-vehicle following behavior

// Set distance to leading vehicle for specific vehicle
carla_error_t carla_traffic_manager_set_vehicle_distance_to_leading_vehicle(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float distance);

// Traffic rule compliance

// Set percentage of running red lights (0-100%)
carla_error_t carla_traffic_manager_set_vehicle_percentage_running_light(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage);

// Set percentage of ignoring stop signs (0-100%)
carla_error_t carla_traffic_manager_set_vehicle_percentage_running_sign(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage);

// Set percentage of ignoring pedestrians (0-100%)
carla_error_t carla_traffic_manager_set_vehicle_percentage_ignore_walkers(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage);

// Set percentage of ignoring other vehicles (0-100%)
carla_error_t carla_traffic_manager_set_vehicle_percentage_ignore_vehicles(
    carla_traffic_manager_t *tm, carla_actor_t *actor, float percentage);

// Collision detection and safety

// Enable/disable collision detection between two actors
carla_error_t carla_traffic_manager_set_collision_detection(
    carla_traffic_manager_t *tm, carla_actor_t *reference_actor,
    carla_actor_t *other_actor, bool detect_collision);

// Vehicle lights and visual behavior

// Enable/disable automatic vehicle light updates
carla_error_t carla_traffic_manager_set_vehicle_update_lights(
    carla_traffic_manager_t *tm, carla_actor_t *actor, bool update_lights);

// Route and path management

// Set custom path for vehicle (sequence of world locations)
carla_error_t carla_traffic_manager_set_vehicle_custom_path(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    const carla_traffic_manager_path_t *path);

// Remove custom path for vehicle
carla_error_t
carla_traffic_manager_remove_vehicle_custom_path(carla_traffic_manager_t *tm,
                                                 carla_actor_t *actor);

// Update custom path for vehicle
carla_error_t carla_traffic_manager_update_vehicle_custom_path(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    const carla_traffic_manager_path_t *path);

// Set imported route for vehicle (sequence of road options)
carla_error_t carla_traffic_manager_set_vehicle_imported_route(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    const carla_traffic_manager_route_t *route);

// Remove imported route for vehicle
carla_error_t
carla_traffic_manager_remove_vehicle_imported_route(carla_traffic_manager_t *tm,
                                                    carla_actor_t *actor);

// Update imported route for vehicle
carla_error_t carla_traffic_manager_update_vehicle_imported_route(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    const carla_traffic_manager_route_t *route);

// Action and decision queries

// Get next action for vehicle
carla_error_t carla_traffic_manager_get_vehicle_next_action(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    carla_traffic_manager_action_t *action);

// Get action buffer for vehicle
carla_error_t carla_traffic_manager_get_vehicle_action_buffer(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    carla_traffic_manager_action_buffer_t *buffer);

// Performance and simulation optimization

// Enable/disable hybrid physics mode for performance
carla_error_t
carla_traffic_manager_set_hybrid_physics_mode(carla_traffic_manager_t *tm,
                                              bool mode);

// Set hybrid physics radius
carla_error_t
carla_traffic_manager_set_hybrid_physics_radius(carla_traffic_manager_t *tm,
                                                float radius);

// Enable/disable respawning of dormant vehicles
carla_error_t
carla_traffic_manager_set_respawn_dormant_vehicles(carla_traffic_manager_t *tm,
                                                   bool enable);

// Set boundaries for respawning dormant vehicles
carla_error_t carla_traffic_manager_set_respawn_boundaries(
    carla_traffic_manager_t *tm, float lower_bound, float upper_bound);

// Set maximum boundaries for traffic manager operation
carla_error_t
carla_traffic_manager_set_max_boundaries(carla_traffic_manager_t *tm,
                                         float lower, float upper);

// Configuration management

// Get current configuration for traffic manager
carla_error_t
carla_traffic_manager_get_config(carla_traffic_manager_t *tm,
                                 carla_traffic_manager_config_t *config);

// Apply configuration to traffic manager
carla_error_t
carla_traffic_manager_set_config(carla_traffic_manager_t *tm,
                                 const carla_traffic_manager_config_t *config);

// Get vehicle-specific configuration
carla_error_t carla_traffic_manager_get_vehicle_config(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    carla_traffic_manager_vehicle_config_t *config);

// Apply vehicle-specific configuration
carla_error_t carla_traffic_manager_set_vehicle_config(
    carla_traffic_manager_t *tm, carla_actor_t *actor,
    const carla_traffic_manager_vehicle_config_t *config);

// Statistics and monitoring

// Get traffic manager statistics
carla_error_t
carla_traffic_manager_get_stats(carla_traffic_manager_t *tm,
                                carla_traffic_manager_stats_t *stats);

// Reset traffic manager statistics
carla_error_t carla_traffic_manager_reset_stats(carla_traffic_manager_t *tm);

// Get list of registered vehicles
carla_error_t carla_traffic_manager_get_registered_vehicles(
    carla_traffic_manager_t *tm, carla_actor_t ***actors, size_t *actor_count);

// Check if vehicle is registered with traffic manager
bool carla_traffic_manager_is_vehicle_registered(carla_traffic_manager_t *tm,
                                                 carla_actor_t *actor);

// Utility functions

// Create default traffic manager configuration
carla_traffic_manager_config_t
carla_traffic_manager_create_default_config(uint16_t port);

// Create default vehicle configuration
carla_traffic_manager_vehicle_config_t
carla_traffic_manager_create_default_vehicle_config(void);

// Create path from array of locations
carla_error_t
carla_traffic_manager_create_path(const carla_vector3d_t *locations,
                                  size_t location_count, bool empty_buffer,
                                  carla_traffic_manager_path_t *path);

// Create route from array of road options
carla_error_t
carla_traffic_manager_create_route(const carla_road_option_t *road_options,
                                   size_t option_count, bool empty_buffer,
                                   carla_traffic_manager_route_t *route);

// String conversion functions

// Convert road option to string
const char *carla_road_option_to_string(carla_road_option_t road_option);

// Convert road option from string
carla_road_option_t carla_road_option_from_string(const char *road_option_str);

// Memory management

// Free traffic manager path
void carla_traffic_manager_path_destroy(carla_traffic_manager_path_t *path);

// Free traffic manager route
void carla_traffic_manager_route_destroy(carla_traffic_manager_route_t *route);

// Free traffic manager action buffer
void carla_traffic_manager_action_buffer_destroy(
    carla_traffic_manager_action_buffer_t *buffer);

// Free array of actors
void carla_traffic_manager_free_actor_array(carla_actor_t **actors);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_TRAFFIC_MANAGER_H
