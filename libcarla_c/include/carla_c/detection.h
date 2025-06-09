#ifndef CARLA_C_DETECTION_H
#define CARLA_C_DETECTION_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Basic detection sensor data access functions

// Get collision event data from sensor data
carla_error_t carla_collision_get_data(const carla_sensor_data_t *data,
                                       carla_collision_event_t *collision);

// Get lane invasion event data from sensor data
carla_error_t
carla_lane_invasion_get_data(const carla_sensor_data_t *data,
                             carla_lane_invasion_event_t *invasion);

// Get obstacle detection event data from sensor data
carla_error_t
carla_obstacle_detection_get_data(const carla_sensor_data_t *data,
                                  carla_obstacle_detection_event_t *detection);

// Collision event processing functions

// Analyze collision severity based on impact parameters
carla_collision_severity_t
carla_collision_analyze_severity(const carla_collision_event_t *collision);

// Classify collision type based on actors involved
carla_collision_type_t
carla_collision_classify_type(const carla_collision_event_t *collision);

// Calculate impact energy from collision parameters
float carla_collision_calculate_impact_energy(
    const carla_collision_event_t *collision);

// Calculate impact speed from normal impulse and actor properties
float carla_collision_calculate_impact_speed(
    const carla_collision_event_t *collision);

// Determine if collision was likely avoidable
bool carla_collision_is_avoidable(const carla_collision_event_t *collision,
                                  float reaction_time, float braking_distance);

// Get collision direction vector (normalized)
carla_error_t
carla_collision_get_direction(const carla_collision_event_t *collision,
                              carla_vector3d_t *direction);

// Lane invasion event processing functions

// Analyze lane invasion intent (intentional vs unintentional)
bool carla_lane_invasion_is_intentional(
    const carla_lane_invasion_event_t *invasion);

// Calculate invasion severity score (0-1, higher is more severe)
float carla_lane_invasion_calculate_severity(
    const carla_lane_invasion_event_t *invasion);

// Get lane marking information by index
carla_error_t
carla_lane_invasion_get_marking(const carla_lane_invasion_event_t *invasion,
                                size_t index, carla_lane_marking_t *marking);

// Check if invasion violates traffic rules
bool carla_lane_invasion_violates_rules(
    const carla_lane_invasion_event_t *invasion);

// Calculate time spent in wrong lane
double carla_lane_invasion_calculate_duration(
    const carla_lane_invasion_event_t *invasion);

// Get invasion angle relative to lane direction
float carla_lane_invasion_get_angle(
    const carla_lane_invasion_event_t *invasion);

// Obstacle detection event processing functions

// Assess threat level of detected obstacle
carla_threat_level_t carla_obstacle_assess_threat_level(
    const carla_obstacle_detection_event_t *detection);

// Classify obstacle type based on actor properties
carla_obstacle_type_t
carla_obstacle_classify_type(const carla_obstacle_detection_event_t *detection);

// Calculate time to collision if current trajectories continue
float carla_obstacle_calculate_time_to_collision(
    const carla_obstacle_detection_event_t *detection);

// Calculate closing speed between actors
float carla_obstacle_calculate_closing_speed(
    const carla_obstacle_detection_event_t *detection);

// Check if obstacle is in the direct path of travel
bool carla_obstacle_is_in_path(
    const carla_obstacle_detection_event_t *detection, float path_width);

// Calculate detection confidence score based on sensor data
float carla_obstacle_calculate_confidence(
    const carla_obstacle_detection_event_t *detection);

// Predict future obstacle position after given time
carla_error_t carla_obstacle_predict_position(
    const carla_obstacle_detection_event_t *detection, double time_delta,
    carla_vector3d_t *predicted_position);

// Detection event filtering functions

// Create default detection filter
carla_detection_filter_t carla_detection_create_default_filter(void);

// Apply filter to collision event
bool carla_collision_apply_filter(const carla_collision_event_t *collision,
                                  const carla_detection_filter_t *filter);

// Apply filter to lane invasion event
bool carla_lane_invasion_apply_filter(
    const carla_lane_invasion_event_t *invasion,
    const carla_detection_filter_t *filter);

// Apply filter to obstacle detection event
bool carla_obstacle_detection_apply_filter(
    const carla_obstacle_detection_event_t *detection,
    const carla_detection_filter_t *filter);

// Filter events by time window
carla_error_t
carla_detection_filter_by_time(const carla_detection_history_t *history,
                               double start_time, double end_time,
                               carla_detection_history_t *filtered_history);

// Filter events by distance range
carla_error_t
carla_detection_filter_by_distance(const carla_detection_history_t *history,
                                   float min_distance, float max_distance,
                                   carla_detection_history_t *filtered_history);

// Detection statistics and analysis functions

// Initialize detection statistics structure
carla_error_t carla_detection_stats_init(carla_detection_statistics_t *stats);

// Update statistics with new collision event
carla_error_t
carla_detection_stats_add_collision(carla_detection_statistics_t *stats,
                                    const carla_collision_event_t *collision);

// Update statistics with new lane invasion event
carla_error_t carla_detection_stats_add_lane_invasion(
    carla_detection_statistics_t *stats,
    const carla_lane_invasion_event_t *invasion);

// Update statistics with new obstacle detection event
carla_error_t carla_detection_stats_add_obstacle_detection(
    carla_detection_statistics_t *stats,
    const carla_obstacle_detection_event_t *detection);

// Calculate overall safety score based on detection statistics (0-100)
float carla_detection_stats_calculate_safety_score(
    const carla_detection_statistics_t *stats);

// Get collision rate per time unit
float carla_detection_stats_get_collision_rate(
    const carla_detection_statistics_t *stats);

// Get lane invasion rate per time unit
float carla_detection_stats_get_invasion_rate(
    const carla_detection_statistics_t *stats);

// Get obstacle detection rate per time unit
float carla_detection_stats_get_detection_rate(
    const carla_detection_statistics_t *stats);

// Detection event history management functions

// Initialize detection history structure
carla_error_t carla_detection_history_init(carla_detection_history_t *history,
                                           size_t initial_capacity);

// Add collision event to history
carla_error_t
carla_detection_history_add_collision(carla_detection_history_t *history,
                                      const carla_collision_event_t *collision);

// Add lane invasion event to history
carla_error_t carla_detection_history_add_lane_invasion(
    carla_detection_history_t *history,
    const carla_lane_invasion_event_t *invasion);

// Add obstacle detection event to history
carla_error_t carla_detection_history_add_obstacle_detection(
    carla_detection_history_t *history,
    const carla_obstacle_detection_event_t *detection);

// Get collision event by index
carla_error_t
carla_detection_history_get_collision(const carla_detection_history_t *history,
                                      size_t index,
                                      carla_collision_event_t *collision);

// Get lane invasion event by index
carla_error_t carla_detection_history_get_lane_invasion(
    const carla_detection_history_t *history, size_t index,
    carla_lane_invasion_event_t *invasion);

// Get obstacle detection event by index
carla_error_t carla_detection_history_get_obstacle_detection(
    const carla_detection_history_t *history, size_t index,
    carla_obstacle_detection_event_t *detection);

// Clear all events from history
void carla_detection_history_clear(carla_detection_history_t *history);

// Get most recent collision event
carla_error_t carla_detection_history_get_latest_collision(
    const carla_detection_history_t *history,
    carla_collision_event_t *collision);

// Get most recent lane invasion event
carla_error_t carla_detection_history_get_latest_lane_invasion(
    const carla_detection_history_t *history,
    carla_lane_invasion_event_t *invasion);

// Get most recent obstacle detection event
carla_error_t carla_detection_history_get_latest_obstacle_detection(
    const carla_detection_history_t *history,
    carla_obstacle_detection_event_t *detection);

// Detection pattern analysis functions

// Detect collision hotspots from history
carla_error_t carla_detection_find_collision_hotspots(
    const carla_detection_history_t *history, float radius, uint32_t min_events,
    carla_vector3d_t **hotspots, uint32_t **event_counts,
    size_t *hotspot_count);

// Analyze collision patterns over time
carla_error_t carla_detection_analyze_collision_patterns(
    const carla_detection_history_t *history, double time_window,
    carla_detection_statistics_t *pattern_stats);

// Find dangerous driving behaviors from lane invasion patterns
carla_error_t carla_detection_analyze_driving_behavior(
    const carla_detection_history_t *history, float *aggressive_score,
    float *inattentive_score, float *reckless_score);

// Predict future collision risk based on historical data
float carla_detection_predict_collision_risk(
    const carla_detection_history_t *history, double prediction_horizon);

// Detection sensor configuration functions

// Create default detection sensor configuration
carla_detection_sensor_config_t carla_detection_create_default_config(void);

// Validate detection sensor configuration
bool carla_detection_validate_config(
    const carla_detection_sensor_config_t *config);

// Update detection sensor configuration
carla_error_t
carla_detection_update_config(carla_detection_sensor_config_t *config,
                              float detection_range, float detection_angle,
                              float update_frequency);

// Utility and conversion functions

// Convert lane marking type to string
const char *carla_lane_marking_type_to_string(carla_lane_marking_type_t type);

// Convert lane marking color to string
const char *
carla_lane_marking_color_to_string(carla_lane_marking_color_t color);

// Convert collision severity to string
const char *
carla_collision_severity_to_string(carla_collision_severity_t severity);

// Convert collision type to string
const char *carla_collision_type_to_string(carla_collision_type_t type);

// Convert obstacle type to string
const char *carla_obstacle_type_to_string(carla_obstacle_type_t type);

// Convert threat level to string
const char *carla_threat_level_to_string(carla_threat_level_t level);

// Export and import functions

// Export detection history to JSON file
carla_error_t
carla_detection_export_history_json(const carla_detection_history_t *history,
                                    const char *filename);

// Export detection statistics to CSV file
carla_error_t
carla_detection_export_stats_csv(const carla_detection_statistics_t *stats,
                                 const char *filename);

// Import detection history from JSON file
carla_error_t
carla_detection_import_history_json(carla_detection_history_t *history,
                                    const char *filename);

// Real-time processing functions

// Process real-time collision event
carla_error_t carla_detection_process_collision_realtime(
    const carla_collision_event_t *collision,
    carla_detection_history_t *history, carla_detection_statistics_t *stats,
    bool *critical_event);

// Process real-time lane invasion event
carla_error_t carla_detection_process_lane_invasion_realtime(
    const carla_lane_invasion_event_t *invasion,
    carla_detection_history_t *history, carla_detection_statistics_t *stats,
    bool *critical_event);

// Process real-time obstacle detection event
carla_error_t carla_detection_process_obstacle_detection_realtime(
    const carla_obstacle_detection_event_t *detection,
    carla_detection_history_t *history, carla_detection_statistics_t *stats,
    carla_threat_level_t *threat_level);

// Multi-sensor fusion functions

// Fuse multiple obstacle detections into single event
carla_error_t carla_detection_fuse_obstacle_detections(
    const carla_obstacle_detection_event_t *detections, size_t detection_count,
    carla_obstacle_detection_event_t *fused_detection);

// Cross-validate detection events from multiple sensors
carla_error_t carla_detection_cross_validate_events(
    const carla_detection_history_t *history1,
    const carla_detection_history_t *history2, float position_tolerance,
    double time_tolerance, carla_detection_history_t *validated_history);

// Memory management functions

// Free collision event data
void carla_collision_event_destroy(carla_collision_event_t *collision);

// Free lane invasion event data
void carla_lane_invasion_event_destroy(carla_lane_invasion_event_t *invasion);

// Free obstacle detection event data
void carla_obstacle_detection_event_destroy(
    carla_obstacle_detection_event_t *detection);

// Free detection history data
void carla_detection_history_destroy(carla_detection_history_t *history);

// Free detection statistics data
void carla_detection_statistics_destroy(carla_detection_statistics_t *stats);

// Free detection filter data
void carla_detection_filter_destroy(carla_detection_filter_t *filter);

// Free array data
void carla_detection_free_array(void *array);

// Free hotspot analysis results
void carla_detection_free_hotspots(carla_vector3d_t *hotspots,
                                   uint32_t *event_counts);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_DETECTION_H
