#ifndef CARLA_C_SENSOR_H
#define CARLA_C_SENSOR_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration of opaque sensor type (sensor is a specialized actor)
typedef carla_actor_t carla_sensor_t;

// Sensor lifecycle and callback management
carla_error_t carla_sensor_listen(carla_sensor_t *sensor,
                                  carla_sensor_callback_t callback,
                                  void *user_data);
carla_error_t carla_sensor_stop(carla_sensor_t *sensor);
bool carla_sensor_is_listening(const carla_sensor_t *sensor);

// Sensor data introspection
carla_sensor_data_type_t
carla_sensor_data_get_type(const carla_sensor_data_t *data);
carla_sensor_data_info_t
carla_sensor_data_get_info(const carla_sensor_data_t *data);

// Sensor data access functions

// Common sensor data access
uint64_t carla_sensor_data_get_frame(const carla_sensor_data_t *data);
double carla_sensor_data_get_timestamp(const carla_sensor_data_t *data);
carla_transform_t
carla_sensor_data_get_transform(const carla_sensor_data_t *data);

// Image data access
carla_image_data_t carla_sensor_data_get_image(const carla_sensor_data_t *data);
uint32_t carla_image_data_get_width(const carla_sensor_data_t *data);
uint32_t carla_image_data_get_height(const carla_sensor_data_t *data);
uint32_t carla_image_data_get_fov(const carla_sensor_data_t *data);
const uint8_t *carla_image_data_get_raw_data(const carla_sensor_data_t *data);
size_t carla_image_data_get_raw_data_size(const carla_sensor_data_t *data);

// Helper function to copy image data safely
carla_error_t carla_image_data_copy_to_buffer(const carla_sensor_data_t *data,
                                              uint8_t *buffer,
                                              size_t buffer_size);

// LiDAR data access
carla_lidar_data_t carla_sensor_data_get_lidar(const carla_sensor_data_t *data);
size_t carla_lidar_data_get_point_count(const carla_sensor_data_t *data);
const carla_lidar_detection_t *
carla_lidar_data_get_points(const carla_sensor_data_t *data);
uint32_t carla_lidar_data_get_horizontal_angle(const carla_sensor_data_t *data);
uint32_t carla_lidar_data_get_channels(const carla_sensor_data_t *data);

// Helper function to copy LiDAR points safely
carla_error_t
carla_lidar_data_copy_points_to_buffer(const carla_sensor_data_t *data,
                                       carla_lidar_detection_t *buffer,
                                       size_t buffer_size);

// Semantic LiDAR data access
carla_semantic_lidar_data_t
carla_sensor_data_get_semantic_lidar(const carla_sensor_data_t *data);
size_t
carla_semantic_lidar_data_get_point_count(const carla_sensor_data_t *data);
const carla_semantic_lidar_detection_t *
carla_semantic_lidar_data_get_points(const carla_sensor_data_t *data);

// Radar data access
carla_radar_data_t carla_sensor_data_get_radar(const carla_sensor_data_t *data);
size_t carla_radar_data_get_detection_count(const carla_sensor_data_t *data);
const carla_radar_detection_t *
carla_radar_data_get_detections(const carla_sensor_data_t *data);

// IMU data access
carla_imu_data_t carla_sensor_data_get_imu(const carla_sensor_data_t *data);
carla_vector3d_t
carla_imu_data_get_accelerometer(const carla_sensor_data_t *data);
carla_vector3d_t carla_imu_data_get_gyroscope(const carla_sensor_data_t *data);
float carla_imu_data_get_compass(const carla_sensor_data_t *data);

// GNSS data access
carla_gnss_data_t carla_sensor_data_get_gnss(const carla_sensor_data_t *data);
double carla_gnss_data_get_latitude(const carla_sensor_data_t *data);
double carla_gnss_data_get_longitude(const carla_sensor_data_t *data);
double carla_gnss_data_get_altitude(const carla_sensor_data_t *data);

// Collision event data access
carla_collision_data_t
carla_sensor_data_get_collision(const carla_sensor_data_t *data);
carla_actor_t *carla_collision_data_get_actor(const carla_sensor_data_t *data);
carla_actor_t *
carla_collision_data_get_other_actor(const carla_sensor_data_t *data);
carla_vector3d_t
carla_collision_data_get_normal_impulse(const carla_sensor_data_t *data);

// Lane invasion event data access
carla_lane_invasion_data_t
carla_sensor_data_get_lane_invasion(const carla_sensor_data_t *data);
carla_actor_t *
carla_lane_invasion_data_get_actor(const carla_sensor_data_t *data);
size_t carla_lane_invasion_data_get_crossed_lane_markings_count(
    const carla_sensor_data_t *data);

// Obstacle detection event data access
carla_obstacle_detection_data_t
carla_sensor_data_get_obstacle_detection(const carla_sensor_data_t *data);
carla_actor_t *
carla_obstacle_detection_data_get_actor(const carla_sensor_data_t *data);
carla_actor_t *
carla_obstacle_detection_data_get_other_actor(const carla_sensor_data_t *data);
float carla_obstacle_detection_data_get_distance(
    const carla_sensor_data_t *data);

// DVS event array data access
carla_dvs_event_array_data_t
carla_sensor_data_get_dvs_event_array(const carla_sensor_data_t *data);
size_t
carla_dvs_event_array_data_get_event_count(const carla_sensor_data_t *data);
const carla_dvs_event_t *
carla_dvs_event_array_data_get_events(const carla_sensor_data_t *data);
uint32_t carla_dvs_event_array_data_get_width(const carla_sensor_data_t *data);
uint32_t carla_dvs_event_array_data_get_height(const carla_sensor_data_t *data);

// Optical flow image data access
carla_optical_flow_data_t
carla_sensor_data_get_optical_flow(const carla_sensor_data_t *data);
uint32_t carla_optical_flow_data_get_width(const carla_sensor_data_t *data);
uint32_t carla_optical_flow_data_get_height(const carla_sensor_data_t *data);
const carla_optical_flow_pixel_t *
carla_optical_flow_data_get_pixels(const carla_sensor_data_t *data);

// Utility functions to check if an actor is a sensor
bool carla_actor_is_sensor(const carla_actor_t *actor);

// Utility function to cast actor to sensor (returns NULL if not a sensor)
carla_sensor_t *carla_actor_as_sensor(carla_actor_t *actor);

// Sensor attribute management
carla_error_t carla_sensor_set_attribute(carla_sensor_t *sensor,
                                         const char *key, const char *value);
const char *carla_sensor_get_attribute(const carla_sensor_t *sensor,
                                       const char *key);
void *carla_sensor_get_calibration_data(const carla_sensor_t *sensor);

// Memory management for sensor data (called automatically, but available for
// manual control)
void carla_sensor_data_destroy(carla_sensor_data_t *data);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_SENSOR_H
