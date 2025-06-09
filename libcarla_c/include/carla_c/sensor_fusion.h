#ifndef CARLA_C_SENSOR_FUSION_H
#define CARLA_C_SENSOR_FUSION_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Sensor Fusion System Management

// Create a new sensor fusion system
carla_sensor_fusion_system_t *
carla_sensor_fusion_create(const carla_sensor_fusion_config_t *config);

// Destroy sensor fusion system and free memory
void carla_sensor_fusion_destroy(carla_sensor_fusion_system_t *system);

// Configure sensor fusion system
carla_error_t
carla_sensor_fusion_configure(carla_sensor_fusion_system_t *system,
                              const carla_sensor_fusion_config_t *config);

// Get current fusion configuration
carla_error_t
carla_sensor_fusion_get_config(const carla_sensor_fusion_system_t *system,
                               carla_sensor_fusion_config_t *config);

// Reset fusion system state
carla_error_t carla_sensor_fusion_reset(carla_sensor_fusion_system_t *system);

// Sensor Registration and Calibration

// Register a sensor with the fusion system
carla_error_t carla_sensor_fusion_register_sensor(
    carla_sensor_fusion_system_t *system, carla_actor_t *sensor,
    carla_sensor_modality_t modality,
    const carla_sensor_calibration_data_t *calibration);

// Unregister a sensor from the fusion system
carla_error_t
carla_sensor_fusion_unregister_sensor(carla_sensor_fusion_system_t *system,
                                      carla_actor_t *sensor);

// Update sensor calibration data
carla_error_t carla_sensor_fusion_update_calibration(
    carla_sensor_fusion_system_t *system, carla_actor_t *sensor,
    const carla_sensor_calibration_data_t *calibration);

// Get sensor calibration data
carla_error_t carla_sensor_fusion_get_calibration(
    const carla_sensor_fusion_system_t *system, carla_actor_t *sensor,
    carla_sensor_calibration_data_t *calibration);

// Validate sensor calibration
carla_error_t carla_sensor_fusion_validate_calibration(
    const carla_sensor_fusion_system_t *system, carla_actor_t *sensor,
    float *calibration_error);

// Sensor Data Processing

// Process incoming sensor data
carla_error_t
carla_sensor_fusion_process_data(carla_sensor_fusion_system_t *system,
                                 carla_actor_t *sensor,
                                 carla_sensor_data_t *data);

// Get synchronized sensor data
carla_error_t carla_sensor_fusion_get_synchronized_data(
    carla_sensor_fusion_system_t *system, double timestamp,
    carla_synchronized_sensor_data_t *sync_data);

// Get latest synchronized sensor data
carla_error_t carla_sensor_fusion_get_latest_synchronized_data(
    carla_sensor_fusion_system_t *system,
    carla_synchronized_sensor_data_t *sync_data);

// Set callback for synchronized sensor data
carla_error_t
carla_sensor_fusion_set_sync_callback(carla_sensor_fusion_system_t *system,
                                      carla_fusion_callback_t callback,
                                      void *user_data);

// Camera-LiDAR Fusion

// Perform camera-LiDAR fusion
carla_error_t carla_camera_lidar_fusion(
    const carla_image_data_t *camera_data, const carla_lidar_data_t *lidar_data,
    const carla_sensor_calibration_data_t *camera_calibration,
    const carla_sensor_calibration_data_t *lidar_calibration,
    const carla_camera_lidar_fusion_config_t *config,
    carla_camera_lidar_fusion_result_t *result);

// Project LiDAR points to camera image
carla_error_t carla_project_lidar_to_camera(
    const carla_lidar_data_t *lidar_data,
    const carla_sensor_calibration_data_t *camera_calibration,
    const carla_sensor_calibration_data_t *lidar_calibration,
    carla_vector3d_t **projected_points, size_t *point_count);

// Colorize point cloud with camera data
carla_error_t carla_colorize_point_cloud(
    const carla_lidar_data_t *lidar_data, const carla_image_data_t *camera_data,
    const carla_sensor_calibration_data_t *camera_calibration,
    const carla_sensor_calibration_data_t *lidar_calibration,
    carla_vector3d_t **colored_points, size_t *point_count);

// Generate depth map from LiDAR data
carla_error_t carla_generate_depth_map_from_lidar(
    const carla_lidar_data_t *lidar_data,
    const carla_sensor_calibration_data_t *camera_calibration,
    const carla_sensor_calibration_data_t *lidar_calibration, uint32_t width,
    uint32_t height, float **depth_map);

// Destroy camera-LiDAR fusion result
void carla_camera_lidar_fusion_result_destroy(
    carla_camera_lidar_fusion_result_t *result);

// Multi-Sensor Object Detection and Tracking

// Perform multi-sensor object detection
carla_error_t
carla_fused_object_detection(carla_sensor_fusion_system_t *system,
                             const carla_synchronized_sensor_data_t *sync_data,
                             carla_fused_object_list_data_t *objects);

// Update object tracking with new detections
carla_error_t
carla_update_object_tracking(carla_sensor_fusion_system_t *system,
                             const carla_fused_object_list_data_t *detections,
                             carla_fused_object_list_data_t *tracked_objects);

// Get current tracked objects
carla_error_t
carla_get_tracked_objects(const carla_sensor_fusion_system_t *system,
                          carla_fused_object_list_data_t *tracked_objects);

// Set callback for object detection
carla_error_t carla_sensor_fusion_set_object_callback(
    carla_sensor_fusion_system_t *system,
    carla_object_detection_callback_t callback, void *user_data);

// Associate objects between sensor modalities
carla_error_t
carla_associate_objects(const carla_fused_object_list_data_t *objects_a,
                        const carla_fused_object_list_data_t *objects_b,
                        float association_threshold, uint32_t **associations,
                        size_t *association_count);

// Multi-Sensor Localization

// Initialize localization filter
carla_error_t
carla_localization_filter_init(carla_sensor_fusion_system_t *system,
                               const carla_localization_filter_config_t *config,
                               const carla_localization_state_t *initial_state);

// Update localization with IMU data
carla_error_t
carla_localization_update_imu(carla_sensor_fusion_system_t *system,
                              const carla_imu_data_t *imu_data);

// Update localization with GNSS data
carla_error_t
carla_localization_update_gnss(carla_sensor_fusion_system_t *system,
                               const carla_gnss_data_t *gnss_data);

// Update localization with odometry data
carla_error_t carla_localization_update_odometry(
    carla_sensor_fusion_system_t *system, const carla_vector3d_t *position,
    const carla_vector3d_t *velocity, double timestamp);

// Get current localization state
carla_error_t
carla_localization_get_state(const carla_sensor_fusion_system_t *system,
                             carla_localization_state_t *state);

// Set callback for localization updates
carla_error_t carla_sensor_fusion_set_localization_callback(
    carla_sensor_fusion_system_t *system,
    carla_localization_callback_t callback, void *user_data);

// Sensor Synchronization

// Create sensor sync buffer
carla_sensor_sync_buffer_t *
carla_sensor_sync_buffer_create(const carla_sensor_sync_config_t *config);

// Destroy sensor sync buffer
void carla_sensor_sync_buffer_destroy(carla_sensor_sync_buffer_t *buffer);

// Add sensor data to sync buffer
carla_error_t
carla_sensor_sync_buffer_add_data(carla_sensor_sync_buffer_t *buffer,
                                  carla_sensor_modality_t modality,
                                  carla_sensor_data_t *data);

// Get synchronized data from buffer
carla_error_t
carla_sensor_sync_buffer_get_data(carla_sensor_sync_buffer_t *buffer,
                                  double timestamp,
                                  carla_synchronized_sensor_data_t *sync_data);

// Clear sync buffer
carla_error_t
carla_sensor_sync_buffer_clear(carla_sensor_sync_buffer_t *buffer);

// Get sync buffer statistics
carla_error_t
carla_sensor_sync_buffer_get_stats(const carla_sensor_sync_buffer_t *buffer,
                                   carla_sensor_fusion_stats_t *stats);

// Coordinate System Transformations

// Transform point from one coordinate frame to another
carla_error_t carla_transform_point(const carla_vector3d_t *point,
                                    const carla_transform_t *transform,
                                    carla_vector3d_t *transformed_point);

// Transform point cloud from one coordinate frame to another
carla_error_t carla_transform_point_cloud(const carla_vector3d_t *points,
                                          size_t point_count,
                                          const carla_transform_t *transform,
                                          carla_vector3d_t *transformed_points);

// Convert between coordinate systems (world, vehicle, sensor)
carla_error_t
carla_convert_coordinates(const carla_vector3d_t *point, const char *from_frame,
                          const char *to_frame,
                          const carla_transform_t *vehicle_transform,
                          const carla_transform_t *sensor_transform,
                          carla_vector3d_t *converted_point);

// Data Interpolation and Extrapolation

// Interpolate between two sensor data points
carla_error_t carla_interpolate_sensor_data(
    const carla_sensor_data_t *data1, const carla_sensor_data_t *data2,
    double target_timestamp, carla_sensor_data_t *interpolated_data);

// Extrapolate sensor data to target timestamp
carla_error_t
carla_extrapolate_sensor_data(const carla_sensor_data_t *data,
                              double target_timestamp,
                              carla_sensor_data_t *extrapolated_data);

// Interpolate position and orientation
carla_error_t carla_interpolate_pose(const carla_transform_t *pose1,
                                     const carla_transform_t *pose2, double t1,
                                     double t2, double target_time,
                                     carla_transform_t *interpolated_pose);

// Utilities and Configuration

// Create default fusion configuration
carla_sensor_fusion_config_t carla_sensor_fusion_create_default_config(void);

// Create default sync configuration
carla_sensor_sync_config_t carla_sensor_sync_create_default_config(void);

// Create default camera-LiDAR fusion configuration
carla_camera_lidar_fusion_config_t
carla_camera_lidar_fusion_create_default_config(void);

// Create default localization filter configuration
carla_localization_filter_config_t
carla_localization_filter_create_default_config(void);

// Validate fusion configuration
carla_error_t
carla_sensor_fusion_validate_config(const carla_sensor_fusion_config_t *config);

// Get fusion system statistics
carla_error_t
carla_sensor_fusion_get_stats(const carla_sensor_fusion_system_t *system,
                              carla_sensor_fusion_stats_t *stats);

// Reset fusion system statistics
carla_error_t
carla_sensor_fusion_reset_stats(carla_sensor_fusion_system_t *system);

// Enable/disable debug output
carla_error_t
carla_sensor_fusion_set_debug_output(carla_sensor_fusion_system_t *system,
                                     bool enable);

// Quality Assessment

// Assess synchronization quality
carla_fusion_quality_t
carla_assess_sync_quality(const carla_synchronized_sensor_data_t *sync_data);

// Assess fusion quality for object detection
carla_fusion_quality_t
carla_assess_object_fusion_quality(const carla_fused_object_data_t *object);

// Assess localization quality
carla_fusion_quality_t
carla_assess_localization_quality(const carla_localization_state_t *state);

// Calculate sensor coverage overlap
float carla_calculate_sensor_coverage_overlap(
    const carla_sensor_calibration_data_t *sensor1,
    const carla_sensor_calibration_data_t *sensor2);

// Memory Management

// Destroy synchronized sensor data
void carla_synchronized_sensor_data_destroy(
    carla_synchronized_sensor_data_t *sync_data);

// Destroy fused object list
void carla_fused_object_list_destroy(carla_fused_object_list_data_t *objects);

// Destroy localization state
void carla_localization_state_destroy(carla_localization_state_t *state);

// Destroy sensor calibration data
void carla_sensor_calibration_data_destroy(
    carla_sensor_calibration_data_t *calibration);

// Destroy fusion statistics
void carla_sensor_fusion_stats_destroy(carla_sensor_fusion_stats_t *stats);

// String conversion utilities
const char *
carla_fusion_algorithm_to_string(carla_fusion_algorithm_t algorithm);
carla_fusion_algorithm_t
carla_fusion_algorithm_from_string(const char *algorithm_str);

const char *carla_fusion_quality_to_string(carla_fusion_quality_t quality);
carla_fusion_quality_t
carla_fusion_quality_from_string(const char *quality_str);

const char *carla_sync_method_to_string(carla_sync_method_t method);
carla_sync_method_t carla_sync_method_from_string(const char *method_str);

const char *carla_sensor_modality_to_string(carla_sensor_modality_t modality);
carla_sensor_modality_t
carla_sensor_modality_from_string(const char *modality_str);

const char *carla_fused_object_type_to_string(carla_fused_object_type_t type);
carla_fused_object_type_t
carla_fused_object_type_from_string(const char *type_str);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_SENSOR_FUSION_H
