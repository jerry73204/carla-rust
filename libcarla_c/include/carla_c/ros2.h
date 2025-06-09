#ifndef CARLA_C_ROS2_H
#define CARLA_C_ROS2_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// ROS2 Manager Lifecycle

// Initialize ROS2 manager with configuration
carla_ros2_manager_t *carla_ros2_create(const carla_ros2_config_t *config);

// Destroy ROS2 manager and cleanup resources
void carla_ros2_destroy(carla_ros2_manager_t *manager);

// Enable or disable ROS2 functionality globally
carla_error_t carla_ros2_set_enabled(carla_ros2_manager_t *manager,
                                     bool enabled);

// Check if ROS2 is currently enabled
bool carla_ros2_is_enabled(const carla_ros2_manager_t *manager);

// Update ROS2 configuration
carla_error_t carla_ros2_configure(carla_ros2_manager_t *manager,
                                   const carla_ros2_config_t *config);

// Get current ROS2 configuration
carla_error_t carla_ros2_get_config(const carla_ros2_manager_t *manager,
                                    carla_ros2_config_t *config);

// ROS2 Time and Frame Management

// Set simulation time for ROS2 clock publishing
carla_error_t carla_ros2_set_timestamp(carla_ros2_manager_t *manager,
                                       double timestamp_seconds);

// Set current simulation frame number
carla_error_t carla_ros2_set_frame_number(carla_ros2_manager_t *manager,
                                          uint64_t frame_number);

// Publish clock message with current simulation time
carla_error_t carla_ros2_publish_clock(carla_ros2_manager_t *manager);

// Actor and Sensor Management

// Add actor to ROS2 with custom name
carla_error_t carla_ros2_add_actor(carla_ros2_manager_t *manager,
                                   carla_actor_t *actor, const char *ros_name);

// Remove actor from ROS2
carla_error_t carla_ros2_remove_actor(carla_ros2_manager_t *manager,
                                      carla_actor_t *actor);

// Get ROS2 name for actor
carla_error_t carla_ros2_get_actor_name(const carla_ros2_manager_t *manager,
                                        carla_actor_t *actor, char *ros_name,
                                        size_t name_size);

// Stream Configuration and Control

// Enable sensor data stream
carla_error_t
carla_ros2_enable_stream(carla_ros2_manager_t *manager, carla_actor_t *sensor,
                         const carla_ros2_stream_config_t *config);

// Disable sensor data stream
carla_error_t carla_ros2_disable_stream(carla_ros2_manager_t *manager,
                                        carla_actor_t *sensor);

// Check if stream is enabled for sensor
bool carla_ros2_is_stream_enabled(const carla_ros2_manager_t *manager,
                                  carla_actor_t *sensor);

// Update stream configuration
carla_error_t
carla_ros2_update_stream_config(carla_ros2_manager_t *manager,
                                carla_actor_t *sensor,
                                const carla_ros2_stream_config_t *config);

// Get stream configuration
carla_error_t carla_ros2_get_stream_config(const carla_ros2_manager_t *manager,
                                           carla_actor_t *sensor,
                                           carla_ros2_stream_config_t *config);

// Data Publishing Functions

// Publish camera data (RGB, depth, semantic, etc.)
carla_error_t carla_ros2_publish_camera_data(carla_ros2_manager_t *manager,
                                             carla_actor_t *camera,
                                             carla_sensor_data_t *data);

// Publish LiDAR point cloud data
carla_error_t carla_ros2_publish_lidar_data(carla_ros2_manager_t *manager,
                                            carla_actor_t *lidar,
                                            carla_sensor_data_t *data);

// Publish IMU sensor data
carla_error_t carla_ros2_publish_imu_data(carla_ros2_manager_t *manager,
                                          carla_actor_t *imu,
                                          carla_sensor_data_t *data);

// Publish GNSS sensor data
carla_error_t carla_ros2_publish_gnss_data(carla_ros2_manager_t *manager,
                                           carla_actor_t *gnss,
                                           carla_sensor_data_t *data);

// Publish radar sensor data
carla_error_t carla_ros2_publish_radar_data(carla_ros2_manager_t *manager,
                                            carla_actor_t *radar,
                                            carla_sensor_data_t *data);

// Publish collision event data
carla_error_t carla_ros2_publish_collision_data(carla_ros2_manager_t *manager,
                                                carla_actor_t *sensor,
                                                carla_sensor_data_t *data);

// Publish lane invasion event data
carla_error_t
carla_ros2_publish_lane_invasion_data(carla_ros2_manager_t *manager,
                                      carla_actor_t *sensor,
                                      carla_sensor_data_t *data);

// Publish actor transform data
carla_error_t carla_ros2_publish_transform(carla_ros2_manager_t *manager,
                                           carla_actor_t *actor,
                                           const carla_transform_t *transform);

// Publish vehicle speed data
carla_error_t carla_ros2_publish_speedometer(carla_ros2_manager_t *manager,
                                             carla_actor_t *vehicle,
                                             float speed_mps);

// Generic sensor data publishing
carla_error_t carla_ros2_publish_sensor_data(carla_ros2_manager_t *manager,
                                             carla_actor_t *sensor,
                                             carla_sensor_data_t *data);

// Vehicle Control Subscription

// Subscribe to vehicle control commands
carla_error_t carla_ros2_subscribe_vehicle_control(
    carla_ros2_manager_t *manager, carla_actor_t *vehicle,
    carla_ros2_vehicle_control_callback_t callback, void *user_data);

// Unsubscribe from vehicle control commands
carla_error_t
carla_ros2_unsubscribe_vehicle_control(carla_ros2_manager_t *manager,
                                       carla_actor_t *vehicle);

// Get latest vehicle control command
carla_error_t
carla_ros2_get_vehicle_control(const carla_ros2_manager_t *manager,
                               carla_actor_t *vehicle,
                               carla_ros2_vehicle_control_t *control);

// Check if vehicle control is available
bool carla_ros2_has_vehicle_control(const carla_ros2_manager_t *manager,
                                    carla_actor_t *vehicle);

// Message Conversion Functions

// Convert CARLA sensor data to ROS2 image message
carla_error_t
carla_ros2_convert_to_image(const carla_sensor_data_t *sensor_data,
                            const char *frame_id,
                            carla_ros2_image_t *ros_image);

// Convert CARLA LiDAR data to ROS2 point cloud message
carla_error_t
carla_ros2_convert_to_point_cloud(const carla_sensor_data_t *sensor_data,
                                  const char *frame_id,
                                  carla_ros2_point_cloud2_t *ros_point_cloud);

// Convert CARLA IMU data to ROS2 IMU message
carla_error_t carla_ros2_convert_to_imu(const carla_sensor_data_t *sensor_data,
                                        const char *frame_id,
                                        carla_ros2_imu_t *ros_imu);

// Convert CARLA GNSS data to ROS2 NavSatFix message
carla_error_t
carla_ros2_convert_to_nav_sat_fix(const carla_sensor_data_t *sensor_data,
                                  const char *frame_id,
                                  carla_ros2_nav_sat_fix_t *ros_gnss);

// Convert CARLA transform to ROS2 transform message
carla_error_t
carla_ros2_convert_transform(const carla_transform_t *carla_transform,
                             const char *frame_id, const char *child_frame_id,
                             carla_ros2_transform_t *ros_transform);

// Convert vehicle control from ROS2 to CARLA format
carla_error_t carla_ros2_convert_from_vehicle_control(
    const carla_ros2_vehicle_control_t *ros_control,
    carla_vehicle_control_t *carla_control);

// Topic and Publisher Management

// Get list of active ROS2 topics
carla_error_t carla_ros2_get_topics(const carla_ros2_manager_t *manager,
                                    carla_ros2_topic_info_t **topics,
                                    size_t *topic_count);

// Get topic information for specific sensor
carla_error_t
carla_ros2_get_sensor_topic_info(const carla_ros2_manager_t *manager,
                                 carla_actor_t *sensor,
                                 carla_ros2_topic_info_t *topic_info);

// Check if topic has subscribers
bool carla_ros2_topic_has_subscribers(const carla_ros2_manager_t *manager,
                                      const char *topic_name);

// Get subscriber count for topic
uint32_t carla_ros2_get_subscriber_count(const carla_ros2_manager_t *manager,
                                         const char *topic_name);

// Quality of Service Configuration

// Create default QoS configuration
carla_ros2_qos_t carla_ros2_create_default_qos(void);

// Create sensor-specific QoS configuration
carla_ros2_qos_t
carla_ros2_create_sensor_qos(carla_ros2_publisher_type_t publisher_type);

// Create reliable QoS configuration
carla_ros2_qos_t carla_ros2_create_reliable_qos(uint32_t queue_depth);

// Create best-effort QoS configuration
carla_ros2_qos_t carla_ros2_create_best_effort_qos(uint32_t queue_depth);

// Validate QoS configuration
carla_error_t carla_ros2_validate_qos(const carla_ros2_qos_t *qos);

// Configuration Helpers

// Create default ROS2 manager configuration
carla_ros2_config_t carla_ros2_create_default_config(void);

// Create default stream configuration for publisher type
carla_ros2_stream_config_t
carla_ros2_create_default_stream_config(carla_ros2_publisher_type_t type);

// Validate ROS2 configuration
carla_error_t carla_ros2_validate_config(const carla_ros2_config_t *config);

// Validate stream configuration
carla_error_t
carla_ros2_validate_stream_config(const carla_ros2_stream_config_t *config);

// Statistics and Monitoring

// Get ROS2 system statistics
carla_error_t carla_ros2_get_stats(const carla_ros2_manager_t *manager,
                                   carla_ros2_stats_t *stats);

// Reset ROS2 statistics
carla_error_t carla_ros2_reset_stats(carla_ros2_manager_t *manager);

// Get node information
carla_error_t carla_ros2_get_node_info(const carla_ros2_manager_t *manager,
                                       char *node_name, size_t name_size,
                                       char *namespace_name,
                                       size_t namespace_size);

// Callback Management

// Set connection callback for topic events
carla_error_t
carla_ros2_set_connection_callback(carla_ros2_manager_t *manager,
                                   carla_ros2_connection_callback_t callback,
                                   void *user_data);

// Remove connection callback
carla_error_t
carla_ros2_remove_connection_callback(carla_ros2_manager_t *manager);

// Set generic message callback for debugging
carla_error_t
carla_ros2_set_message_callback(carla_ros2_manager_t *manager,
                                carla_ros2_message_callback_t callback,
                                void *user_data);

// Advanced Features

// Publish custom ROS2 message
carla_error_t carla_ros2_publish_custom_message(
    carla_ros2_manager_t *manager, const char *topic_name,
    carla_ros2_message_type_t message_type, const void *message_data,
    const carla_ros2_qos_t *qos);

// Create custom publisher
carla_ros2_publisher_t *carla_ros2_create_publisher(
    carla_ros2_manager_t *manager, const char *topic_name,
    carla_ros2_message_type_t message_type, const carla_ros2_qos_t *qos);

// Destroy custom publisher
void carla_ros2_destroy_publisher(carla_ros2_publisher_t *publisher);

// Publish using custom publisher
carla_error_t carla_ros2_publisher_publish(carla_ros2_publisher_t *publisher,
                                           const void *message_data);

// Create custom subscriber
carla_ros2_subscriber_t *
carla_ros2_create_subscriber(carla_ros2_manager_t *manager,
                             const char *topic_name,
                             carla_ros2_message_type_t message_type,
                             carla_ros2_message_callback_t callback,
                             void *user_data, const carla_ros2_qos_t *qos);

// Destroy custom subscriber
void carla_ros2_destroy_subscriber(carla_ros2_subscriber_t *subscriber);

// Utilities and String Conversion

// Convert publisher type to string
const char *
carla_ros2_publisher_type_to_string(carla_ros2_publisher_type_t type);

// Convert publisher type from string
carla_ros2_publisher_type_t
carla_ros2_publisher_type_from_string(const char *type_str);

// Convert message type to string
const char *carla_ros2_message_type_to_string(carla_ros2_message_type_t type);

// Convert message type from string
carla_ros2_message_type_t
carla_ros2_message_type_from_string(const char *type_str);

// Convert QoS reliability to string
const char *
carla_ros2_qos_reliability_to_string(carla_ros2_qos_reliability_t reliability);

// Convert QoS durability to string
const char *
carla_ros2_qos_durability_to_string(carla_ros2_qos_durability_t durability);

// Generate topic name for sensor
carla_error_t
carla_ros2_generate_topic_name(const char *namespace_prefix,
                               const char *actor_name,
                               carla_ros2_publisher_type_t publisher_type,
                               char *topic_name, size_t name_size);

// Validate topic name format
bool carla_ros2_is_valid_topic_name(const char *topic_name);

// Validate frame ID format
bool carla_ros2_is_valid_frame_id(const char *frame_id);

// Memory Management

// Destroy ROS2 image message
void carla_ros2_image_destroy(carla_ros2_image_t *image);

// Destroy ROS2 point cloud message
void carla_ros2_point_cloud2_destroy(carla_ros2_point_cloud2_t *point_cloud);

// Destroy ROS2 topic info array
void carla_ros2_topic_info_array_destroy(carla_ros2_topic_info_t *topics,
                                         size_t count);

// Destroy ROS2 lane invasion message
void carla_ros2_lane_invasion_destroy(
    carla_ros2_lane_invasion_t *lane_invasion);

// Destroy ROS2 statistics
void carla_ros2_stats_destroy(carla_ros2_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_ROS2_H
