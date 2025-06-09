#include "carla_c/ros2.h"
#include "internal.h"

#include <cstring>

// Stub implementation for when ROS2 is not available
// All functions return NOT_IMPLEMENTED error

extern "C" {

// ROS2 Manager Lifecycle
carla_ros2_manager_t *carla_ros2_create(const carla_ros2_config_t *config) {
  (void)config;
  return nullptr;
}

void carla_ros2_destroy(carla_ros2_manager_t *manager) { (void)manager; }

carla_error_t carla_ros2_set_enabled(carla_ros2_manager_t *manager,
                                     bool enabled) {
  (void)manager;
  (void)enabled;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

bool carla_ros2_is_enabled(const carla_ros2_manager_t *manager) {
  (void)manager;
  return false;
}

carla_error_t carla_ros2_configure(carla_ros2_manager_t *manager,
                                   const carla_ros2_config_t *config) {
  (void)manager;
  (void)config;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_get_config(const carla_ros2_manager_t *manager,
                                    carla_ros2_config_t *config) {
  (void)manager;
  (void)config;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

// ROS2 Time and Frame Management
carla_error_t carla_ros2_set_timestamp(carla_ros2_manager_t *manager,
                                       double timestamp_seconds) {
  (void)manager;
  (void)timestamp_seconds;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_set_frame_number(carla_ros2_manager_t *manager,
                                          uint64_t frame_number) {
  (void)manager;
  (void)frame_number;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_publish_clock(carla_ros2_manager_t *manager) {
  (void)manager;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

// Actor and Sensor Management
carla_error_t carla_ros2_add_actor(carla_ros2_manager_t *manager,
                                   carla_actor_t *actor, const char *ros_name) {
  (void)manager;
  (void)actor;
  (void)ros_name;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_remove_actor(carla_ros2_manager_t *manager,
                                      carla_actor_t *actor) {
  (void)manager;
  (void)actor;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_get_actor_name(const carla_ros2_manager_t *manager,
                                        carla_actor_t *actor, char *ros_name,
                                        size_t name_size) {
  (void)manager;
  (void)actor;
  (void)ros_name;
  (void)name_size;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

// Stream Configuration and Control
carla_error_t
carla_ros2_enable_stream(carla_ros2_manager_t *manager, carla_actor_t *sensor,
                         const carla_ros2_stream_config_t *config) {
  (void)manager;
  (void)sensor;
  (void)config;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_disable_stream(carla_ros2_manager_t *manager,
                                        carla_actor_t *sensor) {
  (void)manager;
  (void)sensor;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

bool carla_ros2_is_stream_enabled(const carla_ros2_manager_t *manager,
                                  carla_actor_t *sensor) {
  (void)manager;
  (void)sensor;
  return false;
}

// Configuration Helpers (these can still work)
carla_ros2_config_t carla_ros2_create_default_config(void) {
  carla_ros2_config_t config;
  memset(&config, 0, sizeof(config));
  config.enabled = false; // Always disabled in stub
  strncpy(config.node_name, CARLA_ROS2_DEFAULT_NODE_NAME,
          sizeof(config.node_name) - 1);
  strncpy(config.namespace_prefix, CARLA_ROS2_DEFAULT_NAMESPACE,
          sizeof(config.namespace_prefix) - 1);
  config.use_simulation_time = true;
  config.clock_publish_rate_hz = CARLA_ROS2_DEFAULT_CLOCK_RATE_HZ;
  config.publish_tf = true;
  config.ros_domain_id = CARLA_ROS2_DEFAULT_DOMAIN_ID;
  config.verbose_logging = false;
  return config;
}

carla_ros2_stream_config_t
carla_ros2_create_default_stream_config(carla_ros2_publisher_type_t type) {
  carla_ros2_stream_config_t config;
  memset(&config, 0, sizeof(config));
  config.publisher_type = type;
  config.enabled = false; // Always disabled in stub
  config.qos = carla_ros2_create_sensor_qos(type);
  config.publish_rate_hz = 0.0;
  config.include_camera_info = true;
  strncpy(config.frame_id, "sensor", sizeof(config.frame_id) - 1);
  strncpy(config.parent_frame_id, "base_link",
          sizeof(config.parent_frame_id) - 1);
  return config;
}

carla_ros2_qos_t carla_ros2_create_default_qos(void) {
  carla_ros2_qos_t qos;
  qos.reliability = CARLA_ROS2_QOS_RELIABLE;
  qos.durability = CARLA_ROS2_QOS_VOLATILE;
  qos.queue_depth = 10;
  qos.deadline_ms = 0.0;
  qos.lifespan_ms = 0.0;
  qos.liveliness_lease_duration_ms = 0.0;
  return qos;
}

carla_ros2_qos_t
carla_ros2_create_sensor_qos(carla_ros2_publisher_type_t publisher_type) {
  (void)publisher_type;
  return carla_ros2_create_default_qos();
}

carla_ros2_qos_t carla_ros2_create_reliable_qos(uint32_t queue_depth) {
  carla_ros2_qos_t qos = carla_ros2_create_default_qos();
  qos.reliability = CARLA_ROS2_QOS_RELIABLE;
  qos.queue_depth = queue_depth;
  return qos;
}

carla_ros2_qos_t carla_ros2_create_best_effort_qos(uint32_t queue_depth) {
  carla_ros2_qos_t qos = carla_ros2_create_default_qos();
  qos.reliability = CARLA_ROS2_QOS_BEST_EFFORT;
  qos.queue_depth = queue_depth;
  return qos;
}

// String conversion utilities (these can still work)
const char *
carla_ros2_publisher_type_to_string(carla_ros2_publisher_type_t type) {
  switch (type) {
  case CARLA_ROS2_PUBLISHER_RGB_CAMERA:
    return "rgb_camera";
  case CARLA_ROS2_PUBLISHER_DEPTH_CAMERA:
    return "depth_camera";
  case CARLA_ROS2_PUBLISHER_SEMANTIC_CAMERA:
    return "semantic_camera";
  case CARLA_ROS2_PUBLISHER_INSTANCE_CAMERA:
    return "instance_camera";
  case CARLA_ROS2_PUBLISHER_NORMALS_CAMERA:
    return "normals_camera";
  case CARLA_ROS2_PUBLISHER_OPTICAL_FLOW_CAMERA:
    return "optical_flow_camera";
  case CARLA_ROS2_PUBLISHER_DVS_CAMERA:
    return "dvs_camera";
  case CARLA_ROS2_PUBLISHER_LIDAR:
    return "lidar";
  case CARLA_ROS2_PUBLISHER_SEMANTIC_LIDAR:
    return "semantic_lidar";
  case CARLA_ROS2_PUBLISHER_RADAR:
    return "radar";
  case CARLA_ROS2_PUBLISHER_IMU:
    return "imu";
  case CARLA_ROS2_PUBLISHER_GNSS:
    return "gnss";
  case CARLA_ROS2_PUBLISHER_COLLISION:
    return "collision";
  case CARLA_ROS2_PUBLISHER_LANE_INVASION:
    return "lane_invasion";
  case CARLA_ROS2_PUBLISHER_TRANSFORM:
    return "transform";
  case CARLA_ROS2_PUBLISHER_SPEEDOMETER:
    return "speedometer";
  case CARLA_ROS2_PUBLISHER_CLOCK:
    return "clock";
  case CARLA_ROS2_PUBLISHER_MAP:
    return "map";
  default:
    return "unknown";
  }
}

const char *carla_ros2_message_type_to_string(carla_ros2_message_type_t type) {
  switch (type) {
  case CARLA_ROS2_MSG_IMAGE:
    return "sensor_msgs/Image";
  case CARLA_ROS2_MSG_CAMERA_INFO:
    return "sensor_msgs/CameraInfo";
  case CARLA_ROS2_MSG_POINT_CLOUD2:
    return "sensor_msgs/PointCloud2";
  case CARLA_ROS2_MSG_IMU:
    return "sensor_msgs/Imu";
  case CARLA_ROS2_MSG_NAV_SAT_FIX:
    return "sensor_msgs/NavSatFix";
  case CARLA_ROS2_MSG_TRANSFORM:
    return "geometry_msgs/Transform";
  case CARLA_ROS2_MSG_TWIST:
    return "geometry_msgs/Twist";
  case CARLA_ROS2_MSG_CLOCK:
    return "rosgraph_msgs/Clock";
  case CARLA_ROS2_MSG_CARLA_EGO_VEHICLE_CONTROL:
    return "carla_msgs/CarlaEgoVehicleControl";
  case CARLA_ROS2_MSG_CARLA_COLLISION_EVENT:
    return "carla_msgs/CarlaCollisionEvent";
  case CARLA_ROS2_MSG_CARLA_LINE_INVASION:
    return "carla_msgs/CarlaLineInvasion";
  default:
    return "unknown";
  }
}

const char *
carla_ros2_qos_reliability_to_string(carla_ros2_qos_reliability_t reliability) {
  switch (reliability) {
  case CARLA_ROS2_QOS_RELIABLE:
    return "reliable";
  case CARLA_ROS2_QOS_BEST_EFFORT:
    return "best_effort";
  default:
    return "unknown";
  }
}

const char *
carla_ros2_qos_durability_to_string(carla_ros2_qos_durability_t durability) {
  switch (durability) {
  case CARLA_ROS2_QOS_VOLATILE:
    return "volatile";
  case CARLA_ROS2_QOS_TRANSIENT_LOCAL:
    return "transient_local";
  case CARLA_ROS2_QOS_TRANSIENT:
    return "transient";
  case CARLA_ROS2_QOS_PERSISTENT:
    return "persistent";
  default:
    return "unknown";
  }
}

// All other functions return NOT_IMPLEMENTED
carla_error_t
carla_ros2_update_stream_config(carla_ros2_manager_t *manager,
                                carla_actor_t *sensor,
                                const carla_ros2_stream_config_t *config) {
  (void)manager;
  (void)sensor;
  (void)config;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_get_stream_config(const carla_ros2_manager_t *manager,
                                           carla_actor_t *sensor,
                                           carla_ros2_stream_config_t *config) {
  (void)manager;
  (void)sensor;
  (void)config;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_publish_camera_data(carla_ros2_manager_t *manager,
                                             carla_actor_t *camera,
                                             carla_sensor_data_t *data) {
  (void)manager;
  (void)camera;
  (void)data;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_publish_lidar_data(carla_ros2_manager_t *manager,
                                            carla_actor_t *lidar,
                                            carla_sensor_data_t *data) {
  (void)manager;
  (void)lidar;
  (void)data;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_publish_imu_data(carla_ros2_manager_t *manager,
                                          carla_actor_t *imu,
                                          carla_sensor_data_t *data) {
  (void)manager;
  (void)imu;
  (void)data;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_publish_gnss_data(carla_ros2_manager_t *manager,
                                           carla_actor_t *gnss,
                                           carla_sensor_data_t *data) {
  (void)manager;
  (void)gnss;
  (void)data;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_publish_radar_data(carla_ros2_manager_t *manager,
                                            carla_actor_t *radar,
                                            carla_sensor_data_t *data) {
  (void)manager;
  (void)radar;
  (void)data;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_publish_collision_data(carla_ros2_manager_t *manager,
                                                carla_actor_t *sensor,
                                                carla_sensor_data_t *data) {
  (void)manager;
  (void)sensor;
  (void)data;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t
carla_ros2_publish_lane_invasion_data(carla_ros2_manager_t *manager,
                                      carla_actor_t *sensor,
                                      carla_sensor_data_t *data) {
  (void)manager;
  (void)sensor;
  (void)data;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_publish_transform(carla_ros2_manager_t *manager,
                                           carla_actor_t *actor,
                                           const carla_transform_t *transform) {
  (void)manager;
  (void)actor;
  (void)transform;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_publish_speedometer(carla_ros2_manager_t *manager,
                                             carla_actor_t *vehicle,
                                             float speed_mps) {
  (void)manager;
  (void)vehicle;
  (void)speed_mps;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_publish_sensor_data(carla_ros2_manager_t *manager,
                                             carla_actor_t *sensor,
                                             carla_sensor_data_t *data) {
  (void)manager;
  (void)sensor;
  (void)data;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_validate_config(const carla_ros2_config_t *config) {
  (void)config;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t
carla_ros2_validate_stream_config(const carla_ros2_stream_config_t *config) {
  (void)config;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_error_t carla_ros2_validate_qos(const carla_ros2_qos_t *qos) {
  (void)qos;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

// Memory management stubs
void carla_ros2_image_destroy(carla_ros2_image_t *image) { (void)image; }
void carla_ros2_point_cloud2_destroy(carla_ros2_point_cloud2_t *point_cloud) {
  (void)point_cloud;
}
void carla_ros2_topic_info_array_destroy(carla_ros2_topic_info_t *topics,
                                         size_t count) {
  (void)topics;
  (void)count;
}
void carla_ros2_lane_invasion_destroy(
    carla_ros2_lane_invasion_t *lane_invasion) {
  (void)lane_invasion;
}
void carla_ros2_stats_destroy(carla_ros2_stats_t *stats) { (void)stats; }

carla_ros2_publisher_type_t
carla_ros2_publisher_type_from_string(const char *type_str) {
  (void)type_str;
  return CARLA_ROS2_PUBLISHER_RGB_CAMERA;
}

carla_ros2_message_type_t
carla_ros2_message_type_from_string(const char *type_str) {
  (void)type_str;
  return CARLA_ROS2_MSG_IMAGE;
}

carla_error_t
carla_ros2_generate_topic_name(const char *namespace_prefix,
                               const char *actor_name,
                               carla_ros2_publisher_type_t publisher_type,
                               char *topic_name, size_t name_size) {
  (void)namespace_prefix;
  (void)actor_name;
  (void)publisher_type;
  (void)topic_name;
  (void)name_size;
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

bool carla_ros2_is_valid_topic_name(const char *topic_name) {
  (void)topic_name;
  return false;
}

bool carla_ros2_is_valid_frame_id(const char *frame_id) {
  (void)frame_id;
  return false;
}

} // extern "C"
