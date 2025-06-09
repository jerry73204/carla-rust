#ifndef CARLA_C_TYPES_H
#define CARLA_C_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations of opaque types
typedef struct carla_client carla_client_t;
typedef struct carla_world carla_world_t;
typedef struct carla_map carla_map_t;
typedef struct carla_actor carla_actor_t;
typedef struct carla_actor_list carla_actor_list_t;
typedef struct carla_blueprint_library carla_blueprint_library_t;
typedef struct carla_actor_blueprint carla_actor_blueprint_t;
typedef struct carla_actor_attribute carla_actor_attribute_t;
typedef struct carla_waypoint carla_waypoint_t;
typedef struct carla_waypoint_list carla_waypoint_list_t;
typedef struct carla_transform_list carla_transform_list_t;
typedef struct carla_string_list carla_string_list_t;
typedef struct carla_sensor_data carla_sensor_data_t;

// Error handling
typedef enum {
  CARLA_ERROR_NONE = 0,
  CARLA_ERROR_CONNECTION_FAILED,
  CARLA_ERROR_TIMEOUT,
  CARLA_ERROR_INVALID_ARGUMENT,
  CARLA_ERROR_NOT_FOUND,
  CARLA_ERROR_UNKNOWN
} carla_error_t;

// Map layers enum
typedef enum {
  CARLA_MAP_LAYER_NONE = 0,
  CARLA_MAP_LAYER_BUILDINGS = 0x1,
  CARLA_MAP_LAYER_DECALS = 0x1 << 1,
  CARLA_MAP_LAYER_FOLIAGE = 0x1 << 2,
  CARLA_MAP_LAYER_GROUND = 0x1 << 3,
  CARLA_MAP_LAYER_PARKED_VEHICLES = 0x1 << 4,
  CARLA_MAP_LAYER_PARTICLES = 0x1 << 5,
  CARLA_MAP_LAYER_PROPS = 0x1 << 6,
  CARLA_MAP_LAYER_STREET_LIGHTS = 0x1 << 7,
  CARLA_MAP_LAYER_WALLS = 0x1 << 8,
  CARLA_MAP_LAYER_ALL = 0xFFFF
} carla_map_layer_t;

// Basic geometry types
typedef struct {
  float x;
  float y;
  float z;
} carla_vector3d_t;

typedef struct {
  float pitch;
  float yaw;
  float roll;
} carla_rotation_t;

typedef struct {
  carla_vector3d_t location;
  carla_rotation_t rotation;
} carla_transform_t;

// Actor spawn result
typedef struct {
  carla_actor_t *actor;
  carla_error_t error;
} carla_spawn_result_t;

// Vehicle control types
typedef struct {
  float throttle;
  float steer;
  float brake;
  bool hand_brake;
  bool reverse;
  bool manual_gear_shift;
  int32_t gear;
} carla_vehicle_control_t;

typedef struct {
  float steer;
  float steer_speed;
  float speed;
  float acceleration;
  float jerk;
} carla_vehicle_ackermann_control_t;

typedef struct {
  float speed_kp;
  float speed_ki;
  float speed_kd;
  float accel_kp;
  float accel_ki;
  float accel_kd;
} carla_ackermann_controller_settings_t;

// Vehicle lighting enum (bitflags)
typedef enum {
  CARLA_VEHICLE_LIGHT_NONE = 0,
  CARLA_VEHICLE_LIGHT_POSITION = 0x1,
  CARLA_VEHICLE_LIGHT_LOW_BEAM = 0x1 << 1,
  CARLA_VEHICLE_LIGHT_HIGH_BEAM = 0x1 << 2,
  CARLA_VEHICLE_LIGHT_BRAKE = 0x1 << 3,
  CARLA_VEHICLE_LIGHT_RIGHT_BLINKER = 0x1 << 4,
  CARLA_VEHICLE_LIGHT_LEFT_BLINKER = 0x1 << 5,
  CARLA_VEHICLE_LIGHT_REVERSE = 0x1 << 6,
  CARLA_VEHICLE_LIGHT_FOG = 0x1 << 7,
  CARLA_VEHICLE_LIGHT_INTERIOR = 0x1 << 8,
  CARLA_VEHICLE_LIGHT_SPECIAL1 = 0x1 << 9, // Sirens
  CARLA_VEHICLE_LIGHT_SPECIAL2 = 0x1 << 10,
  CARLA_VEHICLE_LIGHT_ALL = 0xFFFFFFFF
} carla_vehicle_light_state_t;

// Vehicle door enum (CARLA 0.10.0)
typedef enum {
  CARLA_VEHICLE_DOOR_FL = 0, // Front Left
  CARLA_VEHICLE_DOOR_FR = 1, // Front Right
  CARLA_VEHICLE_DOOR_RL = 2, // Rear Left
  CARLA_VEHICLE_DOOR_RR = 3, // Rear Right
  CARLA_VEHICLE_DOOR_HOOD = 4,
  CARLA_VEHICLE_DOOR_TRUNK = 5,
  CARLA_VEHICLE_DOOR_ALL = 6
} carla_vehicle_door_t;

// Vehicle wheel location enum
typedef enum {
  CARLA_VEHICLE_WHEEL_FL = 0,    // Front Left
  CARLA_VEHICLE_WHEEL_FR = 1,    // Front Right
  CARLA_VEHICLE_WHEEL_BL = 2,    // Back Left
  CARLA_VEHICLE_WHEEL_BR = 3,    // Back Right
  CARLA_VEHICLE_WHEEL_FRONT = 0, // For bikes/bicycles
  CARLA_VEHICLE_WHEEL_BACK = 1
} carla_vehicle_wheel_location_t;

// Vehicle failure state enum
typedef enum {
  CARLA_VEHICLE_FAILURE_NONE = 0,
  CARLA_VEHICLE_FAILURE_ROLLOVER,
  CARLA_VEHICLE_FAILURE_ENGINE,
  CARLA_VEHICLE_FAILURE_TIRE_PUNCTURE
} carla_vehicle_failure_state_t;

// Traffic light state enum
typedef enum {
  CARLA_TRAFFIC_LIGHT_RED = 0,
  CARLA_TRAFFIC_LIGHT_YELLOW,
  CARLA_TRAFFIC_LIGHT_GREEN,
  CARLA_TRAFFIC_LIGHT_OFF,
  CARLA_TRAFFIC_LIGHT_UNKNOWN
} carla_traffic_light_state_t;

// Sensor data types enum
typedef enum {
  CARLA_SENSOR_DATA_UNKNOWN = 0,
  CARLA_SENSOR_DATA_IMAGE,
  CARLA_SENSOR_DATA_LIDAR,
  CARLA_SENSOR_DATA_SEMANTIC_LIDAR,
  CARLA_SENSOR_DATA_RADAR,
  CARLA_SENSOR_DATA_IMU,
  CARLA_SENSOR_DATA_GNSS,
  CARLA_SENSOR_DATA_COLLISION,
  CARLA_SENSOR_DATA_LANE_INVASION,
  CARLA_SENSOR_DATA_OBSTACLE_DETECTION,
  CARLA_SENSOR_DATA_DVS_EVENT_ARRAY,
  CARLA_SENSOR_DATA_OPTICAL_FLOW_IMAGE,
  CARLA_SENSOR_DATA_NORMALS_IMAGE
} carla_sensor_data_type_t;

// Basic sensor data properties (common to all sensor data)
typedef struct {
  uint64_t frame;
  double timestamp;
  carla_transform_t sensor_transform;
} carla_sensor_data_info_t;

// Image data structure
typedef struct {
  uint32_t width;
  uint32_t height;
  uint32_t fov;
  const uint8_t *raw_data;
  size_t raw_data_size;
} carla_image_data_t;

// LiDAR point structure
typedef struct {
  float x;
  float y;
  float z;
  float intensity;
} carla_lidar_detection_t;

// LiDAR data structure
typedef struct {
  const carla_lidar_detection_t *points;
  size_t point_count;
  uint32_t horizontal_angle;
  uint32_t channels;
} carla_lidar_data_t;

// Semantic LiDAR point structure
typedef struct {
  float x;
  float y;
  float z;
  float cos_inc_angle;
  uint32_t object_idx;
  uint32_t object_tag;
} carla_semantic_lidar_detection_t;

// Semantic LiDAR data structure
typedef struct {
  const carla_semantic_lidar_detection_t *points;
  size_t point_count;
  uint32_t horizontal_angle;
  uint32_t channels;
} carla_semantic_lidar_data_t;

// LiDAR processing types

// Point cloud bounds structure
typedef struct {
  carla_vector3d_t min_point; // Minimum x, y, z coordinates
  carla_vector3d_t max_point; // Maximum x, y, z coordinates
  carla_vector3d_t center;    // Center point of the cloud
  float radius;               // Radius of bounding sphere
} carla_lidar_bounds_t;

// LiDAR statistics structure
typedef struct {
  size_t total_points;
  float min_intensity;
  float max_intensity;
  float avg_intensity;
  float min_distance;
  float max_distance;
  float avg_distance;
  carla_lidar_bounds_t bounds;
} carla_lidar_stats_t;

// LiDAR filter criteria
typedef struct {
  bool use_intensity_filter;
  float min_intensity;
  float max_intensity;
  bool use_distance_filter;
  float min_distance;
  float max_distance;
  bool use_height_filter;
  float min_height;
  float max_height;
  bool use_channel_filter;
  uint32_t channel_mask; // Bitmask for channels to include
} carla_lidar_filter_t;

// Semantic LiDAR filter criteria
typedef struct {
  bool use_semantic_filter;
  uint32_t *allowed_tags; // Array of allowed semantic tags
  size_t tag_count;
  bool use_object_filter;
  uint32_t *allowed_objects; // Array of allowed object indices
  size_t object_count;
  carla_lidar_filter_t base_filter; // Base geometric filters
} carla_semantic_lidar_filter_t;

// Point cloud format for export
typedef enum {
  CARLA_LIDAR_FORMAT_PLY = 0, // PLY format
  CARLA_LIDAR_FORMAT_PCD,     // PCD format (placeholder)
  CARLA_LIDAR_FORMAT_XYZ,     // Simple XYZ text format
  CARLA_LIDAR_FORMAT_XYZI     // XYZ with intensity text format
} carla_lidar_export_format_t;

// Coordinate system for point cloud transformation
typedef enum {
  CARLA_LIDAR_COORDS_SENSOR = 0, // Sensor coordinate system
  CARLA_LIDAR_COORDS_VEHICLE,    // Vehicle coordinate system
  CARLA_LIDAR_COORDS_WORLD       // World coordinate system
} carla_lidar_coordinate_system_t;

// Downsampling method
typedef enum {
  CARLA_LIDAR_DOWNSAMPLE_UNIFORM = 0, // Uniform random sampling
  CARLA_LIDAR_DOWNSAMPLE_GRID,        // Grid-based downsampling
  CARLA_LIDAR_DOWNSAMPLE_DISTANCE     // Distance-based thinning
} carla_lidar_downsample_method_t;

// Ground segmentation result
typedef struct {
  const carla_lidar_detection_t *ground_points;
  size_t ground_point_count;
  const carla_lidar_detection_t *non_ground_points;
  size_t non_ground_point_count;
  carla_vector3d_t ground_normal; // Normal vector of ground plane
  float ground_distance;          // Distance from origin to ground plane
} carla_lidar_ground_segmentation_t;

// Obstacle cluster
typedef struct {
  const carla_lidar_detection_t *points;
  size_t point_count;
  carla_lidar_bounds_t bounds;
  carla_vector3d_t centroid;
  float confidence; // Confidence score for obstacle detection
} carla_lidar_obstacle_cluster_t;

// Obstacle detection result
typedef struct {
  carla_lidar_obstacle_cluster_t *clusters;
  size_t cluster_count;
  size_t max_clusters; // Maximum number of clusters allocated
} carla_lidar_obstacle_detection_t;

// Radar detection structure
typedef struct {
  float velocity;
  float azimuth;
  float altitude;
  float depth;
} carla_radar_detection_t;

// Radar data structure
typedef struct {
  const carla_radar_detection_t *detections;
  size_t detection_count;
} carla_radar_data_t;

// IMU measurement structure
typedef struct {
  carla_vector3d_t accelerometer; // m/s²
  carla_vector3d_t gyroscope;     // rad/s
  float compass;                  // radians
} carla_imu_data_t;

// GNSS measurement structure
typedef struct {
  double latitude;  // degrees
  double longitude; // degrees
  double altitude;  // meters
} carla_gnss_data_t;

// Collision event structure
typedef struct {
  carla_actor_t *actor;
  carla_actor_t *other_actor;
  carla_vector3d_t normal_impulse;
} carla_collision_data_t;

// Lane invasion event structure
typedef struct {
  carla_actor_t *actor;
  // Array of crossed lane markings (simplified for now)
  size_t crossed_lane_markings_count;
} carla_lane_invasion_data_t;

// Obstacle detection event structure
typedef struct {
  carla_actor_t *actor;
  carla_actor_t *other_actor;
  float distance;
} carla_obstacle_detection_data_t;

// DVS event structure
typedef struct {
  uint16_t x;
  uint16_t y;
  int64_t t; // timestamp
  bool pol;  // polarity
} carla_dvs_event_t;

// DVS event array structure
typedef struct {
  const carla_dvs_event_t *events;
  size_t event_count;
  uint32_t width;
  uint32_t height;
  uint64_t fov_angle;
} carla_dvs_event_array_data_t;

// Motion processing types

// Actor dynamic state (velocity, acceleration, angular velocity)
typedef struct {
  carla_vector3d_t velocity;         // Linear velocity m/s
  carla_vector3d_t angular_velocity; // Angular velocity rad/s
  carla_vector3d_t acceleration;     // Linear acceleration m/s²
} carla_actor_dynamic_state_t;

// Vehicle physics state
typedef struct {
  float speed;                     // Current speed m/s
  float max_rpm;                   // Maximum RPM
  float rpm;                       // Current engine RPM
  int32_t gear;                    // Current gear
  carla_vector3d_t center_of_mass; // Center of mass offset
} carla_vehicle_physics_state_t;

// Optical flow pixel structure
typedef struct {
  float u; // Horizontal motion component
  float v; // Vertical motion component
} carla_optical_flow_pixel_t;

// Optical flow image data structure
typedef struct {
  uint32_t width;
  uint32_t height;
  uint32_t fov;
  const carla_optical_flow_pixel_t *flow_data;
  size_t flow_data_size;
} carla_optical_flow_data_t;

// Motion vector (2D and 3D)
typedef struct {
  float x;
  float y;
} carla_motion_vector2d_t;

typedef struct {
  float x;
  float y;
  float z;
} carla_motion_vector3d_t;

// Motion analysis result structure
typedef struct {
  carla_motion_vector3d_t linear_motion;  // Linear motion vector
  carla_motion_vector3d_t angular_motion; // Angular motion vector
  float magnitude;                        // Motion magnitude
  float direction;                        // Motion direction (radians)
  double timestamp;                       // Timestamp of measurement
} carla_motion_analysis_t;

// Coordinate frame types for motion processing
typedef enum {
  CARLA_COORD_FRAME_WORLD = 0, // World coordinate frame
  CARLA_COORD_FRAME_ACTOR,     // Actor-relative coordinate frame
  CARLA_COORD_FRAME_SENSOR     // Sensor-relative coordinate frame
} carla_coordinate_frame_t;

// Sensor callback function type
typedef void (*carla_sensor_callback_t)(carla_sensor_data_t *data,
                                        void *user_data);

// Callback function types
typedef void (*carla_actor_destroy_callback_t)(carla_actor_t *actor,
                                               void *user_data);

// Utility functions
const char *carla_error_to_string(carla_error_t error);

// Memory management helpers
void carla_free_string(char *str);
void carla_free_string_list(carla_string_list_t *list);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_TYPES_H
