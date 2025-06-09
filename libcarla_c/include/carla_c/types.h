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
  CARLA_ERROR_NOT_IMPLEMENTED,
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

// Camera and image processing types

// Color pixel structure (BGRA format)
typedef struct {
  uint8_t b; // Blue
  uint8_t g; // Green
  uint8_t r; // Red
  uint8_t a; // Alpha
} carla_color_t;

// Float color pixel structure (for HDR images)
typedef struct {
  float r; // Red
  float g; // Green
  float b; // Blue
  float a; // Alpha
} carla_float_color_t;

// Optical flow pixel structure
typedef struct {
  float x; // Horizontal flow
  float y; // Vertical flow
} carla_optical_flow_pixel_t;

// Camera types enumeration
typedef enum {
  CARLA_CAMERA_RGB = 0,               // RGB camera
  CARLA_CAMERA_DEPTH,                 // Depth camera
  CARLA_CAMERA_SEMANTIC_SEGMENTATION, // Semantic segmentation camera
  CARLA_CAMERA_INSTANCE_SEGMENTATION, // Instance segmentation camera
  CARLA_CAMERA_DVS,                   // Dynamic Vision Sensor
  CARLA_CAMERA_OPTICAL_FLOW,          // Optical flow camera
  CARLA_CAMERA_NORMALS                // Surface normals camera
} carla_camera_type_t;

// Image format enumeration
typedef enum {
  CARLA_IMAGE_FORMAT_COLOR_BGRA = 0,  // 4-channel BGRA (32-bit)
  CARLA_IMAGE_FORMAT_DEPTH_GRAYSCALE, // 1-channel depth (8-bit grayscale)
  CARLA_IMAGE_FORMAT_DEPTH_RAW,       // Raw depth (32-bit float)
  CARLA_IMAGE_FORMAT_SEMANTIC_RAW,    // Raw semantic IDs (32-bit)
  CARLA_IMAGE_FORMAT_INSTANCE_RAW,    // Raw instance IDs (32-bit)
  CARLA_IMAGE_FORMAT_OPTICAL_FLOW,    // 2-channel optical flow (64-bit)
  CARLA_IMAGE_FORMAT_NORMALS_BGRA     // Normals as BGRA (32-bit)
} carla_image_format_t;

// Image export format
typedef enum {
  CARLA_IMAGE_EXPORT_PNG = 0, // PNG format
  CARLA_IMAGE_EXPORT_JPEG,    // JPEG format
  CARLA_IMAGE_EXPORT_BMP,     // BMP format
  CARLA_IMAGE_EXPORT_TGA,     // TGA format
  CARLA_IMAGE_EXPORT_RAW      // Raw binary format
} carla_image_export_format_t;

// Color space enumeration
typedef enum {
  CARLA_COLOR_SPACE_RGB = 0,  // RGB color space
  CARLA_COLOR_SPACE_BGR,      // BGR color space
  CARLA_COLOR_SPACE_HSV,      // HSV color space
  CARLA_COLOR_SPACE_GRAYSCALE // Grayscale
} carla_color_space_t;

// Image statistics structure
typedef struct {
  uint32_t width;              // Image width in pixels
  uint32_t height;             // Image height in pixels
  uint32_t channels;           // Number of color channels
  size_t pixel_count;          // Total number of pixels
  float fov_angle;             // Horizontal field of view in degrees
  carla_image_format_t format; // Image format

  // Color statistics (for RGB/BGRA images)
  struct {
    uint8_t min_r, min_g, min_b; // Minimum channel values
    uint8_t max_r, max_g, max_b; // Maximum channel values
    float avg_r, avg_g, avg_b;   // Average channel values
    float brightness;            // Average brightness (0-1)
    float contrast;              // Contrast measure
  } color_stats;

  // Depth statistics (for depth images)
  struct {
    float min_depth;    // Minimum depth value
    float max_depth;    // Maximum depth value
    float avg_depth;    // Average depth value
    float median_depth; // Median depth value
  } depth_stats;
} carla_image_stats_t;

// Image region of interest
typedef struct {
  uint32_t x;      // Top-left X coordinate
  uint32_t y;      // Top-left Y coordinate
  uint32_t width;  // Region width
  uint32_t height; // Region height
} carla_image_roi_t;

// Image filter types
typedef enum {
  CARLA_IMAGE_FILTER_NONE = 0,       // No filtering
  CARLA_IMAGE_FILTER_GAUSSIAN_BLUR,  // Gaussian blur
  CARLA_IMAGE_FILTER_BOX_BLUR,       // Box blur
  CARLA_IMAGE_FILTER_MEDIAN,         // Median filter
  CARLA_IMAGE_FILTER_SHARPEN,        // Sharpening filter
  CARLA_IMAGE_FILTER_EDGE_DETECTION, // Edge detection
  CARLA_IMAGE_FILTER_EMBOSS          // Emboss effect
} carla_image_filter_t;

// Image histogram structure
typedef struct {
  uint32_t bins;                 // Number of histogram bins
  uint32_t *red_histogram;       // Red channel histogram
  uint32_t *green_histogram;     // Green channel histogram
  uint32_t *blue_histogram;      // Blue channel histogram
  uint32_t *grayscale_histogram; // Grayscale histogram
} carla_image_histogram_t;

// Camera calibration parameters
typedef struct {
  float fx, fy;           // Focal lengths
  float cx, cy;           // Principal point
  float k1, k2, k3;       // Radial distortion coefficients
  float p1, p2;           // Tangential distortion coefficients
  uint32_t width, height; // Image dimensions
} carla_camera_intrinsics_t;

// Semantic segmentation class information
typedef struct {
  uint32_t class_id;      // Semantic class ID
  const char *class_name; // Human-readable class name
  carla_color_t color;    // Visualization color
  uint32_t pixel_count;   // Number of pixels in class
  float percentage;       // Percentage of image
} carla_semantic_class_info_t;

// Semantic segmentation analysis result
typedef struct {
  carla_semantic_class_info_t *classes; // Array of class information
  size_t class_count;                   // Number of classes found
  uint32_t total_pixels;                // Total pixels analyzed
  uint32_t *class_histogram;            // Histogram of class IDs
  size_t histogram_size;                // Size of histogram array
} carla_semantic_analysis_t;

// Instance segmentation result
typedef struct {
  uint32_t instance_id;           // Instance identifier
  uint32_t semantic_class;        // Semantic class of instance
  carla_image_roi_t bounding_box; // Bounding box of instance
  uint32_t pixel_count;           // Number of pixels in instance
  float area_percentage;          // Percentage of image area
  carla_vector3d_t centroid;      // 3D centroid (if depth available)
} carla_instance_info_t;

// Instance segmentation analysis result
typedef struct {
  carla_instance_info_t *instances; // Array of instance information
  size_t instance_count;            // Number of instances found
  uint32_t total_pixels;            // Total pixels analyzed
} carla_instance_analysis_t;

// Depth image analysis result
typedef struct {
  float min_depth, max_depth;    // Depth range
  float avg_depth, median_depth; // Depth statistics
  uint32_t *depth_histogram;     // Depth histogram
  size_t histogram_bins;         // Number of histogram bins
  uint32_t invalid_pixels;       // Number of invalid depth pixels
  float depth_density;           // Percentage of valid depth pixels
} carla_depth_analysis_t;

// Optical flow analysis result
typedef struct {
  carla_vector3d_t average_flow; // Average flow vector
  float magnitude_avg;           // Average flow magnitude
  float magnitude_max;           // Maximum flow magnitude
  uint32_t moving_pixels;        // Number of pixels with significant motion
  float motion_density;          // Percentage of moving pixels
  carla_vector3d_t dominant_direction; // Dominant motion direction
} carla_optical_flow_analysis_t;

// DVS event analysis result
typedef struct {
  uint32_t positive_events;        // Number of positive events
  uint32_t negative_events;        // Number of negative events
  uint32_t total_events;           // Total number of events
  float event_rate;                // Events per second
  carla_image_roi_t active_region; // Region with most activity
  float activity_density;          // Event density
} carla_dvs_analysis_t;

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

// Detection sensor processing types

// Lane marking types enumeration
typedef enum {
  CARLA_LANE_MARKING_OTHER = 0,
  CARLA_LANE_MARKING_BROKEN,
  CARLA_LANE_MARKING_SOLID,
  CARLA_LANE_MARKING_SOLID_SOLID,
  CARLA_LANE_MARKING_SOLID_BROKEN,
  CARLA_LANE_MARKING_BROKEN_SOLID,
  CARLA_LANE_MARKING_BROKEN_BROKEN,
  CARLA_LANE_MARKING_BOTTS_DOTS,
  CARLA_LANE_MARKING_GRASS,
  CARLA_LANE_MARKING_CURB,
  CARLA_LANE_MARKING_NONE
} carla_lane_marking_type_t;

// Lane marking color enumeration
typedef enum {
  CARLA_LANE_MARKING_COLOR_STANDARD = 0, // White
  CARLA_LANE_MARKING_COLOR_BLUE = 1,
  CARLA_LANE_MARKING_COLOR_GREEN = 2,
  CARLA_LANE_MARKING_COLOR_RED = 3,
  CARLA_LANE_MARKING_COLOR_WHITE = 0,
  CARLA_LANE_MARKING_COLOR_YELLOW = 4,
  CARLA_LANE_MARKING_COLOR_OTHER = 5
} carla_lane_marking_color_t;

// Lane change permission enumeration (bitflags)
typedef enum {
  CARLA_LANE_CHANGE_NONE = 0x00,  // No lane change allowed
  CARLA_LANE_CHANGE_RIGHT = 0x01, // Right lane change allowed
  CARLA_LANE_CHANGE_LEFT = 0x02,  // Left lane change allowed
  CARLA_LANE_CHANGE_BOTH = 0x03   // Both directions allowed
} carla_lane_change_t;

// Lane marking structure
typedef struct {
  carla_lane_marking_type_t type;   // Type of lane marking
  carla_lane_marking_color_t color; // Color of lane marking
  carla_lane_change_t lane_change;  // Lane change permissions
  double width;                     // Width of the marking in meters
} carla_lane_marking_t;

// Collision severity enumeration
typedef enum {
  CARLA_COLLISION_SEVERITY_UNKNOWN = 0,
  CARLA_COLLISION_SEVERITY_MINOR,    // Low impact collision
  CARLA_COLLISION_SEVERITY_MODERATE, // Medium impact collision
  CARLA_COLLISION_SEVERITY_MAJOR,    // High impact collision
  CARLA_COLLISION_SEVERITY_CRITICAL  // Very high impact collision
} carla_collision_severity_t;

// Collision type enumeration
typedef enum {
  CARLA_COLLISION_TYPE_UNKNOWN = 0,
  CARLA_COLLISION_TYPE_VEHICLE_VEHICLE, // Vehicle to vehicle collision
  CARLA_COLLISION_TYPE_VEHICLE_STATIC,  // Vehicle to static object
  CARLA_COLLISION_TYPE_VEHICLE_WALKER,  // Vehicle to pedestrian
  CARLA_COLLISION_TYPE_WALKER_STATIC,   // Pedestrian to static object
  CARLA_COLLISION_TYPE_WALKER_WALKER    // Pedestrian to pedestrian
} carla_collision_type_t;

// Enhanced collision event structure with analysis
typedef struct {
  carla_actor_t *self_actor;          // Actor that detected the collision
  carla_actor_t *other_actor;         // Actor involved in collision
  carla_vector3d_t normal_impulse;    // Normal impulse vector (N⋅s)
  carla_vector3d_t impact_location;   // Location of impact in world coordinates
  carla_vector3d_t relative_velocity; // Relative velocity at impact (m/s)
  float impact_speed;                 // Speed at impact (m/s)
  float impact_energy;                // Estimated kinetic energy (J)
  carla_collision_severity_t severity;   // Collision severity assessment
  carla_collision_type_t collision_type; // Type of collision
  double timestamp;                      // Timestamp of collision
  uint64_t frame_number;                 // Frame number when collision occurred
} carla_collision_event_t;

// Enhanced lane invasion event structure with analysis
typedef struct {
  carla_actor_t *actor;                   // Actor that invaded the lane
  carla_lane_marking_t *crossed_markings; // Array of crossed lane markings
  size_t marking_count;                   // Number of crossed markings
  carla_vector3d_t invasion_location;     // Location where invasion occurred
  carla_vector3d_t invasion_velocity;     // Velocity at time of invasion
  float invasion_angle;                   // Angle of invasion (radians)
  float lateral_displacement; // Lateral displacement from lane center
  double invasion_duration;   // Duration of invasion (seconds)
  bool intentional;           // Whether invasion appears intentional
  double timestamp;           // Timestamp of invasion
  uint64_t frame_number;      // Frame number when invasion occurred
} carla_lane_invasion_event_t;

// Obstacle detection classification
typedef enum {
  CARLA_OBSTACLE_UNKNOWN = 0,
  CARLA_OBSTACLE_VEHICLE,      // Another vehicle
  CARLA_OBSTACLE_PEDESTRIAN,   // Pedestrian or walker
  CARLA_OBSTACLE_CYCLIST,      // Bicycle or motorcycle
  CARLA_OBSTACLE_STATIC,       // Static object (building, pole, etc.)
  CARLA_OBSTACLE_TRAFFIC_SIGN, // Traffic sign or signal
  CARLA_OBSTACLE_DEBRIS        // Road debris or temporary obstacle
} carla_obstacle_type_t;

// Obstacle detection threat level
typedef enum {
  CARLA_THREAT_NONE = 0, // No threat detected
  CARLA_THREAT_LOW,      // Low threat level
  CARLA_THREAT_MEDIUM,   // Medium threat level
  CARLA_THREAT_HIGH,     // High threat level
  CARLA_THREAT_CRITICAL  // Critical threat requiring immediate action
} carla_threat_level_t;

// Enhanced obstacle detection event structure
typedef struct {
  carla_actor_t *self_actor;           // Actor that detected the obstacle
  carla_actor_t *obstacle_actor;       // Detected obstacle actor
  float distance;                      // Distance to obstacle (meters)
  carla_vector3d_t obstacle_location;  // Obstacle location in world coordinates
  carla_vector3d_t obstacle_velocity;  // Obstacle velocity (m/s)
  carla_vector3d_t relative_position;  // Relative position vector
  carla_vector3d_t relative_velocity;  // Relative velocity vector
  float closing_speed;                 // Speed at which objects are approaching
  float time_to_collision;             // Estimated time to collision (seconds)
  carla_obstacle_type_t obstacle_type; // Type of detected obstacle
  carla_threat_level_t threat_level;   // Assessed threat level
  float detection_confidence;          // Confidence in detection (0-1)
  bool in_path;                        // Whether obstacle is in travel path
  double timestamp;                    // Timestamp of detection
  uint64_t frame_number;               // Frame number when detected
} carla_obstacle_detection_event_t;

// Detection event filter criteria
typedef struct {
  bool filter_by_actor_type;     // Enable actor type filtering
  uint32_t *allowed_actor_types; // Array of allowed actor type IDs
  size_t actor_type_count;       // Number of allowed actor types

  bool filter_by_distance; // Enable distance filtering
  float min_distance;      // Minimum detection distance
  float max_distance;      // Maximum detection distance

  bool filter_by_speed; // Enable speed filtering
  float min_speed;      // Minimum speed threshold
  float max_speed;      // Maximum speed threshold

  bool filter_by_threat_level;           // Enable threat level filtering
  carla_threat_level_t min_threat_level; // Minimum threat level

  bool filter_by_location;        // Enable location-based filtering
  carla_vector3d_t filter_center; // Center point for location filter
  float filter_radius;            // Radius for location filter

  bool filter_by_time;      // Enable time-based filtering
  double time_window_start; // Start of time window
  double time_window_end;   // End of time window
} carla_detection_filter_t;

// Detection statistics structure
typedef struct {
  uint32_t total_collisions;          // Total collision events
  uint32_t total_lane_invasions;      // Total lane invasion events
  uint32_t total_obstacle_detections; // Total obstacle detection events

  // Collision statistics
  uint32_t collisions_by_severity[5]; // Count by severity level
  uint32_t collisions_by_type[6];     // Count by collision type
  float avg_collision_speed;          // Average collision speed
  float max_collision_speed;          // Maximum collision speed

  // Lane invasion statistics
  uint32_t invasions_by_marking_type[11]; // Count by lane marking type
  float avg_invasion_angle;               // Average invasion angle
  float avg_lateral_displacement;         // Average lateral displacement

  // Obstacle detection statistics
  uint32_t detections_by_type[7];   // Count by obstacle type
  uint32_t detections_by_threat[5]; // Count by threat level
  float avg_detection_distance;     // Average detection distance
  float min_time_to_collision;      // Minimum time to collision observed

  double analysis_start_time;     // Start time of analysis period
  double analysis_end_time;       // End time of analysis period
  double total_analysis_duration; // Total duration analyzed
} carla_detection_statistics_t;

// Detection event history structure for trend analysis
typedef struct {
  carla_collision_event_t *collision_events; // Array of collision events
  size_t collision_count;                    // Number of collision events
  size_t collision_capacity;                 // Capacity of collision array

  carla_lane_invasion_event_t
      *lane_invasion_events;     // Array of lane invasion events
  size_t lane_invasion_count;    // Number of lane invasion events
  size_t lane_invasion_capacity; // Capacity of lane invasion array

  carla_obstacle_detection_event_t
      *obstacle_events;     // Array of obstacle detection events
  size_t obstacle_count;    // Number of obstacle events
  size_t obstacle_capacity; // Capacity of obstacle array

  bool auto_resize;           // Whether to auto-resize arrays
  size_t max_events_per_type; // Maximum events to store per type
} carla_detection_history_t;

// Detection sensor configuration
typedef struct {
  float detection_range;                   // Maximum detection range (meters)
  float detection_angle;                   // Detection cone angle (radians)
  float update_frequency;                  // Update frequency (Hz)
  bool enable_collision_prediction;        // Enable collision prediction
  bool enable_threat_assessment;           // Enable threat level assessment
  float collision_prediction_horizon;      // Prediction time horizon (seconds)
  carla_detection_filter_t default_filter; // Default filter settings
} carla_detection_sensor_config_t;

// Traffic Manager types

// Forward declaration of traffic manager
typedef struct carla_traffic_manager carla_traffic_manager_t;

// Road option enumeration for navigation decisions
typedef enum {
  CARLA_ROAD_OPTION_VOID = 0,              // No specific action
  CARLA_ROAD_OPTION_LEFT = 1,              // Turn left
  CARLA_ROAD_OPTION_RIGHT = 2,             // Turn right
  CARLA_ROAD_OPTION_STRAIGHT = 3,          // Go straight
  CARLA_ROAD_OPTION_LANE_FOLLOW = 4,       // Follow current lane
  CARLA_ROAD_OPTION_CHANGE_LANE_LEFT = 5,  // Change to left lane
  CARLA_ROAD_OPTION_CHANGE_LANE_RIGHT = 6, // Change to right lane
  CARLA_ROAD_OPTION_ROAD_END = 7           // End of road
} carla_road_option_t;

// Traffic manager action (road option + waypoint)
typedef struct {
  carla_road_option_t road_option; // Navigation decision
  carla_waypoint_t *waypoint;      // Target waypoint (can be NULL)
} carla_traffic_manager_action_t;

// Traffic manager vehicle configuration
typedef struct {
  float speed_percentage_difference; // Speed difference from speed limit (-100
                                     // to 100%)
  float desired_speed; // Desired speed in m/s (0 = use percentage)
  float lane_offset;   // Lane center offset in meters
  float distance_to_leading_vehicle; // Following distance in meters
  bool auto_lane_change;             // Enable automatic lane changes
  bool force_lane_change_direction;  // Force lane change direction (true=left,
                                     // false=right)
  bool force_lane_change_active;     // Whether force lane change is active
  float keep_right_percentage;       // Tendency to keep right (0-100%)
  float random_left_lane_change_percentage;  // Random left lane change
                                             // probability (0-100%)
  float random_right_lane_change_percentage; // Random right lane change
                                             // probability (0-100%)
  float percentage_running_light; // Probability of running red lights (0-100%)
  float percentage_running_sign;  // Probability of ignoring stop signs (0-100%)
  float
      percentage_ignore_walkers; // Probability of ignoring pedestrians (0-100%)
  float percentage_ignore_vehicles; // Probability of ignoring other vehicles
                                    // (0-100%)
  bool update_vehicle_lights; // Whether to update vehicle lights automatically
  bool collision_detection_enabled; // Whether collision detection is enabled
} carla_traffic_manager_vehicle_config_t;

// Traffic manager global configuration
typedef struct {
  float global_speed_percentage_difference; // Global speed difference from
                                            // speed limit
  float global_lane_offset;                 // Global lane center offset
  float global_distance_to_leading_vehicle; // Global following distance
  bool synchronous_mode;                    // Synchronous execution mode
  double synchronous_mode_timeout_ms; // Timeout for synchronous operations
  bool hybrid_physics_mode;           // Enable hybrid physics for optimization
  float hybrid_physics_radius;        // Radius for hybrid physics mode
  bool respawn_dormant_vehicles;      // Enable respawning of dormant vehicles
  float respawn_lower_bound;          // Lower bound for respawn area
  float respawn_upper_bound;          // Upper bound for respawn area
  uint64_t random_device_seed;        // Seed for random number generation
  bool osm_mode;                      // OpenStreetMap mode
  uint16_t port;                      // Traffic manager port
} carla_traffic_manager_config_t;

// Custom path (sequence of world locations)
typedef struct {
  carla_vector3d_t *locations; // Array of 3D world positions
  size_t location_count;       // Number of locations in path
  bool empty_buffer;           // Whether to clear existing buffer
} carla_traffic_manager_path_t;

// Custom route (sequence of road options)
typedef struct {
  carla_road_option_t *road_options; // Array of road options
  size_t option_count;               // Number of road options in route
  bool empty_buffer;                 // Whether to clear existing buffer
} carla_traffic_manager_route_t;

// Action buffer (planned sequence of actions)
typedef struct {
  carla_traffic_manager_action_t *actions; // Array of actions
  size_t action_count;                     // Number of actions in buffer
  size_t capacity;                         // Capacity of actions array
} carla_traffic_manager_action_buffer_t;

// Traffic manager instance information
typedef struct {
  uint16_t port;                         // Port number
  bool is_running;                       // Whether TM is running
  size_t registered_vehicle_count;       // Number of registered vehicles
  carla_traffic_manager_config_t config; // Current configuration
} carla_traffic_manager_info_t;

// Traffic manager statistics
typedef struct {
  uint32_t total_registered_vehicles;   // Total vehicles ever registered
  uint32_t active_vehicle_count;        // Currently active vehicles
  uint32_t total_ticks;                 // Total simulation ticks processed
  double average_tick_time_ms;          // Average tick processing time
  uint32_t collision_count;             // Number of collisions detected
  uint32_t lane_change_count;           // Number of lane changes performed
  uint32_t traffic_light_violations;    // Number of red light violations
  uint32_t stop_sign_violations;        // Number of stop sign violations
  double total_simulation_time_seconds; // Total simulation time
} carla_traffic_manager_stats_t;

// Traffic manager constants
#define CARLA_TM_DEFAULT_PORT 8000
#define CARLA_TM_HYBRID_MODE_DT 0.05f
#define CARLA_TM_MINIMUM_STOP_TIME 2.0f
#define CARLA_TM_EXIT_JUNCTION_THRESHOLD 0.0f
#define CARLA_TM_MAX_VEHICLES_PER_TM 500
#define CARLA_TM_DEFAULT_SYNC_TIMEOUT_MS 2000.0

// OpenDRIVE generation and road network types

// Forward declarations for OpenDRIVE
typedef struct carla_opendrive_map carla_opendrive_map_t;
typedef struct carla_road carla_road_t;
typedef struct carla_lane carla_lane_t;
typedef struct carla_junction carla_junction_t;
typedef struct carla_road_geometry carla_road_geometry_t;

// OpenDRIVE generation parameters
typedef struct {
  double vertex_distance;            // Distance between vertices in meters
  double max_road_length;            // Maximum road length for chunking
  double wall_height;                // Height of generated walls
  double additional_width;           // Additional width for roads
  double vertex_width_resolution;    // Resolution for width calculations
  float simplification_percentage;   // Mesh simplification percentage (0-100)
  bool smooth_junctions;             // Enable junction smoothing
  bool enable_mesh_visibility;       // Enable mesh visibility optimization
  bool enable_pedestrian_navigation; // Enable pedestrian navigation mesh
} carla_opendrive_generation_params_t;

// Road and lane identification types
typedef uint32_t carla_road_id_t;
typedef int32_t carla_junction_id_t;
typedef int32_t carla_lane_id_t;
typedef uint32_t carla_section_id_t;
typedef uint32_t carla_connection_id_t;

// Road geometry types
typedef enum {
  CARLA_GEOMETRY_LINE = 0,      // Straight line geometry
  CARLA_GEOMETRY_ARC = 1,       // Circular arc geometry
  CARLA_GEOMETRY_SPIRAL = 2,    // Clothoid spiral geometry
  CARLA_GEOMETRY_POLY3 = 3,     // Cubic polynomial geometry
  CARLA_GEOMETRY_POLY3PARAM = 4 // Parametric cubic polynomial
} carla_road_geometry_type_t;

// Lane types (bitflags)
typedef enum {
  CARLA_LANE_TYPE_NONE = 0x1,            // No specific lane type
  CARLA_LANE_TYPE_DRIVING = 0x2,         // Driving lane
  CARLA_LANE_TYPE_STOP = 0x4,            // Stop lane
  CARLA_LANE_TYPE_SHOULDER = 0x8,        // Shoulder lane
  CARLA_LANE_TYPE_BIKING = 0x10,         // Bicycle lane
  CARLA_LANE_TYPE_SIDEWALK = 0x20,       // Sidewalk
  CARLA_LANE_TYPE_BORDER = 0x40,         // Border lane
  CARLA_LANE_TYPE_RESTRICTED = 0x80,     // Restricted access lane
  CARLA_LANE_TYPE_PARKING = 0x100,       // Parking lane
  CARLA_LANE_TYPE_BIDIRECTIONAL = 0x200, // Bidirectional lane
  CARLA_LANE_TYPE_MEDIAN = 0x400,        // Median strip
  CARLA_LANE_TYPE_SPECIAL1 = 0x800,      // Special purpose lane 1
  CARLA_LANE_TYPE_SPECIAL2 = 0x1000,     // Special purpose lane 2
  CARLA_LANE_TYPE_SPECIAL3 = 0x2000,     // Special purpose lane 3
  CARLA_LANE_TYPE_ROAD_WORKS = 0x4000,   // Road works lane
  CARLA_LANE_TYPE_TRAM = 0x8000,         // Tram lane
  CARLA_LANE_TYPE_RAIL = 0x10000,        // Railway
  CARLA_LANE_TYPE_ENTRY = 0x20000,       // Highway entry lane
  CARLA_LANE_TYPE_EXIT = 0x40000,        // Highway exit lane
  CARLA_LANE_TYPE_OFF_RAMP = 0x80000,    // Off-ramp
  CARLA_LANE_TYPE_ON_RAMP = 0x100000,    // On-ramp
  CARLA_LANE_TYPE_ANY = -2               // Any lane type (used for queries)
} carla_lane_type_t;

// OpenDRIVE lane marking types (separate from detection sensor types)
typedef enum {
  CARLA_OPENDRIVE_LANE_MARKING_NONE = 0,
  CARLA_OPENDRIVE_LANE_MARKING_SOLID,
  CARLA_OPENDRIVE_LANE_MARKING_BROKEN,
  CARLA_OPENDRIVE_LANE_MARKING_SOLID_SOLID,
  CARLA_OPENDRIVE_LANE_MARKING_SOLID_BROKEN,
  CARLA_OPENDRIVE_LANE_MARKING_BROKEN_SOLID,
  CARLA_OPENDRIVE_LANE_MARKING_BROKEN_BROKEN,
  CARLA_OPENDRIVE_LANE_MARKING_BOTTS_DOTS,
  CARLA_OPENDRIVE_LANE_MARKING_GRASS,
  CARLA_OPENDRIVE_LANE_MARKING_CURB
} carla_opendrive_lane_marking_type_t;

// OpenDRIVE lane marking color (separate from detection sensor types)
typedef enum {
  CARLA_OPENDRIVE_LANE_MARKING_COLOR_STANDARD = 0, // White
  CARLA_OPENDRIVE_LANE_MARKING_COLOR_BLUE = 1,
  CARLA_OPENDRIVE_LANE_MARKING_COLOR_GREEN = 2,
  CARLA_OPENDRIVE_LANE_MARKING_COLOR_RED = 3,
  CARLA_OPENDRIVE_LANE_MARKING_COLOR_WHITE = 4,
  CARLA_OPENDRIVE_LANE_MARKING_COLOR_YELLOW = 5,
  CARLA_OPENDRIVE_LANE_MARKING_COLOR_OTHER = 6
} carla_opendrive_lane_marking_color_t;

// Road geometry structure
typedef struct {
  carla_road_geometry_type_t type; // Type of geometry
  double s;                        // Start position along road
  double x, y;                     // Start coordinates
  double hdg;                      // Start heading (radians)
  double length;                   // Length of geometry

  // Geometry-specific parameters
  union {
    struct {
      // No additional parameters for line
    } line;

    struct {
      double curvature; // Curvature (1/radius)
    } arc;

    struct {
      double curv_start; // Start curvature
      double curv_end;   // End curvature
    } spiral;

    struct {
      double a, b, c, d; // Polynomial coefficients
    } poly3;
  } params;
} carla_road_geometry_data_t;

// OpenDRIVE lane marking structure
typedef struct {
  carla_opendrive_lane_marking_type_t type;   // Type of lane marking
  carla_opendrive_lane_marking_color_t color; // Color of lane marking
  double width;                               // Width of marking in meters
  double s_offset;                            // Offset along lane centerline
  double t_offset; // Lateral offset from lane centerline
} carla_lane_marking_data_t;

// Lane width entry structure
typedef struct {
  double s_offset;   // Position along lane where width applies
  double a, b, c, d; // Width polynomial coefficients
} carla_lane_width_data_t;

// Lane data structure
typedef struct {
  carla_lane_id_t id;          // Lane ID
  carla_lane_type_t type;      // Lane type
  bool level;                  // Whether lane is level
  carla_lane_id_t predecessor; // Predecessor lane ID
  carla_lane_id_t successor;   // Successor lane ID

  // Lane markings (left and right)
  carla_lane_marking_data_t left_marking;
  carla_lane_marking_data_t right_marking;

  // Lane width information
  carla_lane_width_data_t *width_entries;
  size_t width_entry_count;

  // Speed restrictions
  double speed_limit; // Speed limit in m/s (0 = no limit)
} carla_lane_data_t;

// Lane section structure
typedef struct {
  carla_section_id_t id;    // Section ID
  double s;                 // Start position along road
  carla_lane_data_t *lanes; // Array of lanes in section
  size_t lane_count;        // Number of lanes
} carla_lane_section_data_t;

// Road structure
typedef struct {
  carla_road_id_t id;              // Road ID
  char *name;                      // Road name
  double length;                   // Total road length
  carla_junction_id_t junction_id; // Junction ID (or -1 if not in junction)

  // Road geometry
  carla_road_geometry_data_t *geometries;
  size_t geometry_count;

  // Lane sections
  carla_lane_section_data_t *sections;
  size_t section_count;

  // Road links
  carla_road_id_t predecessor; // Predecessor road ID
  carla_road_id_t successor;   // Successor road ID
} carla_road_data_t;

// Lane link for junctions
typedef struct {
  carla_lane_id_t from; // Source lane ID
  carla_lane_id_t to;   // Target lane ID
} carla_lane_link_t;

// Junction connection
typedef struct {
  carla_connection_id_t id;        // Connection ID
  carla_road_id_t incoming_road;   // Incoming road ID
  carla_road_id_t connecting_road; // Connecting road ID
  carla_lane_link_t *lane_links;   // Array of lane links
  size_t lane_link_count;          // Number of lane links
} carla_junction_connection_t;

// Junction structure
typedef struct {
  carla_junction_id_t id;                   // Junction ID
  char *name;                               // Junction name
  carla_junction_connection_t *connections; // Array of connections
  size_t connection_count;                  // Number of connections
} carla_junction_data_t;

// OpenDRIVE map data structure
typedef struct {
  char *name;     // Map name
  double version; // OpenDRIVE version

  // Map bounds
  carla_vector3d_t min_bounds; // Minimum coordinates
  carla_vector3d_t max_bounds; // Maximum coordinates

  // Road network elements
  carla_road_data_t *roads; // Array of roads
  size_t road_count;        // Number of roads

  carla_junction_data_t *junctions; // Array of junctions
  size_t junction_count;            // Number of junctions

  // Raw OpenDRIVE content
  char *opendrive_content; // Full OpenDRIVE XML content
  size_t content_length;   // Length of content
} carla_opendrive_map_data_t;

// Waypoint structure for navigation
typedef struct {
  carla_road_id_t road_id;       // Road ID
  carla_section_id_t section_id; // Section ID
  carla_lane_id_t lane_id;       // Lane ID
  double s;                      // Position along road (meters)
  carla_transform_t transform;   // World transform at waypoint
  bool is_junction;              // Whether waypoint is in junction
} carla_waypoint_data_t;

// Waypoint list for navigation paths
typedef struct {
  carla_waypoint_data_t *waypoints; // Array of waypoints
  size_t waypoint_count;            // Number of waypoints
  double total_distance;            // Total path distance
} carla_waypoint_path_t;

// Topology pair (waypoint connection)
typedef struct {
  carla_waypoint_data_t from; // Start waypoint
  carla_waypoint_data_t to;   // End waypoint
} carla_topology_pair_t;

// Map topology structure
typedef struct {
  carla_topology_pair_t *pairs; // Array of topology pairs
  size_t pair_count;            // Number of pairs
} carla_map_topology_t;

// Mesh generation options
typedef struct {
  double vertex_distance;      // Distance between mesh vertices
  float extra_width;           // Extra width for road mesh
  bool smooth_junctions;       // Smooth junction transitions
  bool generate_walls;         // Generate side walls
  double wall_height;          // Height of walls
  bool generate_crosswalks;    // Generate crosswalk meshes
  bool generate_line_markings; // Generate lane marking meshes
} carla_mesh_generation_options_t;

// Mesh data structure (simplified)
typedef struct {
  carla_vector3d_t *vertices;  // Array of vertices
  size_t vertex_count;         // Number of vertices
  uint32_t *indices;           // Array of triangle indices
  size_t index_count;          // Number of indices
  carla_vector3d_t *normals;   // Array of vertex normals
  carla_vector3d_t bounds_min; // Minimum bounds
  carla_vector3d_t bounds_max; // Maximum bounds
} carla_mesh_data_t;

// OpenDRIVE constants
#define CARLA_OPENDRIVE_DEFAULT_VERTEX_DISTANCE 2.0
#define CARLA_OPENDRIVE_DEFAULT_MAX_ROAD_LENGTH 50.0
#define CARLA_OPENDRIVE_DEFAULT_WALL_HEIGHT 1.0
#define CARLA_OPENDRIVE_DEFAULT_ADDITIONAL_WIDTH 0.6
#define CARLA_OPENDRIVE_DEFAULT_VERTEX_WIDTH_RESOLUTION 4.0
#define CARLA_OPENDRIVE_DEFAULT_SIMPLIFICATION_PERCENTAGE 20.0
#define CARLA_OPENDRIVE_INVALID_ROAD_ID ((carla_road_id_t)-1)
#define CARLA_OPENDRIVE_INVALID_JUNCTION_ID ((carla_junction_id_t)-1)
#define CARLA_OPENDRIVE_INVALID_LANE_ID ((carla_lane_id_t)0)

// Multi-sensor fusion types

// Forward declarations for sensor fusion
typedef struct carla_sensor_fusion_system carla_sensor_fusion_system_t;
typedef struct carla_sensor_sync_buffer carla_sensor_sync_buffer_t;
typedef struct carla_sensor_calibration carla_sensor_calibration_t;
typedef struct carla_fused_object carla_fused_object_t;
typedef struct carla_fused_object_list carla_fused_object_list_t;

// Sensor fusion algorithm types
typedef enum {
  CARLA_FUSION_ALGORITHM_NAIVE = 0,        // Simple concatenation
  CARLA_FUSION_ALGORITHM_KALMAN_FILTER,    // Kalman filter fusion
  CARLA_FUSION_ALGORITHM_PARTICLE_FILTER,  // Particle filter fusion
  CARLA_FUSION_ALGORITHM_WEIGHTED_AVERAGE, // Weighted average fusion
  CARLA_FUSION_ALGORITHM_BAYESIAN,         // Bayesian fusion
  CARLA_FUSION_ALGORITHM_NEURAL_NETWORK    // Neural network fusion
} carla_fusion_algorithm_t;

// Sensor fusion quality metrics
typedef enum {
  CARLA_FUSION_QUALITY_UNKNOWN = 0,
  CARLA_FUSION_QUALITY_POOR,     // Low confidence fusion
  CARLA_FUSION_QUALITY_FAIR,     // Moderate confidence fusion
  CARLA_FUSION_QUALITY_GOOD,     // High confidence fusion
  CARLA_FUSION_QUALITY_EXCELLENT // Very high confidence fusion
} carla_fusion_quality_t;

// Sensor synchronization method
typedef enum {
  CARLA_SYNC_METHOD_EXACT_TIME = 0, // Exact timestamp match
  CARLA_SYNC_METHOD_NEAREST_TIME,   // Nearest timestamp within tolerance
  CARLA_SYNC_METHOD_INTERPOLATION,  // Linear interpolation between frames
  CARLA_SYNC_METHOD_EXTRAPOLATION,  // Extrapolation beyond available data
  CARLA_SYNC_METHOD_FRAME_NUMBER    // Frame number-based synchronization
} carla_sync_method_t;

// Sensor calibration status
typedef enum {
  CARLA_CALIBRATION_STATUS_UNKNOWN = 0,
  CARLA_CALIBRATION_STATUS_UNCALIBRATED,
  CARLA_CALIBRATION_STATUS_MANUAL,    // Manually set calibration
  CARLA_CALIBRATION_STATUS_AUTO,      // Automatically computed calibration
  CARLA_CALIBRATION_STATUS_VALIDATED, // Calibration validated
  CARLA_CALIBRATION_STATUS_INVALID    // Calibration invalid/outdated
} carla_calibration_status_t;

// Fused object classification
typedef enum {
  CARLA_FUSED_OBJECT_UNKNOWN = 0,
  CARLA_FUSED_OBJECT_VEHICLE,
  CARLA_FUSED_OBJECT_PEDESTRIAN,
  CARLA_FUSED_OBJECT_CYCLIST,
  CARLA_FUSED_OBJECT_TRAFFIC_SIGN,
  CARLA_FUSED_OBJECT_TRAFFIC_LIGHT,
  CARLA_FUSED_OBJECT_BUILDING,
  CARLA_FUSED_OBJECT_VEGETATION,
  CARLA_FUSED_OBJECT_ROAD_SURFACE,
  CARLA_FUSED_OBJECT_OBSTACLE,
  CARLA_FUSED_OBJECT_DEBRIS
} carla_fused_object_type_t;

// Sensor modality identification
typedef enum {
  CARLA_SENSOR_MODALITY_CAMERA = 0x1,
  CARLA_SENSOR_MODALITY_LIDAR = 0x2,
  CARLA_SENSOR_MODALITY_RADAR = 0x4,
  CARLA_SENSOR_MODALITY_IMU = 0x8,
  CARLA_SENSOR_MODALITY_GNSS = 0x10,
  CARLA_SENSOR_MODALITY_COLLISION = 0x20,
  CARLA_SENSOR_MODALITY_LANE_INVASION = 0x40,
  CARLA_SENSOR_MODALITY_OBSTACLE_DETECTION = 0x80,
  CARLA_SENSOR_MODALITY_DVS = 0x100,
  CARLA_SENSOR_MODALITY_OPTICAL_FLOW = 0x200
} carla_sensor_modality_t;

// Sensor data synchronization configuration
typedef struct {
  double time_tolerance_ms;         // Maximum time difference for sync (ms)
  carla_sync_method_t sync_method;  // Synchronization method to use
  uint32_t buffer_size;             // Size of sync buffer per sensor
  bool enable_interpolation;        // Enable data interpolation
  bool enable_extrapolation;        // Enable data extrapolation
  double max_interpolation_gap_ms;  // Maximum gap for interpolation (ms)
  double max_extrapolation_time_ms; // Maximum extrapolation time (ms)
  bool drop_late_data;              // Drop data arriving too late
} carla_sensor_sync_config_t;

// Sensor calibration parameters
typedef struct {
  carla_transform_t extrinsic; // Sensor pose relative to vehicle
  carla_camera_intrinsics_t
      intrinsic; // Camera intrinsic parameters (if applicable)
  carla_calibration_status_t status;  // Calibration status
  double timestamp;                   // Calibration timestamp
  char *calibration_method;           // Method used for calibration
  float calibration_error;            // Estimated calibration error
  carla_vector3d_t mounting_position; // Physical mounting position
  carla_rotation_t mounting_rotation; // Physical mounting rotation
} carla_sensor_calibration_data_t;

// Synchronized sensor data packet
typedef struct {
  double timestamp;           // Synchronized timestamp
  uint64_t frame_number;      // Frame number
  uint32_t sensor_modalities; // Bitmask of available sensor modalities

  // Sensor data pointers (NULL if not available)
  carla_sensor_data_t *camera_data;
  carla_sensor_data_t *lidar_data;
  carla_sensor_data_t *radar_data;
  carla_sensor_data_t *imu_data;
  carla_sensor_data_t *gnss_data;
  carla_sensor_data_t *collision_data;
  carla_sensor_data_t *lane_invasion_data;
  carla_sensor_data_t *obstacle_detection_data;
  carla_sensor_data_t *dvs_data;
  carla_sensor_data_t *optical_flow_data;

  // Fusion metadata
  carla_fusion_quality_t fusion_quality;
  double sync_error_ms;   // Synchronization error
  size_t dropped_sensors; // Number of sensors dropped due to sync
} carla_synchronized_sensor_data_t;

// Fused object detection result
typedef struct {
  uint32_t object_id;             // Unique object identifier
  carla_fused_object_type_t type; // Object classification
  carla_vector3d_t position;      // 3D world position
  carla_vector3d_t velocity;      // 3D velocity vector
  carla_vector3d_t size;          // Object bounding box size
  carla_rotation_t orientation;   // Object orientation

  // Confidence and quality metrics
  float detection_confidence;            // Overall detection confidence (0-1)
  float tracking_confidence;             // Tracking confidence (0-1)
  carla_fusion_quality_t fusion_quality; // Quality of sensor fusion

  // Contributing sensor modalities
  uint32_t sensor_modalities; // Bitmask of contributing sensors
  float sensor_weights[10];   // Weight of each sensor modality

  // Temporal information
  double first_detection_time; // Time of first detection
  double last_update_time;     // Time of last update
  uint32_t detection_count;    // Number of detections

  // Additional properties
  carla_color_t color;           // Object color (from camera)
  float reflectivity;            // Object reflectivity (from LiDAR/radar)
  carla_vector3d_t *point_cloud; // Associated point cloud points
  size_t point_count;            // Number of point cloud points
} carla_fused_object_data_t;

// Fused object list container
typedef struct {
  carla_fused_object_data_t *objects; // Array of fused objects
  size_t object_count;                // Number of objects
  size_t capacity;                    // Array capacity
  double timestamp;                   // Fusion timestamp
  carla_fusion_algorithm_t algorithm; // Fusion algorithm used
  uint32_t contributing_sensors;      // Sensors that contributed
} carla_fused_object_list_data_t;

// Camera-LiDAR fusion configuration
typedef struct {
  bool enable_point_colorization; // Colorize point cloud with camera
  bool enable_depth_completion;   // Complete sparse depth maps
  bool enable_occlusion_handling; // Handle occlusions between sensors
  float min_depth_range;          // Minimum depth for fusion (meters)
  float max_depth_range;          // Maximum depth for fusion (meters)
  float depth_noise_threshold;    // Depth noise filtering threshold
  carla_image_roi_t roi;          // Region of interest for fusion
} carla_camera_lidar_fusion_config_t;

// Camera-LiDAR fusion result
typedef struct {
  carla_vector3d_t *colored_points;   // Point cloud with RGB colors
  size_t point_count;                 // Number of colored points
  float *depth_map;                   // Dense depth map from LiDAR
  uint32_t depth_width;               // Depth map width
  uint32_t depth_height;              // Depth map height
  carla_image_data_t *enhanced_image; // Enhanced camera image
  carla_fusion_quality_t quality;     // Fusion quality assessment
} carla_camera_lidar_fusion_result_t;

// Multi-sensor localization state
typedef struct {
  carla_vector3d_t position;         // Estimated position
  carla_vector3d_t velocity;         // Estimated velocity
  carla_vector3d_t acceleration;     // Estimated acceleration
  carla_rotation_t orientation;      // Estimated orientation
  carla_vector3d_t angular_velocity; // Estimated angular velocity

  // Uncertainty covariance matrices (6x6 for pose, 3x3 for others)
  float position_covariance[9];    // Position uncertainty (3x3)
  float orientation_covariance[9]; // Orientation uncertainty (3x3)
  float velocity_covariance[9];    // Velocity uncertainty (3x3)

  // Quality metrics
  float localization_confidence; // Overall localization confidence
  uint32_t contributing_sensors; // Sensors contributing to estimate
  double timestamp;              // State timestamp
  uint64_t update_count;         // Number of updates
} carla_localization_state_t;

// Kalman filter configuration for localization
typedef struct {
  float process_noise_position;     // Process noise for position
  float process_noise_velocity;     // Process noise for velocity
  float process_noise_acceleration; // Process noise for acceleration
  float process_noise_orientation;  // Process noise for orientation

  float measurement_noise_gnss;      // GNSS measurement noise
  float measurement_noise_imu_accel; // IMU acceleration noise
  float measurement_noise_imu_gyro;  // IMU gyroscope noise
  float measurement_noise_odometry;  // Odometry measurement noise

  float initial_position_uncertainty; // Initial position uncertainty
  float initial_velocity_uncertainty; // Initial velocity uncertainty

  bool enable_outlier_rejection; // Enable measurement outlier rejection
  float outlier_threshold;       // Outlier rejection threshold
} carla_localization_filter_config_t;

// Sensor fusion system statistics
typedef struct {
  uint64_t total_sensor_frames; // Total sensor frames processed
  uint64_t synchronized_frames; // Successfully synchronized frames
  uint64_t dropped_frames;      // Frames dropped due to sync issues
  uint64_t interpolated_frames; // Frames requiring interpolation
  uint64_t extrapolated_frames; // Frames requiring extrapolation

  double average_sync_error_ms; // Average synchronization error
  double max_sync_error_ms;     // Maximum synchronization error
  double min_sync_error_ms;     // Minimum synchronization error

  uint64_t total_fused_objects;  // Total objects detected via fusion
  double average_fusion_time_ms; // Average fusion computation time
  double max_fusion_time_ms;     // Maximum fusion computation time

  // Per-sensor statistics
  uint64_t frames_per_sensor[10];   // Frames per sensor modality
  double latency_per_sensor[10];    // Average latency per sensor
  float reliability_per_sensor[10]; // Reliability score per sensor
} carla_sensor_fusion_stats_t;

// Sensor fusion system configuration
typedef struct {
  carla_sensor_sync_config_t sync_config;    // Synchronization configuration
  carla_fusion_algorithm_t fusion_algorithm; // Primary fusion algorithm
  carla_camera_lidar_fusion_config_t
      camera_lidar_config; // Camera-LiDAR fusion config
  carla_localization_filter_config_t
      localization_config; // Localization filter config

  bool enable_object_tracking;        // Enable multi-sensor object tracking
  bool enable_localization_fusion;    // Enable localization fusion
  bool enable_performance_monitoring; // Enable performance statistics

  float object_association_threshold; // Object association distance threshold
  float tracking_persistence_time;    // Time to keep lost tracks (seconds)
  uint32_t max_tracked_objects;       // Maximum number of tracked objects

  char *output_coordinate_frame; // Output coordinate frame ("world", "vehicle",
                                 // "sensor")
  bool enable_debug_output;      // Enable debug information output
} carla_sensor_fusion_config_t;

// Sensor fusion callback function types
typedef void (*carla_fusion_callback_t)(
    const carla_synchronized_sensor_data_t *data, void *user_data);
typedef void (*carla_object_detection_callback_t)(
    const carla_fused_object_list_data_t *objects, void *user_data);
typedef void (*carla_localization_callback_t)(
    const carla_localization_state_t *state, void *user_data);

// Sensor fusion constants
#define CARLA_FUSION_MAX_SENSORS 10
#define CARLA_FUSION_DEFAULT_BUFFER_SIZE 100
#define CARLA_FUSION_DEFAULT_TIME_TOLERANCE_MS 50.0
#define CARLA_FUSION_MAX_OBJECT_TRACKS 1000
#define CARLA_FUSION_INVALID_OBJECT_ID ((uint32_t)-1)

// ROS2 Native Integration Types

// Forward declarations for ROS2
typedef struct carla_ros2_manager carla_ros2_manager_t;
typedef struct carla_ros2_publisher carla_ros2_publisher_t;
typedef struct carla_ros2_subscriber carla_ros2_subscriber_t;
typedef struct carla_ros2_node carla_ros2_node_t;

// ROS2 Quality of Service policies
typedef enum {
  CARLA_ROS2_QOS_RELIABLE = 0,   // Reliable delivery
  CARLA_ROS2_QOS_BEST_EFFORT = 1 // Best effort delivery
} carla_ros2_qos_reliability_t;

typedef enum {
  CARLA_ROS2_QOS_VOLATILE = 0,        // Only latest data
  CARLA_ROS2_QOS_TRANSIENT_LOCAL = 1, // Keep last value for late joiners
  CARLA_ROS2_QOS_TRANSIENT = 2,       // Persist across restarts
  CARLA_ROS2_QOS_PERSISTENT = 3       // Persist to storage
} carla_ros2_qos_durability_t;

// ROS2 Quality of Service configuration
typedef struct {
  carla_ros2_qos_reliability_t reliability; // Reliability policy
  carla_ros2_qos_durability_t durability;   // Durability policy
  uint32_t queue_depth;                     // Queue size
  double deadline_ms;                       // Deadline in milliseconds
  double lifespan_ms;                       // Lifespan in milliseconds
  double liveliness_lease_duration_ms;      // Liveliness lease duration
} carla_ros2_qos_t;

// ROS2 publisher types (corresponding to CARLA sensor types)
typedef enum {
  CARLA_ROS2_PUBLISHER_RGB_CAMERA = 0,      // RGB camera publisher
  CARLA_ROS2_PUBLISHER_DEPTH_CAMERA,        // Depth camera publisher
  CARLA_ROS2_PUBLISHER_SEMANTIC_CAMERA,     // Semantic segmentation camera
  CARLA_ROS2_PUBLISHER_INSTANCE_CAMERA,     // Instance segmentation camera
  CARLA_ROS2_PUBLISHER_NORMALS_CAMERA,      // Surface normals camera
  CARLA_ROS2_PUBLISHER_OPTICAL_FLOW_CAMERA, // Optical flow camera
  CARLA_ROS2_PUBLISHER_DVS_CAMERA,          // Dynamic Vision Sensor
  CARLA_ROS2_PUBLISHER_LIDAR,               // LiDAR point cloud
  CARLA_ROS2_PUBLISHER_SEMANTIC_LIDAR,      // Semantic LiDAR
  CARLA_ROS2_PUBLISHER_RADAR,               // Radar sensor
  CARLA_ROS2_PUBLISHER_IMU,                 // Inertial measurement unit
  CARLA_ROS2_PUBLISHER_GNSS,                // GPS/GNSS sensor
  CARLA_ROS2_PUBLISHER_COLLISION,           // Collision sensor
  CARLA_ROS2_PUBLISHER_LANE_INVASION,       // Lane invasion sensor
  CARLA_ROS2_PUBLISHER_TRANSFORM,           // Actor transform
  CARLA_ROS2_PUBLISHER_SPEEDOMETER,         // Vehicle speed
  CARLA_ROS2_PUBLISHER_CLOCK,               // Simulation clock
  CARLA_ROS2_PUBLISHER_MAP                  // Map data
} carla_ros2_publisher_type_t;

// ROS2 subscriber types
typedef enum {
  CARLA_ROS2_SUBSCRIBER_VEHICLE_CONTROL = 0, // Vehicle control commands
  CARLA_ROS2_SUBSCRIBER_GENERIC              // Generic subscriber
} carla_ros2_subscriber_type_t;

// ROS2 message types
typedef enum {
  CARLA_ROS2_MSG_IMAGE = 0,                 // sensor_msgs/Image
  CARLA_ROS2_MSG_CAMERA_INFO,               // sensor_msgs/CameraInfo
  CARLA_ROS2_MSG_POINT_CLOUD2,              // sensor_msgs/PointCloud2
  CARLA_ROS2_MSG_IMU,                       // sensor_msgs/Imu
  CARLA_ROS2_MSG_NAV_SAT_FIX,               // sensor_msgs/NavSatFix
  CARLA_ROS2_MSG_TRANSFORM,                 // geometry_msgs/Transform
  CARLA_ROS2_MSG_TWIST,                     // geometry_msgs/Twist
  CARLA_ROS2_MSG_CLOCK,                     // rosgraph_msgs/Clock
  CARLA_ROS2_MSG_CARLA_EGO_VEHICLE_CONTROL, // carla_msgs/CarlaEgoVehicleControl
  CARLA_ROS2_MSG_CARLA_COLLISION_EVENT,     // carla_msgs/CarlaCollisionEvent
  CARLA_ROS2_MSG_CARLA_LINE_INVASION        // carla_msgs/CarlaLineInvasion
} carla_ros2_message_type_t;

// ROS2 vehicle control command structure
typedef struct {
  float throttle;              // Throttle pedal (0.0 to 1.0)
  float steer;                 // Steering angle (-1.0 to 1.0)
  float brake;                 // Brake pedal (0.0 to 1.0)
  bool hand_brake;             // Hand brake engaged
  bool reverse;                // Reverse gear
  bool manual_gear_shift;      // Manual gear shift mode
  int32_t gear;                // Current gear
  double header_stamp_sec;     // ROS2 timestamp seconds
  double header_stamp_nanosec; // ROS2 timestamp nanoseconds
  char frame_id[64];           // ROS2 frame ID
} carla_ros2_vehicle_control_t;

// ROS2 transform message structure
typedef struct {
  carla_vector3d_t translation; // Translation vector
  struct {                      // Quaternion rotation
    float x, y, z, w;
  } rotation;
  double header_stamp_sec;     // ROS2 timestamp seconds
  double header_stamp_nanosec; // ROS2 timestamp nanoseconds
  char frame_id[64];           // ROS2 frame ID
  char child_frame_id[64];     // Child frame ID
} carla_ros2_transform_t;

// ROS2 twist message structure (velocity)
typedef struct {
  carla_vector3d_t linear;     // Linear velocity
  carla_vector3d_t angular;    // Angular velocity
  double header_stamp_sec;     // ROS2 timestamp seconds
  double header_stamp_nanosec; // ROS2 timestamp nanoseconds
  char frame_id[64];           // ROS2 frame ID
} carla_ros2_twist_t;

// ROS2 clock message structure
typedef struct {
  double clock_sec;     // Simulation time seconds
  double clock_nanosec; // Simulation time nanoseconds
} carla_ros2_clock_t;

// ROS2 camera info structure
typedef struct {
  uint32_t width;                 // Image width
  uint32_t height;                // Image height
  char distortion_model[32];      // Distortion model name
  double distortion_params[8];    // Distortion parameters
  double camera_matrix[9];        // 3x3 camera matrix
  double rectification_matrix[9]; // 3x3 rectification matrix
  double projection_matrix[12];   // 3x4 projection matrix
  uint32_t binning_x;             // Binning factor X
  uint32_t binning_y;             // Binning factor Y
  struct {                        // Region of interest
    uint32_t x_offset;
    uint32_t y_offset;
    uint32_t width;
    uint32_t height;
    bool do_rectify;
  } roi;
  double header_stamp_sec;     // ROS2 timestamp seconds
  double header_stamp_nanosec; // ROS2 timestamp nanoseconds
  char frame_id[64];           // ROS2 frame ID
} carla_ros2_camera_info_t;

// ROS2 point cloud field structure
typedef struct {
  char name[32];    // Field name
  uint32_t offset;  // Offset from start of point struct
  uint8_t datatype; // Data type (see ROS2 sensor_msgs)
  uint32_t count;   // Number of elements in field
} carla_ros2_point_field_t;

// ROS2 point cloud2 message structure
typedef struct {
  uint32_t width;                   // Width of point cloud
  uint32_t height;                  // Height of point cloud (1 for unorganized)
  carla_ros2_point_field_t *fields; // Point field descriptions
  size_t field_count;               // Number of fields
  bool is_bigendian;                // Byte order
  uint32_t point_step;              // Length of point in bytes
  uint32_t row_step;                // Length of row in bytes
  uint8_t *data;                    // Point cloud data
  size_t data_size;                 // Size of data array
  bool is_dense;                    // True if no invalid points
  double header_stamp_sec;          // ROS2 timestamp seconds
  double header_stamp_nanosec;      // ROS2 timestamp nanoseconds
  char frame_id[64];                // ROS2 frame ID
} carla_ros2_point_cloud2_t;

// ROS2 IMU message structure
typedef struct {
  struct { // Quaternion orientation
    float x, y, z, w;
  } orientation;
  double orientation_covariance[9];      // 3x3 orientation covariance
  carla_vector3d_t angular_velocity;     // Angular velocity (rad/s)
  double angular_velocity_covariance[9]; // 3x3 angular velocity covariance
  carla_vector3d_t linear_acceleration;  // Linear acceleration (m/s²)
  double
      linear_acceleration_covariance[9]; // 3x3 linear acceleration covariance
  double header_stamp_sec;               // ROS2 timestamp seconds
  double header_stamp_nanosec;           // ROS2 timestamp nanoseconds
  char frame_id[64];                     // ROS2 frame ID
} carla_ros2_imu_t;

// ROS2 NavSatFix message structure (GNSS)
typedef struct {
  double latitude;                  // Latitude in degrees
  double longitude;                 // Longitude in degrees
  double altitude;                  // Altitude in meters
  double position_covariance[9];    // 3x3 position covariance
  uint8_t position_covariance_type; // Covariance type
  uint8_t status;                   // GPS fix status
  uint16_t service;                 // GPS service flags
  double header_stamp_sec;          // ROS2 timestamp seconds
  double header_stamp_nanosec;      // ROS2 timestamp nanoseconds
  char frame_id[64];                // ROS2 frame ID
} carla_ros2_nav_sat_fix_t;

// ROS2 image encoding constants
#define CARLA_ROS2_IMAGE_ENCODING_RGB8 "rgb8"
#define CARLA_ROS2_IMAGE_ENCODING_RGBA8 "rgba8"
#define CARLA_ROS2_IMAGE_ENCODING_BGR8 "bgr8"
#define CARLA_ROS2_IMAGE_ENCODING_BGRA8 "bgra8"
#define CARLA_ROS2_IMAGE_ENCODING_MONO8 "mono8"
#define CARLA_ROS2_IMAGE_ENCODING_32FC1 "32FC1"

// ROS2 image message structure
typedef struct {
  uint32_t width;              // Image width
  uint32_t height;             // Image height
  char encoding[32];           // Image encoding
  uint8_t is_bigendian;        // Byte order
  uint32_t step;               // Row stride in bytes
  uint8_t *data;               // Image data
  size_t data_size;            // Size of data array
  double header_stamp_sec;     // ROS2 timestamp seconds
  double header_stamp_nanosec; // ROS2 timestamp nanoseconds
  char frame_id[64];           // ROS2 frame ID
} carla_ros2_image_t;

// ROS2 collision event message structure
typedef struct {
  carla_vector3d_t normal_impulse; // Normal impulse vector
  uint32_t other_actor_id;         // ID of other actor in collision
  char other_actor_type_id[128];   // Type ID of other actor
  double header_stamp_sec;         // ROS2 timestamp seconds
  double header_stamp_nanosec;     // ROS2 timestamp nanoseconds
  char frame_id[64];               // ROS2 frame ID
} carla_ros2_collision_event_t;

// ROS2 lane invasion event message structure
typedef struct {
  uint32_t *crossed_lane_markings; // Array of crossed lane marking types
  size_t lane_marking_count;       // Number of crossed markings
  double header_stamp_sec;         // ROS2 timestamp seconds
  double header_stamp_nanosec;     // ROS2 timestamp nanoseconds
  char frame_id[64];               // ROS2 frame ID
} carla_ros2_lane_invasion_t;

// ROS2 stream configuration for sensors
typedef struct {
  carla_ros2_publisher_type_t publisher_type; // Type of publisher
  bool enabled;                               // Whether stream is enabled
  char topic_name[256];                       // Custom topic name (optional)
  carla_ros2_qos_t qos;                       // Quality of service settings
  double publish_rate_hz;   // Publishing rate in Hz (0 = sensor rate)
  bool include_camera_info; // Include camera info for cameras
  char frame_id[64];        // ROS2 frame ID
  char parent_frame_id[64]; // Parent frame ID for transforms
} carla_ros2_stream_config_t;

// ROS2 manager configuration
typedef struct {
  bool enabled;                 // Whether ROS2 is enabled
  char node_name[128];          // ROS2 node name
  char namespace_prefix[128];   // Namespace prefix for topics
  bool use_simulation_time;     // Use simulation time instead of wall time
  double clock_publish_rate_hz; // Clock publishing rate
  bool publish_tf;              // Whether to publish transforms
  uint32_t ros_domain_id;       // ROS2 domain ID
  bool verbose_logging;         // Enable verbose logging
} carla_ros2_config_t;

// ROS2 topic information
typedef struct {
  char topic_name[256];                       // Full topic name
  carla_ros2_message_type_t message_type;     // Message type
  carla_ros2_publisher_type_t publisher_type; // Publisher type
  uint32_t subscriber_count;                  // Number of subscribers
  bool is_active;           // Whether topic is actively publishing
  double last_publish_time; // Last publish timestamp
  uint64_t message_count;   // Total messages published
} carla_ros2_topic_info_t;

// ROS2 statistics
typedef struct {
  uint64_t total_messages_published; // Total messages published
  uint64_t total_messages_received;  // Total messages received
  uint32_t active_publishers;        // Number of active publishers
  uint32_t active_subscribers;       // Number of active subscribers
  double average_publish_rate_hz;    // Average publishing rate
  double last_activity_time;         // Last activity timestamp
  uint64_t dropped_messages;         // Number of dropped messages
} carla_ros2_stats_t;

// ROS2 callback function types
typedef void (*carla_ros2_vehicle_control_callback_t)(
    const carla_ros2_vehicle_control_t *control, void *user_data);

typedef void (*carla_ros2_message_callback_t)(const void *message,
                                              carla_ros2_message_type_t type,
                                              void *user_data);

typedef void (*carla_ros2_connection_callback_t)(const char *topic_name,
                                                 bool connected,
                                                 void *user_data);

// ROS2 constants
#define CARLA_ROS2_DEFAULT_NODE_NAME "carla_ros2_bridge"
#define CARLA_ROS2_DEFAULT_NAMESPACE "/carla"
#define CARLA_ROS2_DEFAULT_DOMAIN_ID 0
#define CARLA_ROS2_DEFAULT_CLOCK_RATE_HZ 100.0
#define CARLA_ROS2_MAX_TOPIC_NAME_LENGTH 256
#define CARLA_ROS2_MAX_FRAME_ID_LENGTH 64
#define CARLA_ROS2_MAX_PUBLISHERS 50
#define CARLA_ROS2_MAX_SUBSCRIBERS 10

// Memory management helpers
void carla_free_string(char *str);
void carla_free_string_list(carla_string_list_t *list);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_TYPES_H
