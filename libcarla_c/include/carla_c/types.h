#ifndef CARLA_C_TYPES_H
#define CARLA_C_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

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
    carla_actor_t* actor;
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
    CARLA_VEHICLE_LIGHT_NONE         = 0,
    CARLA_VEHICLE_LIGHT_POSITION     = 0x1,
    CARLA_VEHICLE_LIGHT_LOW_BEAM     = 0x1 << 1,
    CARLA_VEHICLE_LIGHT_HIGH_BEAM    = 0x1 << 2,
    CARLA_VEHICLE_LIGHT_BRAKE        = 0x1 << 3,
    CARLA_VEHICLE_LIGHT_RIGHT_BLINKER = 0x1 << 4,
    CARLA_VEHICLE_LIGHT_LEFT_BLINKER = 0x1 << 5,
    CARLA_VEHICLE_LIGHT_REVERSE      = 0x1 << 6,
    CARLA_VEHICLE_LIGHT_FOG          = 0x1 << 7,
    CARLA_VEHICLE_LIGHT_INTERIOR     = 0x1 << 8,
    CARLA_VEHICLE_LIGHT_SPECIAL1     = 0x1 << 9,  // Sirens
    CARLA_VEHICLE_LIGHT_SPECIAL2     = 0x1 << 10,
    CARLA_VEHICLE_LIGHT_ALL          = 0xFFFFFFFF
} carla_vehicle_light_state_t;

// Vehicle door enum (CARLA 0.10.0)
typedef enum {
    CARLA_VEHICLE_DOOR_FL = 0,      // Front Left
    CARLA_VEHICLE_DOOR_FR = 1,      // Front Right
    CARLA_VEHICLE_DOOR_RL = 2,      // Rear Left
    CARLA_VEHICLE_DOOR_RR = 3,      // Rear Right
    CARLA_VEHICLE_DOOR_HOOD = 4,
    CARLA_VEHICLE_DOOR_TRUNK = 5,
    CARLA_VEHICLE_DOOR_ALL = 6
} carla_vehicle_door_t;

// Vehicle wheel location enum
typedef enum {
    CARLA_VEHICLE_WHEEL_FL = 0,     // Front Left
    CARLA_VEHICLE_WHEEL_FR = 1,     // Front Right
    CARLA_VEHICLE_WHEEL_BL = 2,     // Back Left
    CARLA_VEHICLE_WHEEL_BR = 3,     // Back Right
    CARLA_VEHICLE_WHEEL_FRONT = 0,  // For bikes/bicycles
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

// Callback function types
typedef void (*carla_actor_destroy_callback_t)(carla_actor_t* actor, void* user_data);

// Utility functions
const char* carla_error_to_string(carla_error_t error);

// Memory management helpers
void carla_free_string(char* str);
void carla_free_string_list(carla_string_list_t* list);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_TYPES_H