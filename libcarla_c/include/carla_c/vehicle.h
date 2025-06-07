#ifndef CARLA_C_VEHICLE_H
#define CARLA_C_VEHICLE_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration of opaque vehicle type (vehicle is a specialized actor)
typedef carla_actor_t carla_vehicle_t;

// Vehicle control operations
carla_error_t carla_vehicle_apply_control(carla_vehicle_t* vehicle, const carla_vehicle_control_t* control);
carla_error_t carla_vehicle_apply_ackermann_control(carla_vehicle_t* vehicle, const carla_vehicle_ackermann_control_t* control);
carla_vehicle_control_t carla_vehicle_get_control(const carla_vehicle_t* vehicle);

// Autopilot control
carla_error_t carla_vehicle_set_autopilot(carla_vehicle_t* vehicle, bool enabled, uint16_t tm_port);
carla_error_t carla_vehicle_set_autopilot_default_port(carla_vehicle_t* vehicle, bool enabled);

// Vehicle lighting
carla_error_t carla_vehicle_set_light_state(carla_vehicle_t* vehicle, carla_vehicle_light_state_t light_state);
carla_vehicle_light_state_t carla_vehicle_get_light_state(const carla_vehicle_t* vehicle);

// Vehicle door control (CARLA 0.10.0)
carla_error_t carla_vehicle_open_door(carla_vehicle_t* vehicle, carla_vehicle_door_t door);
carla_error_t carla_vehicle_close_door(carla_vehicle_t* vehicle, carla_vehicle_door_t door);

// Wheel control
carla_error_t carla_vehicle_set_wheel_steer_direction(carla_vehicle_t* vehicle, 
                                                      carla_vehicle_wheel_location_t wheel_location, 
                                                      float angle_in_deg);
float carla_vehicle_get_wheel_steer_angle(const carla_vehicle_t* vehicle, 
                                          carla_vehicle_wheel_location_t wheel_location);

// Ackermann controller settings
carla_error_t carla_vehicle_apply_ackermann_controller_settings(carla_vehicle_t* vehicle, 
                                                                const carla_ackermann_controller_settings_t* settings);
carla_ackermann_controller_settings_t carla_vehicle_get_ackermann_controller_settings(const carla_vehicle_t* vehicle);

// Vehicle state queries
float carla_vehicle_get_speed_limit(const carla_vehicle_t* vehicle);
carla_traffic_light_state_t carla_vehicle_get_traffic_light_state(const carla_vehicle_t* vehicle);
bool carla_vehicle_is_at_traffic_light(const carla_vehicle_t* vehicle);
carla_actor_t* carla_vehicle_get_traffic_light(const carla_vehicle_t* vehicle);
carla_vehicle_failure_state_t carla_vehicle_get_failure_state(const carla_vehicle_t* vehicle);

// Debug and telemetry
carla_error_t carla_vehicle_show_debug_telemetry(carla_vehicle_t* vehicle, bool enabled);

// Utility function to check if an actor is a vehicle
bool carla_actor_is_vehicle(const carla_actor_t* actor);

// Utility function to cast actor to vehicle (returns NULL if not a vehicle)
carla_vehicle_t* carla_actor_as_vehicle(carla_actor_t* actor);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_VEHICLE_H