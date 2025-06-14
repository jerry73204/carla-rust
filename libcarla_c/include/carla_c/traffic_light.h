#pragma once

#include "carla_c/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Traffic light control functions
carla_error_t carla_traffic_light_set_state(carla_actor_t *traffic_light,
                                            carla_traffic_light_state_t state);
carla_traffic_light_state_t
carla_traffic_light_get_state(const carla_actor_t *traffic_light);

// Traffic light timing functions
carla_error_t carla_traffic_light_set_green_time(carla_actor_t *traffic_light,
                                                 float green_time);
float carla_traffic_light_get_green_time(const carla_actor_t *traffic_light);

carla_error_t carla_traffic_light_set_yellow_time(carla_actor_t *traffic_light,
                                                  float yellow_time);
float carla_traffic_light_get_yellow_time(const carla_actor_t *traffic_light);

carla_error_t carla_traffic_light_set_red_time(carla_actor_t *traffic_light,
                                               float red_time);
float carla_traffic_light_get_red_time(const carla_actor_t *traffic_light);

float carla_traffic_light_get_elapsed_time(const carla_actor_t *traffic_light);

// Traffic light control functions
carla_error_t carla_traffic_light_freeze(carla_actor_t *traffic_light,
                                         bool freeze);
bool carla_traffic_light_is_frozen(const carla_actor_t *traffic_light);

// Traffic light group functions
uint32_t carla_traffic_light_get_pole_index(const carla_actor_t *traffic_light);
carla_error_t carla_traffic_light_reset_group(carla_actor_t *traffic_light);

// TODO: The following functions require additional types that are not yet
// implemented: carla_actor_list_t, carla_waypoint_list_t,
// carla_bounding_box_list_t
//
// Traffic light group management
// carla_actor_list_t *carla_traffic_light_get_group_traffic_lights(const
// carla_actor_t *traffic_light);
//
// Traffic light waypoint functions
// carla_waypoint_list_t *carla_traffic_light_get_affected_lane_waypoints(const
// carla_actor_t *traffic_light); carla_waypoint_list_t
// *carla_traffic_light_get_stop_waypoints(const carla_actor_t *traffic_light);
//
// Traffic light geometry functions
// carla_bounding_box_list_t *carla_traffic_light_get_light_boxes(const
// carla_actor_t *traffic_light);

#ifdef __cplusplus
}
#endif
