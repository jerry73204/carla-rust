#ifndef CARLA_C_MAP_H
#define CARLA_C_MAP_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Map lifecycle
void carla_map_free(carla_map_t* map);

// Map properties
const char* carla_map_get_name(const carla_map_t* map);
double carla_map_get_lane_width(const carla_map_t* map);

// Waypoint queries
carla_waypoint_t* carla_map_get_waypoint(const carla_map_t* map, 
                                         const carla_vector3d_t* location,
                                         bool project_to_road,
                                         int32_t lane_type);

carla_waypoint_t* carla_map_get_waypoint_xodr(const carla_map_t* map,
                                              const char* road_id,
                                              int32_t lane_id, 
                                              float s);

carla_waypoint_list_t* carla_map_get_topology(const carla_map_t* map);
carla_waypoint_list_t* carla_map_generate_waypoints(const carla_map_t* map, double distance);

// Spawn points
carla_transform_list_t* carla_map_get_spawn_points(const carla_map_t* map);

// Waypoint operations
void carla_waypoint_free(carla_waypoint_t* waypoint);
uint64_t carla_waypoint_get_id(const carla_waypoint_t* waypoint);
uint32_t carla_waypoint_get_road_id(const carla_waypoint_t* waypoint);
uint32_t carla_waypoint_get_section_id(const carla_waypoint_t* waypoint);
int32_t carla_waypoint_get_lane_id(const carla_waypoint_t* waypoint);
double carla_waypoint_get_s(const carla_waypoint_t* waypoint);
bool carla_waypoint_is_junction(const carla_waypoint_t* waypoint);
double carla_waypoint_get_lane_width(const carla_waypoint_t* waypoint);
int32_t carla_waypoint_get_lane_change(const carla_waypoint_t* waypoint);
int32_t carla_waypoint_get_lane_type(const carla_waypoint_t* waypoint);
carla_transform_t carla_waypoint_get_transform(const carla_waypoint_t* waypoint);

// Waypoint navigation
carla_waypoint_t* carla_waypoint_get_next(const carla_waypoint_t* waypoint, double distance);
carla_waypoint_list_t* carla_waypoint_get_next_list(const carla_waypoint_t* waypoint, double distance);
carla_waypoint_t* carla_waypoint_get_previous(const carla_waypoint_t* waypoint, double distance);
carla_waypoint_list_t* carla_waypoint_get_previous_list(const carla_waypoint_t* waypoint, double distance);
carla_waypoint_t* carla_waypoint_get_right(const carla_waypoint_t* waypoint);
carla_waypoint_t* carla_waypoint_get_left(const carla_waypoint_t* waypoint);

// Waypoint list operations  
size_t carla_waypoint_list_size(const carla_waypoint_list_t* list);
carla_waypoint_t* carla_waypoint_list_get(carla_waypoint_list_t* list, size_t index);
void carla_waypoint_list_free(carla_waypoint_list_t* list);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_MAP_H