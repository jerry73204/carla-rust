#ifndef CARLA_C_WORLD_H
#define CARLA_C_WORLD_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// World lifecycle
void carla_world_free(carla_world_t* world);

// World properties
uint64_t carla_world_get_id(const carla_world_t* world);
carla_map_t* carla_world_get_map(const carla_world_t* world);
carla_blueprint_library_t* carla_world_get_blueprint_library(const carla_world_t* world);
carla_actor_t* carla_world_get_spectator(carla_world_t* world);

// Simulation control
uint64_t carla_world_tick(carla_world_t* world, uint64_t timeout_ms);
void carla_world_wait_for_tick(carla_world_t* world, uint64_t timeout_ms);

// Actor management
carla_actor_list_t* carla_world_get_actors(const carla_world_t* world);
carla_actor_list_t* carla_world_get_actors_by_id(const carla_world_t* world, 
                                                  const uint32_t* ids, size_t count);
carla_actor_t* carla_world_get_actor(const carla_world_t* world, uint32_t id);

// Actor spawning
carla_spawn_result_t carla_world_spawn_actor(carla_world_t* world,
                                             const carla_actor_blueprint_t* blueprint,
                                             const carla_transform_t* transform,
                                             carla_actor_t* attach_to);

carla_spawn_result_t carla_world_try_spawn_actor(carla_world_t* world,
                                                 const carla_actor_blueprint_t* blueprint,
                                                 const carla_transform_t* transform,
                                                 carla_actor_t* attach_to);

// Transform queries
carla_transform_list_t* carla_world_get_random_location_from_navigation(carla_world_t* world);
carla_transform_t carla_world_get_spectator_transform(const carla_world_t* world);
void carla_world_set_spectator_transform(carla_world_t* world, const carla_transform_t* transform);

// Weather control
typedef struct {
    float cloudiness;
    float precipitation;
    float precipitation_deposits;
    float wind_intensity;
    float sun_azimuth_angle;
    float sun_altitude_angle;
    float fog_density;
    float fog_distance;
    float wetness;
    float fog_falloff;
    float scattering_intensity;
    float mie_scattering_scale;
    float rayleigh_scattering_scale;
} carla_weather_parameters_t;

carla_weather_parameters_t carla_world_get_weather(const carla_world_t* world);
void carla_world_set_weather(carla_world_t* world, const carla_weather_parameters_t* weather);

// Snapshot
typedef struct {
    uint64_t id;
    uint64_t frame;
    double timestamp;
} carla_world_snapshot_t;

carla_world_snapshot_t carla_world_get_snapshot(const carla_world_t* world);

// Actor list operations
size_t carla_actor_list_size(const carla_actor_list_t* list);
carla_actor_t* carla_actor_list_get(carla_actor_list_t* list, size_t index);
carla_actor_t* carla_actor_list_find(carla_actor_list_t* list, uint32_t id);
void carla_actor_list_free(carla_actor_list_t* list);

// Transform list operations
size_t carla_transform_list_size(const carla_transform_list_t* list);
carla_transform_t carla_transform_list_get(const carla_transform_list_t* list, size_t index);
void carla_transform_list_free(carla_transform_list_t* list);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_WORLD_H