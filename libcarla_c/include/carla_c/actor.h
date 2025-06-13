#ifndef CARLA_C_ACTOR_H
#define CARLA_C_ACTOR_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Actor lifecycle
void carla_actor_destroy(carla_actor_t *actor);
bool carla_actor_is_alive(const carla_actor_t *actor);

// Actor properties
uint32_t carla_actor_get_id(const carla_actor_t *actor);
const char *carla_actor_get_type_id(const carla_actor_t *actor);
const char *carla_actor_get_display_id(const carla_actor_t *actor);
carla_actor_blueprint_t *carla_actor_get_blueprint(const carla_actor_t *actor);

// Actor type checking
bool carla_actor_is_walker(const carla_actor_t *actor);
bool carla_actor_is_traffic_light(const carla_actor_t *actor);
bool carla_actor_is_traffic_sign(const carla_actor_t *actor);

// Actor relationships
carla_actor_t *carla_actor_get_parent(const carla_actor_t *actor);

// Actor transform
carla_transform_t carla_actor_get_transform(const carla_actor_t *actor);
carla_vector3d_t carla_actor_get_location(const carla_actor_t *actor);
carla_vector3d_t carla_actor_get_velocity(const carla_actor_t *actor);
carla_vector3d_t carla_actor_get_acceleration(const carla_actor_t *actor);
carla_vector3d_t carla_actor_get_angular_velocity(const carla_actor_t *actor);
void carla_actor_set_transform(carla_actor_t *actor,
                               const carla_transform_t *transform);
void carla_actor_set_location(carla_actor_t *actor,
                              const carla_vector3d_t *location);
void carla_actor_set_simulate_physics(carla_actor_t *actor, bool enabled);
void carla_actor_set_enable_gravity(carla_actor_t *actor, bool enabled);

// Actor physics
void carla_actor_add_impulse(carla_actor_t *actor,
                             const carla_vector3d_t *impulse);
void carla_actor_add_impulse_at_location(carla_actor_t *actor,
                                         const carla_vector3d_t *impulse,
                                         const carla_vector3d_t *location);
void carla_actor_add_force(carla_actor_t *actor, const carla_vector3d_t *force);
void carla_actor_add_force_at_location(carla_actor_t *actor,
                                       const carla_vector3d_t *force,
                                       const carla_vector3d_t *location);
void carla_actor_add_angular_impulse(carla_actor_t *actor,
                                     const carla_vector3d_t *angular_impulse);
void carla_actor_add_torque(carla_actor_t *actor,
                            const carla_vector3d_t *torque);

// Blueprint library operations
void carla_blueprint_library_free(carla_blueprint_library_t *library);
size_t carla_blueprint_library_size(const carla_blueprint_library_t *library);
carla_actor_blueprint_t *
carla_blueprint_library_at(carla_blueprint_library_t *library, size_t index);
carla_actor_blueprint_t *
carla_blueprint_library_find(carla_blueprint_library_t *library,
                             const char *id);
carla_actor_blueprint_t **
carla_blueprint_library_filter(carla_blueprint_library_t *library,
                               const char *wildcard_pattern, size_t *out_count);

// Actor blueprint operations
const char *
carla_actor_blueprint_get_id(const carla_actor_blueprint_t *blueprint);
bool carla_actor_blueprint_has_tag(const carla_actor_blueprint_t *blueprint,
                                   const char *tag);
bool carla_actor_blueprint_match_tags(const carla_actor_blueprint_t *blueprint,
                                      const char *wildcard_pattern);
size_t carla_actor_blueprint_get_attribute_count(
    const carla_actor_blueprint_t *blueprint);
carla_actor_attribute_t *
carla_actor_blueprint_get_attribute(const carla_actor_blueprint_t *blueprint,
                                    size_t index);
carla_actor_attribute_t *
carla_actor_blueprint_find_attribute(const carla_actor_blueprint_t *blueprint,
                                     const char *id);
bool carla_actor_blueprint_set_attribute(carla_actor_blueprint_t *blueprint,
                                         const char *id, const char *value);

// Actor attribute operations
const char *
carla_actor_attribute_get_id(const carla_actor_attribute_t *attribute);
const char *
carla_actor_attribute_get_value(const carla_actor_attribute_t *attribute);
bool carla_actor_attribute_is_modifiable(
    const carla_actor_attribute_t *attribute);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_ACTOR_H
