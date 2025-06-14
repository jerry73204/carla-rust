#pragma once

#include "carla_c/types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Walker control structure
typedef struct {
  carla_vector3d_t direction; // Walking direction (normalized)
  float speed;                // Walking speed (0.0 to 1.0)
  bool jump;                  // Whether walker should jump
} carla_walker_control_t;

// Walker bone control structures
typedef struct {
  carla_transform_t transform;
  char bone_name[64];
} carla_walker_bone_transform_t;

typedef struct {
  carla_walker_bone_transform_t *bone_transforms;
  size_t bone_count;
  float blend_weight;
} carla_walker_bone_control_t;

// Walker functions
carla_error_t carla_walker_apply_control(carla_actor_t *walker,
                                         const carla_walker_control_t *control);
carla_walker_control_t carla_walker_get_control(const carla_actor_t *walker);

// Walker AI controller functions
carla_error_t carla_walker_ai_start(carla_actor_t *walker);
carla_error_t carla_walker_ai_stop(carla_actor_t *walker);
carla_error_t carla_walker_ai_set_max_speed(carla_actor_t *walker,
                                            float max_speed);
carla_error_t
carla_walker_ai_go_to_location(carla_actor_t *walker,
                               const carla_vector3d_t *destination);
carla_error_t carla_walker_ai_get_random_location(carla_actor_t *walker,
                                                  carla_vector3d_t *location);

// Walker bone control functions
carla_error_t
carla_walker_set_bones_transform(carla_actor_t *walker,
                                 const carla_walker_bone_control_t *bones);
carla_walker_bone_control_t
carla_walker_get_bones_transform(const carla_actor_t *walker);
carla_error_t carla_walker_blend_pose(carla_actor_t *walker, float blend);
carla_error_t carla_walker_show_pose(carla_actor_t *walker);
carla_error_t carla_walker_hide_pose(carla_actor_t *walker);
carla_error_t carla_walker_get_pose_from_animation(carla_actor_t *walker);

// Walker bone control memory management
void carla_walker_bone_control_free(carla_walker_bone_control_t *bone_control);

#ifdef __cplusplus
}
#endif
