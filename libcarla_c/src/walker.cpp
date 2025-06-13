#include "carla_c/walker.h"
#include "carla/client/Walker.h"
#include "carla/client/WalkerAIController.h"
#include "carla/rpc/WalkerControl.h"
#include "internal.h"
#include <cstring>
#include <memory>

// Convert C walker control to C++ walker control
carla::rpc::WalkerControl
convert_to_cpp_control(const carla_walker_control_t *c_control) {
  carla::geom::Vector3D direction(
      c_control->direction.x, c_control->direction.y, c_control->direction.z);
  return carla::rpc::WalkerControl(direction, c_control->speed,
                                   c_control->jump);
}

// Convert C++ walker control to C walker control
carla_walker_control_t
convert_to_c_control(const carla::rpc::WalkerControl &cpp_control) {
  carla_walker_control_t c_control;
  c_control.direction.x = cpp_control.direction.x;
  c_control.direction.y = cpp_control.direction.y;
  c_control.direction.z = cpp_control.direction.z;
  c_control.speed = cpp_control.speed;
  c_control.jump = cpp_control.jump;
  return c_control;
}

carla_error_t
carla_walker_apply_control(carla_actor_t *walker,
                           const carla_walker_control_t *control) {
  if (!walker || !walker->actor || !control) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto walker_ptr =
        std::dynamic_pointer_cast<carla::client::Walker>(walker->actor);
    if (!walker_ptr) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    auto cpp_control = convert_to_cpp_control(control);
    walker_ptr->ApplyControl(cpp_control);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &e) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_walker_control_t carla_walker_get_control(const carla_actor_t *walker) {
  carla_walker_control_t default_control = {{0.0f, 0.0f, 0.0f}, 0.0f, false};

  if (!walker || !walker->actor) {
    return default_control;
  }

  try {
    auto walker_ptr =
        std::dynamic_pointer_cast<carla::client::Walker>(walker->actor);
    if (!walker_ptr) {
      return default_control;
    }

    auto cpp_control = walker_ptr->GetWalkerControl();
    return convert_to_c_control(cpp_control);
  } catch (const std::exception &e) {
    return default_control;
  }
}

carla_error_t carla_walker_ai_start(carla_actor_t *walker) {
  if (!walker || !walker->actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto ai_controller =
        std::dynamic_pointer_cast<carla::client::WalkerAIController>(
            walker->actor);
    if (!ai_controller) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    ai_controller->Start();
    return CARLA_ERROR_NONE;
  } catch (const std::exception &e) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t carla_walker_ai_stop(carla_actor_t *walker) {
  if (!walker || !walker->actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto ai_controller =
        std::dynamic_pointer_cast<carla::client::WalkerAIController>(
            walker->actor);
    if (!ai_controller) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    ai_controller->Stop();
    return CARLA_ERROR_NONE;
  } catch (const std::exception &e) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t carla_walker_ai_set_max_speed(carla_actor_t *walker,
                                            float max_speed) {
  if (!walker || !walker->actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto ai_controller =
        std::dynamic_pointer_cast<carla::client::WalkerAIController>(
            walker->actor);
    if (!ai_controller) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    ai_controller->SetMaxSpeed(max_speed);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &e) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t
carla_walker_ai_go_to_location(carla_actor_t *walker,
                               const carla_vector3d_t *destination) {
  if (!walker || !walker->actor || !destination) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto ai_controller =
        std::dynamic_pointer_cast<carla::client::WalkerAIController>(
            walker->actor);
    if (!ai_controller) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    carla::geom::Location cpp_location(destination->x, destination->y,
                                       destination->z);
    ai_controller->GoToLocation(cpp_location);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &e) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// Simplified bone control functions - just stubs for now since they're complex
carla_error_t
carla_walker_set_bones_transform(carla_actor_t *walker,
                                 const carla_walker_bone_control_t *bones) {
  if (!walker || !walker->actor || !bones) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }
  // TODO: Implement bone control when fully needed
  return CARLA_ERROR_NOT_IMPLEMENTED;
}

carla_walker_bone_control_t
carla_walker_get_bones_transform(const carla_actor_t *walker) {
  carla_walker_bone_control_t default_bones = {nullptr, 0, 1.0f};
  // TODO: Implement bone control when fully needed
  return default_bones;
}

carla_error_t carla_walker_blend_pose(carla_actor_t *walker, float blend) {
  if (!walker || !walker->actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto walker_ptr =
        std::dynamic_pointer_cast<carla::client::Walker>(walker->actor);
    if (!walker_ptr) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    walker_ptr->BlendPose(blend);
    return CARLA_ERROR_NONE;
  } catch (const std::exception &e) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_error_t carla_walker_show_pose(carla_actor_t *walker) {
  return carla_walker_blend_pose(walker, 1.0f);
}

carla_error_t carla_walker_hide_pose(carla_actor_t *walker) {
  return carla_walker_blend_pose(walker, 0.0f);
}

carla_error_t carla_walker_get_pose_from_animation(carla_actor_t *walker) {
  if (!walker || !walker->actor) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto walker_ptr =
        std::dynamic_pointer_cast<carla::client::Walker>(walker->actor);
    if (!walker_ptr) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    walker_ptr->GetPoseFromAnimation();
    return CARLA_ERROR_NONE;
  } catch (const std::exception &e) {
    return CARLA_ERROR_UNKNOWN;
  }
}

void carla_walker_bone_control_free(carla_walker_bone_control_t *bone_control) {
  if (bone_control && bone_control->bone_transforms) {
    delete[] bone_control->bone_transforms;
    bone_control->bone_transforms = nullptr;
    bone_control->bone_count = 0;
  }
}
