#include "carla_c/actor.h"
#include "carla/client/ActorAttribute.h"
#include "carla/client/TrafficLight.h"
#include "carla/client/TrafficSign.h"
#include "carla/client/Walker.h"
#include "internal.h"
#include <cstdlib>
#include <cstring>
#include <vector>

// Actor blueprint structure (customized for this file)
struct carla_actor_blueprint {
  const carla::client::ActorBlueprint *blueprint;
  std::shared_ptr<carla::client::BlueprintLibrary>
      library; // Keep library alive
};

struct carla_actor_attribute {
  const carla::client::ActorAttribute *attribute;
  std::shared_ptr<carla::client::ActorBlueprint>
      blueprint; // Keep blueprint alive
};

// Helper functions
static carla::geom::Location from_c_location(const carla_vector3d_t *loc) {
  return carla::geom::Location(loc->x, loc->y, loc->z);
}

static carla::geom::Transform from_c_transform(const carla_transform_t *t) {
  carla::geom::Transform result;
  result.location.x = t->location.x;
  result.location.y = t->location.y;
  result.location.z = t->location.z;
  result.rotation.pitch = t->rotation.pitch;
  result.rotation.yaw = t->rotation.yaw;
  result.rotation.roll = t->rotation.roll;
  return result;
}

static carla_transform_t to_c_transform(const carla::geom::Transform &t) {
  carla_transform_t result;
  result.location.x = t.location.x;
  result.location.y = t.location.y;
  result.location.z = t.location.z;
  result.rotation.pitch = t.rotation.pitch;
  result.rotation.yaw = t.rotation.yaw;
  result.rotation.roll = t.rotation.roll;
  return result;
}

static carla_vector3d_t to_c_vector(const carla::geom::Vector3D &v) {
  carla_vector3d_t result;
  result.x = v.x;
  result.y = v.y;
  result.z = v.z;
  return result;
}

// Actor lifecycle
void carla_actor_destroy(carla_actor_t *actor) {
  if (actor && actor->actor) {
    try {
      actor->actor->Destroy();
    } catch (...) {
      // Silently ignore errors
    }
  }
  delete actor;
}

bool carla_actor_is_alive(const carla_actor_t *actor) {
  if (actor && actor->actor) {
    return actor->actor->IsAlive();
  }
  return false;
}

// Actor properties
uint32_t carla_actor_get_id(const carla_actor_t *actor) {
  if (actor && actor->actor) {
    return actor->actor->GetId();
  }
  return 0;
}

const char *carla_actor_get_type_id(const carla_actor_t *actor) {
  if (actor && actor->actor) {
    static thread_local std::string type_id_buffer;
    type_id_buffer = actor->actor->GetTypeId();
    return type_id_buffer.c_str();
  }
  return nullptr;
}

const char *carla_actor_get_display_id(const carla_actor_t *actor) {
  if (actor && actor->actor) {
    static thread_local std::string display_id_buffer;
    display_id_buffer = actor->actor->GetDisplayId();
    return display_id_buffer.c_str();
  }
  return nullptr;
}

carla_actor_blueprint_t *carla_actor_get_blueprint(const carla_actor_t *actor) {
  // Note: LibCarla doesn't provide direct access to actor's blueprint
  // This would need to be implemented differently or removed
  return nullptr;
}

// Actor transform
carla_transform_t carla_actor_get_transform(const carla_actor_t *actor) {
  carla_transform_t result = {{0, 0, 0}, {0, 0, 0}};
  if (actor && actor->actor) {
    auto transform = actor->actor->GetTransform();
    result = to_c_transform(transform);
  }
  return result;
}

carla_vector3d_t carla_actor_get_location(const carla_actor_t *actor) {
  carla_vector3d_t result = {0, 0, 0};
  if (actor && actor->actor) {
    auto location = actor->actor->GetLocation();
    result.x = location.x;
    result.y = location.y;
    result.z = location.z;
  }
  return result;
}

carla_vector3d_t carla_actor_get_velocity(const carla_actor_t *actor) {
  carla_vector3d_t result = {0, 0, 0};
  if (actor && actor->actor) {
    auto velocity = actor->actor->GetVelocity();
    result = to_c_vector(velocity);
  }
  return result;
}

carla_vector3d_t carla_actor_get_acceleration(const carla_actor_t *actor) {
  carla_vector3d_t result = {0, 0, 0};
  if (actor && actor->actor) {
    auto acceleration = actor->actor->GetAcceleration();
    result = to_c_vector(acceleration);
  }
  return result;
}

carla_vector3d_t carla_actor_get_angular_velocity(const carla_actor_t *actor) {
  carla_vector3d_t result = {0, 0, 0};
  if (actor && actor->actor) {
    auto angular_velocity = actor->actor->GetAngularVelocity();
    result = to_c_vector(angular_velocity);
  }
  return result;
}

void carla_actor_set_transform(carla_actor_t *actor,
                               const carla_transform_t *transform) {
  if (actor && actor->actor && transform) {
    try {
      auto cpp_transform = from_c_transform(transform);
      actor->actor->SetTransform(cpp_transform);
    } catch (...) {
      // Silently ignore errors
    }
  }
}

void carla_actor_set_location(carla_actor_t *actor,
                              const carla_vector3d_t *location) {
  if (actor && actor->actor && location) {
    try {
      auto cpp_location = from_c_location(location);
      actor->actor->SetLocation(cpp_location);
    } catch (...) {
      // Silently ignore errors
    }
  }
}

void carla_actor_set_simulate_physics(carla_actor_t *actor, bool enabled) {
  if (actor && actor->actor) {
    try {
      actor->actor->SetSimulatePhysics(enabled);
    } catch (...) {
      // Silently ignore errors
    }
  }
}

void carla_actor_set_enable_gravity(carla_actor_t *actor, bool enabled) {
  if (actor && actor->actor) {
    try {
      actor->actor->SetEnableGravity(enabled);
    } catch (...) {
      // Silently ignore errors
    }
  }
}

// Actor physics
void carla_actor_add_impulse(carla_actor_t *actor,
                             const carla_vector3d_t *impulse) {
  if (actor && actor->actor && impulse) {
    try {
      carla::geom::Vector3D cpp_impulse(impulse->x, impulse->y, impulse->z);
      actor->actor->AddImpulse(cpp_impulse);
    } catch (...) {
      // Silently ignore errors
    }
  }
}

void carla_actor_add_impulse_at_location(carla_actor_t *actor,
                                         const carla_vector3d_t *impulse,
                                         const carla_vector3d_t *location) {
  if (actor && actor->actor && impulse && location) {
    try {
      carla::geom::Vector3D cpp_impulse(impulse->x, impulse->y, impulse->z);
      carla::geom::Vector3D cpp_location(location->x, location->y, location->z);
      actor->actor->AddImpulse(cpp_impulse, cpp_location);
    } catch (...) {
      // Silently ignore errors
    }
  }
}

void carla_actor_add_force(carla_actor_t *actor,
                           const carla_vector3d_t *force) {
  if (actor && actor->actor && force) {
    try {
      carla::geom::Vector3D cpp_force(force->x, force->y, force->z);
      actor->actor->AddForce(cpp_force);
    } catch (...) {
      // Silently ignore errors
    }
  }
}

void carla_actor_add_force_at_location(carla_actor_t *actor,
                                       const carla_vector3d_t *force,
                                       const carla_vector3d_t *location) {
  if (actor && actor->actor && force && location) {
    try {
      carla::geom::Vector3D cpp_force(force->x, force->y, force->z);
      carla::geom::Vector3D cpp_location(location->x, location->y, location->z);
      actor->actor->AddForce(cpp_force, cpp_location);
    } catch (...) {
      // Silently ignore errors
    }
  }
}

void carla_actor_add_angular_impulse(carla_actor_t *actor,
                                     const carla_vector3d_t *angular_impulse) {
  if (actor && actor->actor && angular_impulse) {
    try {
      carla::geom::Vector3D cpp_angular_impulse(
          angular_impulse->x, angular_impulse->y, angular_impulse->z);
      actor->actor->AddAngularImpulse(cpp_angular_impulse);
    } catch (...) {
      // Silently ignore errors
    }
  }
}

void carla_actor_add_torque(carla_actor_t *actor,
                            const carla_vector3d_t *torque) {
  if (actor && actor->actor && torque) {
    try {
      carla::geom::Vector3D cpp_torque(torque->x, torque->y, torque->z);
      actor->actor->AddTorque(cpp_torque);
    } catch (...) {
      // Silently ignore errors
    }
  }
}

// Blueprint library operations
void carla_blueprint_library_free(carla_blueprint_library_t *library) {
  delete library;
}

size_t carla_blueprint_library_size(const carla_blueprint_library_t *library) {
  if (library && library->library) {
    return library->library->size();
  }
  return 0;
}

carla_actor_blueprint_t *
carla_blueprint_library_at(carla_blueprint_library_t *library, size_t index) {
  if (library && library->library && index < library->library->size()) {
    auto *blueprint_wrapper = new carla_actor_blueprint();
    blueprint_wrapper->blueprint = &library->library->at(index);
    blueprint_wrapper->library = library->library;
    return blueprint_wrapper;
  }
  return nullptr;
}

carla_actor_blueprint_t *
carla_blueprint_library_find(carla_blueprint_library_t *library,
                             const char *id) {
  if (library && library->library && id) {
    try {
      auto *found = library->library->Find(id);
      if (found) {
        auto *blueprint_wrapper = new carla_actor_blueprint();
        blueprint_wrapper->blueprint = found;
        blueprint_wrapper->library = library->library;
        return blueprint_wrapper;
      }
    } catch (...) {
      return nullptr;
    }
  }
  return nullptr;
}

carla_actor_blueprint_t **
carla_blueprint_library_filter(carla_blueprint_library_t *library,
                               const char *wildcard_pattern,
                               size_t *out_count) {
  if (library && library->library && wildcard_pattern && out_count) {
    try {
      auto filtered = library->library->Filter(wildcard_pattern);
      *out_count = filtered->size();

      if (*out_count == 0) {
        return nullptr;
      }

      auto **result = (carla_actor_blueprint_t **)calloc(
          *out_count, sizeof(carla_actor_blueprint_t *));
      size_t i = 0;
      for (const auto &blueprint : *filtered) {
        auto *blueprint_wrapper = new carla_actor_blueprint();
        blueprint_wrapper->blueprint = &blueprint;
        blueprint_wrapper->library = filtered; // Keep filtered library alive
        result[i] = blueprint_wrapper;
        ++i;
      }
      return result;
    } catch (...) {
      *out_count = 0;
      return nullptr;
    }
  }
  if (out_count) {
    *out_count = 0;
  }
  return nullptr;
}

// Actor blueprint operations
const char *
carla_actor_blueprint_get_id(const carla_actor_blueprint_t *blueprint) {
  if (blueprint && blueprint->blueprint) {
    static thread_local std::string id_buffer;
    id_buffer = blueprint->blueprint->GetId();
    return id_buffer.c_str();
  }
  return nullptr;
}

bool carla_actor_blueprint_has_tag(const carla_actor_blueprint_t *blueprint,
                                   const char *tag) {
  if (blueprint && blueprint->blueprint && tag) {
    return blueprint->blueprint->ContainsTag(tag);
  }
  return false;
}

bool carla_actor_blueprint_match_tags(const carla_actor_blueprint_t *blueprint,
                                      const char *wildcard_pattern) {
  if (blueprint && blueprint->blueprint && wildcard_pattern) {
    return blueprint->blueprint->MatchTags(wildcard_pattern);
  }
  return false;
}

size_t carla_actor_blueprint_get_attribute_count(
    const carla_actor_blueprint_t *blueprint) {
  if (blueprint && blueprint->blueprint) {
    return blueprint->blueprint->size();
  }
  return 0;
}

carla_actor_attribute_t *
carla_actor_blueprint_get_attribute(const carla_actor_blueprint_t *blueprint,
                                    size_t index) {
  if (blueprint && blueprint->blueprint &&
      index < blueprint->blueprint->size()) {
    auto *attr_wrapper = new carla_actor_attribute();
    // Iterate to the index position since there's no direct indexing
    auto it = blueprint->blueprint->begin();
    std::advance(it, index);
    attr_wrapper->attribute = &(*it);
    // Note: We can't keep the blueprint alive in a shared_ptr since it's const
    return attr_wrapper;
  }
  return nullptr;
}

carla_actor_attribute_t *
carla_actor_blueprint_find_attribute(const carla_actor_blueprint_t *blueprint,
                                     const char *id) {
  if (blueprint && blueprint->blueprint && id) {
    try {
      const auto &attr = blueprint->blueprint->GetAttribute(id);
      auto *attr_wrapper = new carla_actor_attribute();
      attr_wrapper->attribute = &attr;
      return attr_wrapper;
    } catch (...) {
      // GetAttribute throws std::out_of_range if not found
      return nullptr;
    }
  }
  return nullptr;
}

bool carla_actor_blueprint_set_attribute(carla_actor_blueprint_t *blueprint,
                                         const char *id, const char *value) {
  if (blueprint && blueprint->blueprint && id && value) {
    try {
      // Note: ActorBlueprint is const in the library, so we can't modify it
      // This would need to be handled differently in the actual implementation
      return false;
    } catch (...) {
      return false;
    }
  }
  return false;
}

// Actor attribute operations
const char *
carla_actor_attribute_get_id(const carla_actor_attribute_t *attribute) {
  if (attribute && attribute->attribute) {
    static thread_local std::string id_buffer;
    id_buffer = attribute->attribute->GetId();
    return id_buffer.c_str();
  }
  return nullptr;
}

const char *
carla_actor_attribute_get_value(const carla_actor_attribute_t *attribute) {
  if (attribute && attribute->attribute) {
    static thread_local std::string value_buffer;
    value_buffer = attribute->attribute->GetValue();
    return value_buffer.c_str();
  }
  return nullptr;
}

bool carla_actor_attribute_is_modifiable(
    const carla_actor_attribute_t *attribute) {
  if (attribute && attribute->attribute) {
    return attribute->attribute->IsModifiable();
  }
  return false;
}

// Actor type checking functions
bool carla_actor_is_walker(const carla_actor_t *actor) {
  if (!actor || !actor->actor) {
    return false;
  }

  const std::string &type_id = actor->actor->GetTypeId();
  return type_id.find("walker.pedestrian.") == 0;
}

bool carla_actor_is_traffic_light(const carla_actor_t *actor) {
  if (!actor || !actor->actor) {
    return false;
  }

  const std::string &type_id = actor->actor->GetTypeId();
  return type_id.find("traffic.traffic_light") == 0;
}

bool carla_actor_is_traffic_sign(const carla_actor_t *actor) {
  if (!actor || !actor->actor) {
    return false;
  }

  const std::string &type_id = actor->actor->GetTypeId();
  return type_id.find("traffic.") == 0 &&
         type_id.find("traffic.traffic_light") != 0;
}

// Actor relationship functions
carla_actor_t *carla_actor_get_parent(const carla_actor_t *actor) {
  if (!actor || !actor->actor) {
    return nullptr;
  }

  try {
    auto parent = actor->actor->GetParent();
    if (parent) {
      auto *parent_wrapper = new carla_actor_t();
      parent_wrapper->actor = parent;
      return parent_wrapper;
    }
  } catch (...) {
    // Silently ignore errors
  }
  return nullptr;
}

// Note: Actor attachment functionality is not available in this version of
// CARLA
