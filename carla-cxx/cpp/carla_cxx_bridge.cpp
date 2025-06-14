#include "carla_cxx_bridge.h"
#include "carla-cxx/src/ffi.rs.h"

// Must include the actual CARLA headers
#include <carla/Time.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/World.h>
#include <carla/geom/Vector3D.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace carla {
namespace client {

std::unique_ptr<Client> create_client(rust::Str host, uint16_t port,
                                      size_t worker_threads) {

  try {
    // Convert rust::Str to std::string
    std::string host_str(host);

    // Create the client with specified worker threads
    return std::make_unique<Client>(host_str, port, worker_threads);
  } catch (const std::exception &e) {
    // Return nullptr on error - Rust side will handle this
    return nullptr;
  }
}

// Client wrapper functions
void Client_SetTimeout(Client &client, double timeout_seconds) {
  auto duration = carla::time_duration::seconds(timeout_seconds);
  client.SetTimeout(duration);
}

double Client_GetTimeout(Client &client) {
  auto duration = client.GetTimeout();
  // Convert carla::time_duration to seconds
  // time_duration stores milliseconds internally
  return duration.to_chrono().count() / 1000.0; // milliseconds to seconds
}

rust::String Client_GetServerVersion(const Client &client) {
  return rust::String(client.GetServerVersion());
}

std::shared_ptr<World> Client_GetWorld(const Client &client) {
  // GetWorld returns by value, we need to wrap it in a shared_ptr
  return std::make_shared<World>(client.GetWorld());
}

// World wrapper functions
uint64_t World_GetId(const World &world) { return world.GetId(); }

std::shared_ptr<BlueprintLibrary>
World_GetBlueprintLibrary(const World &world) {
  return world.GetBlueprintLibrary();
}

std::shared_ptr<Actor> World_GetSpectator(const World &world) {
  return world.GetSpectator();
}

uint64_t World_Tick(const World &world, double timeout_seconds) {
  auto duration = carla::time_duration::seconds(timeout_seconds);
  // const_cast is needed because CARLA's API is not const-correct
  return const_cast<World &>(world).Tick(duration);
}

std::shared_ptr<Actor> World_SpawnActor(const World &world,
                                        const ActorBlueprint &blueprint,
                                        const SimpleTransform &transform,
                                        const Actor *parent) {

  // Convert SimpleTransform to carla::geom::Transform
  carla::geom::Transform carla_transform(
      carla::geom::Location(transform.location.x, transform.location.y,
                            transform.location.z),
      carla::geom::Rotation(transform.rotation.pitch, transform.rotation.yaw,
                            transform.rotation.roll));

  // const_cast is needed because CARLA's API is not const-correct
  return const_cast<World &>(world).SpawnActor(blueprint, carla_transform,
                                               const_cast<Actor *>(parent));
}

std::shared_ptr<Actor> World_TrySpawnActor(const World &world,
                                           const ActorBlueprint &blueprint,
                                           const SimpleTransform &transform,
                                           const Actor *parent) {

  // Convert SimpleTransform to carla::geom::Transform
  carla::geom::Transform carla_transform(
      carla::geom::Location(transform.location.x, transform.location.y,
                            transform.location.z),
      carla::geom::Rotation(transform.rotation.pitch, transform.rotation.yaw,
                            transform.rotation.roll));

  // const_cast is needed because CARLA's API is not const-correct
  return const_cast<World &>(world).TrySpawnActor(blueprint, carla_transform,
                                                  const_cast<Actor *>(parent));
}

// Actor wrapper functions
uint32_t Actor_GetId(const Actor &actor) { return actor.GetId(); }

rust::String Actor_GetTypeId(const Actor &actor) {
  return rust::String(actor.GetTypeId());
}

rust::String Actor_GetDisplayId(const Actor &actor) {
  return rust::String(actor.GetDisplayId());
}

SimpleLocation Actor_GetLocation(const Actor &actor) {
  auto loc = actor.GetLocation();
  return SimpleLocation{loc.x, loc.y, loc.z};
}

SimpleTransform Actor_GetTransform(const Actor &actor) {
  auto trans = actor.GetTransform();
  return SimpleTransform{
      SimpleLocation{trans.location.x, trans.location.y, trans.location.z},
      SimpleRotation{trans.rotation.pitch, trans.rotation.yaw,
                     trans.rotation.roll}};
}

void Actor_SetLocation(const Actor &actor, const SimpleLocation &location) {
  carla::geom::Location carla_location(location.x, location.y, location.z);
  // const_cast is needed because CARLA's API is not const-correct
  const_cast<Actor &>(actor).SetLocation(carla_location);
}

void Actor_SetTransform(const Actor &actor, const SimpleTransform &transform) {
  carla::geom::Transform carla_transform(
      carla::geom::Location(transform.location.x, transform.location.y,
                            transform.location.z),
      carla::geom::Rotation(transform.rotation.pitch, transform.rotation.yaw,
                            transform.rotation.roll));
  // const_cast is needed because CARLA's API is not const-correct
  const_cast<Actor &>(actor).SetTransform(carla_transform);
}

bool Actor_Destroy(const Actor &actor) {
  // const_cast is needed because CARLA's API is not const-correct
  return const_cast<Actor &>(actor).Destroy();
}

bool Actor_IsAlive(const Actor &actor) { return actor.IsAlive(); }

// BlueprintLibrary wrapper functions
std::shared_ptr<ActorBlueprint>
BlueprintLibrary_Find(const BlueprintLibrary &library, rust::Str id) {
  std::string id_str(id);
  auto result = library.Find(id_str);
  // CARLA returns raw pointer, we need to make a shared_ptr
  if (result) {
    // Make a copy wrapped in shared_ptr
    return std::make_shared<ActorBlueprint>(*result);
  }
  return nullptr;
}

// TODO: Implement filter - CXX doesn't support Vec<SharedPtr<T>>
// rust::Vec<std::shared_ptr<ActorBlueprint>> BlueprintLibrary_Filter(const
// BlueprintLibrary& library, rust::Str wildcard_pattern) {
//     std::string pattern_str(wildcard_pattern);
//     auto filtered = library.Filter(pattern_str);
//
//     rust::Vec<std::shared_ptr<ActorBlueprint>> result;
//     for (const auto& blueprint : filtered) {
//         // Convert to shared_ptr
//         result.push_back(std::make_shared<ActorBlueprint>(blueprint));
//     }
//     return result;
// }

size_t BlueprintLibrary_Size(const BlueprintLibrary &library) {
  return library.size();
}

// ActorBlueprint wrapper functions
rust::String ActorBlueprint_GetId(const ActorBlueprint &blueprint) {
  return rust::String(blueprint.GetId());
}

rust::Vec<rust::String>
ActorBlueprint_GetTags(const ActorBlueprint &blueprint) {
  rust::Vec<rust::String> result;
  for (const auto &tag : blueprint.GetTags()) {
    result.push_back(rust::String(tag));
  }
  return result;
}

bool ActorBlueprint_MatchTags(const ActorBlueprint &blueprint,
                              rust::Str wildcard_pattern) {
  std::string pattern_str(wildcard_pattern);
  return blueprint.MatchTags(pattern_str);
}

bool ActorBlueprint_ContainsTag(const ActorBlueprint &blueprint,
                                rust::Str tag) {
  std::string tag_str(tag);
  return blueprint.ContainsTag(tag_str);
}

bool ActorBlueprint_ContainsAttribute(const ActorBlueprint &blueprint,
                                      rust::Str id) {
  std::string id_str(id);
  return blueprint.ContainsAttribute(id_str);
}

void ActorBlueprint_SetAttribute(const ActorBlueprint &blueprint, rust::Str id,
                                 rust::Str value) {
  std::string id_str(id);
  std::string value_str(value);
  // const_cast is needed because CARLA's API is not const-correct
  const_cast<ActorBlueprint &>(blueprint).SetAttribute(id_str, value_str);
}

// Geometry utility functions implementations
double Vector2D_Length(const SimpleVector2D &vector) {
  return std::sqrt(vector.x * vector.x + vector.y * vector.y);
}

double Vector2D_SquaredLength(const SimpleVector2D &vector) {
  return vector.x * vector.x + vector.y * vector.y;
}

double Vector2D_Distance(const SimpleVector2D &a, const SimpleVector2D &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double Vector2D_DistanceSquared(const SimpleVector2D &a,
                                const SimpleVector2D &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

double Vector2D_Dot(const SimpleVector2D &a, const SimpleVector2D &b) {
  return a.x * b.x + a.y * b.y;
}

double Vector3D_Length(const SimpleVector3D &vector) {
  return std::sqrt(vector.x * vector.x + vector.y * vector.y +
                   vector.z * vector.z);
}

double Vector3D_SquaredLength(const SimpleVector3D &vector) {
  return vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
}

double Vector3D_Distance(const SimpleVector3D &a, const SimpleVector3D &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double Vector3D_DistanceSquared(const SimpleVector3D &a,
                                const SimpleVector3D &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

double Vector3D_Dot(const SimpleVector3D &a, const SimpleVector3D &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

SimpleVector3D Vector3D_Cross(const SimpleVector3D &a,
                              const SimpleVector3D &b) {
  return SimpleVector3D{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                        a.x * b.y - a.y * b.x};
}

double Location_Distance(const SimpleLocation &a, const SimpleLocation &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double Location_DistanceSquared(const SimpleLocation &a,
                                const SimpleLocation &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

SimpleLocation Transform_TransformPoint(const SimpleTransform &transform,
                                        const SimpleLocation &point) {
  // Convert to CARLA types
  carla::geom::Transform carla_transform(
      carla::geom::Location(transform.location.x, transform.location.y,
                            transform.location.z),
      carla::geom::Rotation(transform.rotation.pitch, transform.rotation.yaw,
                            transform.rotation.roll));
  carla::geom::Vector3D carla_point(point.x, point.y, point.z);

  // Transform the point (modifies carla_point in-place)
  carla_transform.TransformPoint(carla_point);

  return SimpleLocation{carla_point.x, carla_point.y, carla_point.z};
}

SimpleLocation Transform_InverseTransformPoint(const SimpleTransform &transform,
                                               const SimpleLocation &point) {
  // Convert to CARLA types
  carla::geom::Transform carla_transform(
      carla::geom::Location(transform.location.x, transform.location.y,
                            transform.location.z),
      carla::geom::Rotation(transform.rotation.pitch, transform.rotation.yaw,
                            transform.rotation.roll));
  carla::geom::Vector3D carla_point(point.x, point.y, point.z);

  // Inverse transform the point (modifies carla_point in-place)
  carla_transform.InverseTransformPoint(carla_point);

  return SimpleLocation{carla_point.x, carla_point.y, carla_point.z};
}

SimpleVector3D Transform_GetForwardVector(const SimpleTransform &transform) {
  // Convert to CARLA rotation
  carla::geom::Rotation carla_rotation(transform.rotation.pitch,
                                       transform.rotation.yaw,
                                       transform.rotation.roll);

  // Get forward vector from rotation
  auto forward = carla_rotation.GetForwardVector();

  return SimpleVector3D{forward.x, forward.y, forward.z};
}

SimpleVector3D Transform_GetRightVector(const SimpleTransform &transform) {
  // Convert to CARLA rotation
  carla::geom::Rotation carla_rotation(transform.rotation.pitch,
                                       transform.rotation.yaw,
                                       transform.rotation.roll);

  // Get right vector from rotation
  auto right = carla_rotation.GetRightVector();

  return SimpleVector3D{right.x, right.y, right.z};
}

SimpleVector3D Transform_GetUpVector(const SimpleTransform &transform) {
  // Convert to CARLA rotation
  carla::geom::Rotation carla_rotation(transform.rotation.pitch,
                                       transform.rotation.yaw,
                                       transform.rotation.roll);

  // Get up vector from rotation
  auto up = carla_rotation.GetUpVector();

  return SimpleVector3D{up.x, up.y, up.z};
}

bool BoundingBox_Contains(const SimpleBoundingBox &bbox,
                          const SimpleLocation &point) {
  // Check if point is within the bounding box
  double min_x = bbox.location.x - bbox.extent.x;
  double max_x = bbox.location.x + bbox.extent.x;
  double min_y = bbox.location.y - bbox.extent.y;
  double max_y = bbox.location.y + bbox.extent.y;
  double min_z = bbox.location.z - bbox.extent.z;
  double max_z = bbox.location.z + bbox.extent.z;

  return (point.x >= min_x && point.x <= max_x && point.y >= min_y &&
          point.y <= max_y && point.z >= min_z && point.z <= max_z);
}

rust::Vec<SimpleLocation>
BoundingBox_GetVertices(const SimpleBoundingBox &bbox) {
  rust::Vec<SimpleLocation> vertices;

  // Calculate all 8 vertices of the bounding box
  double min_x = bbox.location.x - bbox.extent.x;
  double max_x = bbox.location.x + bbox.extent.x;
  double min_y = bbox.location.y - bbox.extent.y;
  double max_y = bbox.location.y + bbox.extent.y;
  double min_z = bbox.location.z - bbox.extent.z;
  double max_z = bbox.location.z + bbox.extent.z;

  vertices.push_back(SimpleLocation{min_x, min_y, min_z}); // 0: min corner
  vertices.push_back(SimpleLocation{max_x, min_y, min_z}); // 1: +x
  vertices.push_back(SimpleLocation{min_x, max_y, min_z}); // 2: +y
  vertices.push_back(SimpleLocation{max_x, max_y, min_z}); // 3: +x+y
  vertices.push_back(SimpleLocation{min_x, min_y, max_z}); // 4: +z
  vertices.push_back(SimpleLocation{max_x, min_y, max_z}); // 5: +x+z
  vertices.push_back(SimpleLocation{min_x, max_y, max_z}); // 6: +y+z
  vertices.push_back(SimpleLocation{max_x, max_y, max_z}); // 7: max corner

  return vertices;
}

} // namespace client
} // namespace carla
