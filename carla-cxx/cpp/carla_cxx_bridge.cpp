#include "carla_cxx_bridge.h"
#include "carla-cxx/src/ffi.rs.h"

// Must include the actual CARLA headers
#include <carla/Time.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Sensor.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Walker.h>
#include <carla/client/World.h>
#include <carla/geom/Vector3D.h>
#include <carla/rpc/AckermannControllerSettings.h>
#include <carla/rpc/TrafficLightState.h>
#include <carla/rpc/VehicleControl.h>
#include <carla/rpc/VehicleDoor.h>
#include <carla/rpc/VehicleLightState.h>
#include <carla/rpc/VehiclePhysicsControl.h>
#include <carla/rpc/WalkerBoneControlIn.h>
#include <carla/rpc/WalkerBoneControlOut.h>
#include <carla/rpc/WalkerControl.h>
#include <carla/sensor/SensorData.h>
#include <carla/sensor/data/GnssMeasurement.h>
#include <carla/sensor/data/IMUMeasurement.h>
#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/RadarMeasurement.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

// Global storage for sensor data - in a real implementation this would be
// per-sensor
static std::shared_ptr<carla::sensor::SensorData> g_last_sensor_data;
static bool g_has_new_data = false;

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

// Actor casting functions
std::shared_ptr<Vehicle> Actor_CastToVehicle(const Actor &actor) {
  try {
    // Get the actor as a shared_ptr, remove const, and cast to Vehicle
    auto actor_ptr = std::const_pointer_cast<Actor>(actor.shared_from_this());
    return std::dynamic_pointer_cast<Vehicle>(actor_ptr);
  } catch (...) {
    return nullptr;
  }
}

std::shared_ptr<Walker> Actor_CastToWalker(const Actor &actor) {
  try {
    auto actor_ptr = std::const_pointer_cast<Actor>(actor.shared_from_this());
    return std::dynamic_pointer_cast<Walker>(actor_ptr);
  } catch (...) {
    return nullptr;
  }
}

std::shared_ptr<WalkerAIController>
Actor_CastToWalkerAIController(const Actor &actor) {
  try {
    auto actor_ptr = std::const_pointer_cast<Actor>(actor.shared_from_this());
    return std::dynamic_pointer_cast<WalkerAIController>(actor_ptr);
  } catch (...) {
    return nullptr;
  }
}

std::shared_ptr<Sensor> Actor_CastToSensor(const Actor &actor) {
  try {
    auto actor_ptr = std::const_pointer_cast<Actor>(actor.shared_from_this());
    return std::dynamic_pointer_cast<Sensor>(actor_ptr);
  } catch (...) {
    return nullptr;
  }
}

std::shared_ptr<TrafficLight> Actor_CastToTrafficLight(const Actor &actor) {
  try {
    auto actor_ptr = std::const_pointer_cast<Actor>(actor.shared_from_this());
    return std::dynamic_pointer_cast<TrafficLight>(actor_ptr);
  } catch (...) {
    return nullptr;
  }
}

// Vehicle wrapper functions
void Vehicle_ApplyControl(const Vehicle &vehicle,
                          const SimpleVehicleControl &control) {
  carla::rpc::VehicleControl carla_control;
  carla_control.throttle = control.throttle;
  carla_control.steer = control.steer;
  carla_control.brake = control.brake;
  carla_control.hand_brake = control.hand_brake;
  carla_control.reverse = control.reverse;
  carla_control.manual_gear_shift = control.manual_gear_shift;
  carla_control.gear = control.gear;

  const_cast<Vehicle &>(vehicle).ApplyControl(carla_control);
}

SimpleVehicleControl Vehicle_GetControl(const Vehicle &vehicle) {
  auto carla_control = vehicle.GetControl();
  return SimpleVehicleControl{
      carla_control.throttle, carla_control.steer,
      carla_control.brake,    carla_control.hand_brake,
      carla_control.reverse,  carla_control.manual_gear_shift,
      carla_control.gear};
}

void Vehicle_SetAutopilot(const Vehicle &vehicle, bool enabled) {
  const_cast<Vehicle &>(vehicle).SetAutopilot(enabled);
}

float Vehicle_GetSpeed(const Vehicle &vehicle) {
  auto velocity = vehicle.GetVelocity();
  return std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
                   velocity.z * velocity.z);
}

float Vehicle_GetSpeedLimit(const Vehicle &vehicle) {
  return vehicle.GetSpeedLimit();
}

void Vehicle_SetLightState(const Vehicle &vehicle, uint32_t light_state) {
  auto carla_light_state =
      static_cast<carla::rpc::VehicleLightState::LightState>(light_state);
  const_cast<Vehicle &>(vehicle).SetLightState(carla_light_state);
}

uint32_t Vehicle_GetLightState(const Vehicle &vehicle) {
  auto light_state = vehicle.GetLightState();
  return static_cast<uint32_t>(light_state);
}

// Advanced vehicle control functions
void Vehicle_ApplyAckermannControl(const Vehicle &vehicle,
                                   const SimpleAckermannControl &control) {
  carla::rpc::VehicleAckermannControl carla_control;
  carla_control.steer = control.steer;
  carla_control.steer_speed = control.steer_speed;
  carla_control.speed = control.speed;
  carla_control.acceleration = control.acceleration;
  carla_control.jerk = control.jerk;

  const_cast<Vehicle &>(vehicle).ApplyAckermannControl(carla_control);
}

SimpleAckermannControl Vehicle_GetAckermannControl(const Vehicle &vehicle) {
  auto carla_settings = vehicle.GetAckermannControllerSettings();
  // Note: AckermannControllerSettings contains PID parameters, not direct
  // control values For now, return default values as this requires a different
  // approach
  return SimpleAckermannControl{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
}

void Vehicle_ApplyPhysicsControl(const Vehicle &vehicle,
                                 const SimpleVehiclePhysicsControl &control) {
  carla::rpc::VehiclePhysicsControl carla_control;

  // Engine
  carla_control.torque_curve.push_back(
      {control.torque_curve_max_rpm, control.torque_curve_max_torque_nm});
  carla_control.max_rpm = control.max_rpm;
  carla_control.rev_up_moi = control.moi;
  // Note: damping rate fields were removed in CARLA 0.10.0

  // Transmission
  carla_control.use_automatic_gears = control.use_gear_autobox;
  carla_control.gear_change_time = control.gear_switch_time;
  // Note: clutch_strength was removed in CARLA 0.10.0
  carla_control.final_ratio = control.final_ratio;

  // Vehicle properties
  carla_control.mass = control.mass;
  carla_control.drag_coefficient = control.drag_coefficient;
  carla_control.center_of_mass =
      carla::geom::Vector3D(control.center_of_mass_x, control.center_of_mass_y,
                            control.center_of_mass_z);

  // Steering
  carla_control.steering_curve.push_back({0.0f, control.steering_curve_0});
  carla_control.steering_curve.push_back({1.0f, control.steering_curve_1});
  carla_control.use_sweep_wheel_collision = control.use_sweep_wheel_collision;

  const_cast<Vehicle &>(vehicle).ApplyPhysicsControl(carla_control);
}

SimpleVehiclePhysicsControl Vehicle_GetPhysicsControl(const Vehicle &vehicle) {
  auto carla_control = vehicle.GetPhysicsControl();

  // Extract torque curve values (take first if available)
  float max_rpm = 0.0f, max_torque = 0.0f;
  if (!carla_control.torque_curve.empty()) {
    max_rpm = carla_control.torque_curve[0].x;
    max_torque = carla_control.torque_curve[0].y;
  }

  // Extract steering curve values (take first two if available)
  float steering_curve_0 = 0.0f, steering_curve_1 = 0.0f;
  if (carla_control.steering_curve.size() >= 2) {
    steering_curve_0 = carla_control.steering_curve[0].y;
    steering_curve_1 = carla_control.steering_curve[1].y;
  }

  return SimpleVehiclePhysicsControl{
      max_rpm,
      max_torque,
      carla_control.max_rpm,
      carla_control.rev_up_moi,
      0.0f, // damping_rate_full_throttle was removed
      0.0f, // damping_rate_zero_throttle_clutch_engaged was removed
      0.0f, // damping_rate_zero_throttle_clutch_disengaged was removed
      carla_control.use_automatic_gears,
      carla_control.gear_change_time,
      0.0f, // clutch_strength was removed
      carla_control.final_ratio,
      carla_control.mass,
      carla_control.drag_coefficient,
      carla_control.center_of_mass.x,
      carla_control.center_of_mass.y,
      carla_control.center_of_mass.z,
      steering_curve_0,
      steering_curve_1,
      carla_control.use_sweep_wheel_collision};
}

// Vehicle telemetry functions
SimpleVector3D Vehicle_GetVelocity(const Vehicle &vehicle) {
  auto velocity = vehicle.GetVelocity();
  return SimpleVector3D{velocity.x, velocity.y, velocity.z};
}

SimpleVector3D Vehicle_GetAngularVelocity(const Vehicle &vehicle) {
  auto angular_velocity = vehicle.GetAngularVelocity();
  return SimpleVector3D{angular_velocity.x, angular_velocity.y,
                        angular_velocity.z};
}

SimpleVector3D Vehicle_GetAcceleration(const Vehicle &vehicle) {
  auto acceleration = vehicle.GetAcceleration();
  return SimpleVector3D{acceleration.x, acceleration.y, acceleration.z};
}

float Vehicle_GetTireFriction(const Vehicle &vehicle) {
  // Get physics control and return average tire friction
  auto physics = vehicle.GetPhysicsControl();
  if (physics.wheels.empty()) {
    return 0.0f;
  }

  float total_friction = 0.0f;
  for (const auto &wheel : physics.wheels) {
    total_friction += wheel.friction_force_multiplier;
  }
  return total_friction / physics.wheels.size();
}

float Vehicle_GetEngineRpm(const Vehicle &vehicle) {
  // Note: This might need to be implemented differently based on CARLA 0.10.0
  // API For now, return a calculated value based on speed and gear
  auto control = vehicle.GetControl();
  return control.throttle * 3000.0f; // Placeholder calculation
}

float Vehicle_GetGearRatio(const Vehicle &vehicle) {
  auto physics = vehicle.GetPhysicsControl();
  auto control = vehicle.GetControl();

  if (control.gear >= 0 &&
      control.gear < static_cast<int>(physics.forward_gear_ratios.size())) {
    return physics.forward_gear_ratios[control.gear];
  }
  return 1.0f;
}

// Vehicle door functions (CARLA 0.10.0)
void Vehicle_OpenDoor(const Vehicle &vehicle, uint32_t door_type) {
  auto carla_door = static_cast<carla::rpc::VehicleDoor>(door_type);
  const_cast<Vehicle &>(vehicle).OpenDoor(carla_door);
}

void Vehicle_CloseDoor(const Vehicle &vehicle, uint32_t door_type) {
  auto carla_door = static_cast<carla::rpc::VehicleDoor>(door_type);
  const_cast<Vehicle &>(vehicle).CloseDoor(carla_door);
}

bool Vehicle_IsDoorOpen(const Vehicle &vehicle, uint32_t door_type) {
  // CARLA 0.10.0 doesn't provide a direct way to check door state
  // This is a limitation of the current API
  return false;
}

rust::Vec<SimpleVehicleDoor> Vehicle_GetDoorStates(const Vehicle &vehicle) {
  rust::Vec<SimpleVehicleDoor> doors;

  // CARLA 0.10.0 doesn't provide a way to query door states
  // Return empty list as this functionality is not available
  return doors;
}

// Wheel physics functions
rust::Vec<SimpleWheelPhysicsControl>
Vehicle_GetWheelPhysicsControls(const Vehicle &vehicle) {
  rust::Vec<SimpleWheelPhysicsControl> wheels;
  auto physics = vehicle.GetPhysicsControl();

  for (const auto &wheel : physics.wheels) {
    wheels.push_back(SimpleWheelPhysicsControl{
        wheel.friction_force_multiplier, wheel.spring_rate,
        wheel.max_steer_angle, wheel.wheel_radius, wheel.max_brake_torque,
        wheel.max_hand_brake_torque, wheel.location.x, wheel.location.y,
        wheel.location.z});
  }

  return wheels;
}

void Vehicle_SetWheelPhysicsControls(
    const Vehicle &vehicle,
    rust::Slice<const SimpleWheelPhysicsControl> wheels) {
  auto physics = vehicle.GetPhysicsControl();
  physics.wheels.clear();

  for (const auto &wheel : wheels) {
    carla::rpc::WheelPhysicsControl carla_wheel;
    carla_wheel.friction_force_multiplier = wheel.tire_friction;
    carla_wheel.spring_rate = wheel.damping_rate;
    carla_wheel.max_steer_angle = wheel.max_steer_angle;
    carla_wheel.wheel_radius = wheel.radius;
    carla_wheel.max_brake_torque = wheel.max_brake_torque;
    carla_wheel.max_hand_brake_torque = wheel.max_handbrake_torque;
    carla_wheel.location = carla::geom::Location(
        wheel.position_x, wheel.position_y, wheel.position_z);

    physics.wheels.push_back(carla_wheel);
  }

  const_cast<Vehicle &>(vehicle).ApplyPhysicsControl(physics);
}

// Gear physics functions
rust::Vec<SimpleGearPhysicsControl>
Vehicle_GetGearPhysicsControls(const Vehicle &vehicle) {
  rust::Vec<SimpleGearPhysicsControl> gears;
  auto physics = vehicle.GetPhysicsControl();

  // In CARLA 0.10.0, gear ratios are stored as simple float vectors
  for (size_t i = 0; i < physics.forward_gear_ratios.size(); ++i) {
    gears.push_back(SimpleGearPhysicsControl{
        physics.forward_gear_ratios[i],
        0.5f, // Default down_ratio
        0.65f // Default up_ratio
    });
  }

  return gears;
}

void Vehicle_SetGearPhysicsControls(
    const Vehicle &vehicle, rust::Slice<const SimpleGearPhysicsControl> gears) {
  auto physics = vehicle.GetPhysicsControl();
  physics.forward_gear_ratios.clear();

  // In CARLA 0.10.0, only ratios are stored directly
  for (const auto &gear : gears) {
    physics.forward_gear_ratios.push_back(gear.ratio);
  }

  const_cast<Vehicle &>(vehicle).ApplyPhysicsControl(physics);
}

// Walker wrapper functions
void Walker_ApplyControl(const Walker &walker,
                         const SimpleWalkerControl &control) {
  carla::rpc::WalkerControl carla_control;
  carla_control.direction = carla::geom::Vector3D(
      control.direction.x, control.direction.y, control.direction.z);
  carla_control.speed = control.speed;
  carla_control.jump = control.jump;

  const_cast<Walker &>(walker).ApplyControl(carla_control);
}

SimpleWalkerControl Walker_GetControl(const Walker &walker) {
  auto carla_control = walker.GetWalkerControl();
  return SimpleWalkerControl{SimpleVector3D{carla_control.direction.x,
                                            carla_control.direction.y,
                                            carla_control.direction.z},
                             carla_control.speed, carla_control.jump};
}

float Walker_GetSpeed(const Walker &walker) {
  auto velocity = walker.GetVelocity();
  return std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
                   velocity.z * velocity.z);
}

// Walker pose control functions
void Walker_BlendPose(const Walker &walker, float blend) {
  const_cast<Walker &>(walker).BlendPose(blend);
}

void Walker_ShowPose(const Walker &walker) {
  const_cast<Walker &>(walker).ShowPose();
}

void Walker_HidePose(const Walker &walker) {
  const_cast<Walker &>(walker).HidePose();
}

void Walker_GetPoseFromAnimation(const Walker &walker) {
  const_cast<Walker &>(walker).GetPoseFromAnimation();
}

// Walker AI Controller functions
void WalkerAIController_Start(const WalkerAIController &controller) {
  const_cast<WalkerAIController &>(controller).Start();
}

void WalkerAIController_Stop(const WalkerAIController &controller) {
  const_cast<WalkerAIController &>(controller).Stop();
}

void WalkerAIController_SetMaxSpeed(const WalkerAIController &controller,
                                    float max_speed) {
  const_cast<WalkerAIController &>(controller).SetMaxSpeed(max_speed);
}

void WalkerAIController_GoToLocation(
    const WalkerAIController &controller,
    const SimpleWalkerDestination &destination) {
  carla::geom::Location location(destination.x, destination.y, destination.z);
  const_cast<WalkerAIController &>(controller).GoToLocation(location);
}

SimpleWalkerDestination
WalkerAIController_GetRandomLocation(const WalkerAIController &controller) {
  auto location_opt =
      const_cast<WalkerAIController &>(controller).GetRandomLocation();
  if (location_opt.has_value()) {
    auto loc = location_opt.value();
    return SimpleWalkerDestination{loc.x, loc.y, loc.z};
  }
  // Return invalid location (0,0,0) if no random location available
  return SimpleWalkerDestination{0.0, 0.0, 0.0};
}

bool WalkerAIController_HasValidDestination(
    const WalkerAIController &controller) {
  // Check if the controller has a valid destination
  // This is a heuristic based on whether GetRandomLocation returns a value
  return const_cast<WalkerAIController &>(controller)
      .GetRandomLocation()
      .has_value();
}

// Sensor wrapper functions
void Sensor_Stop(const Sensor &sensor) { const_cast<Sensor &>(sensor).Stop(); }

bool Sensor_IsListening(const Sensor &sensor) { return sensor.IsListening(); }

void Sensor_Listen(const Sensor &sensor) {
  // Set up a lambda callback that stores data globally
  auto callback = [](std::shared_ptr<carla::sensor::SensorData> data) {
    g_last_sensor_data = data;
    g_has_new_data = true;
  };

  const_cast<Sensor &>(sensor).Listen(std::move(callback));
}

// Sensor data retrieval functions
SimpleImageData Sensor_GetLastImageData(const Sensor &sensor) {
  if (!g_last_sensor_data) {
    return SimpleImageData{0, 0, 0.0f, 0};
  }

  auto image_data =
      std::dynamic_pointer_cast<carla::sensor::data::Image>(g_last_sensor_data);
  if (image_data) {
    return SimpleImageData{
        image_data->GetWidth(), image_data->GetHeight(),
        90.0f, // Default FOV - would need to get from sensor blueprint
        image_data->size()};
  }

  return SimpleImageData{0, 0, 0.0f, 0};
}

bool Sensor_GetImageDataBuffer(const Sensor &sensor,
                               rust::Slice<uint8_t> buffer) {
  if (!g_last_sensor_data) {
    return false;
  }

  auto image_data =
      std::dynamic_pointer_cast<carla::sensor::data::Image>(g_last_sensor_data);
  if (image_data && buffer.size() >= image_data->size()) {
    std::memcpy(buffer.data(), image_data->data(), image_data->size());
    return true;
  }

  return false;
}

rust::Vec<SimpleLiDARPoint> Sensor_GetLastLiDARData(const Sensor &sensor) {
  rust::Vec<SimpleLiDARPoint> points;

  if (!g_last_sensor_data) {
    return points;
  }

  auto lidar_data =
      std::dynamic_pointer_cast<carla::sensor::data::LidarMeasurement>(
          g_last_sensor_data);
  if (lidar_data) {
    for (const auto &detection : *lidar_data) {
      points.push_back(SimpleLiDARPoint{detection.point.x, detection.point.y,
                                        detection.point.z,
                                        detection.intensity});
    }
  }

  return points;
}

rust::Vec<SimpleRadarDetection> Sensor_GetLastRadarData(const Sensor &sensor) {
  rust::Vec<SimpleRadarDetection> detections;

  if (!g_last_sensor_data) {
    return detections;
  }

  auto radar_data =
      std::dynamic_pointer_cast<carla::sensor::data::RadarMeasurement>(
          g_last_sensor_data);
  if (radar_data) {
    for (const auto &detection : *radar_data) {
      detections.push_back(
          SimpleRadarDetection{detection.velocity, detection.azimuth,
                               detection.altitude, detection.depth});
    }
  }

  return detections;
}

SimpleIMUData Sensor_GetLastIMUData(const Sensor &sensor) {
  if (!g_last_sensor_data) {
    return SimpleIMUData{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  auto imu_data =
      std::dynamic_pointer_cast<carla::sensor::data::IMUMeasurement>(
          g_last_sensor_data);
  if (imu_data) {
    const auto &accelerometer = imu_data->GetAccelerometer();
    const auto &gyroscope = imu_data->GetGyroscope();
    return SimpleIMUData{
        accelerometer.x, accelerometer.y, accelerometer.z,       gyroscope.x,
        gyroscope.y,     gyroscope.z,     imu_data->GetCompass()};
  }

  return SimpleIMUData{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

SimpleGNSSData Sensor_GetLastGNSSData(const Sensor &sensor) {
  if (!g_last_sensor_data) {
    return SimpleGNSSData{0.0, 0.0, 0.0};
  }

  auto gnss_data =
      std::dynamic_pointer_cast<carla::sensor::data::GnssMeasurement>(
          g_last_sensor_data);
  if (gnss_data) {
    return SimpleGNSSData{gnss_data->GetLatitude(), gnss_data->GetLongitude(),
                          gnss_data->GetAltitude()};
  }

  return SimpleGNSSData{0.0, 0.0, 0.0};
}

bool Sensor_HasNewData(const Sensor &sensor) {
  bool result = g_has_new_data;
  g_has_new_data = false; // Reset flag after checking
  return result;
}

// Traffic Light wrapper functions
uint32_t TrafficLight_GetState(const TrafficLight &traffic_light) {
  auto state = traffic_light.GetState();
  return static_cast<uint32_t>(state);
}

void TrafficLight_SetState(const TrafficLight &traffic_light, uint32_t state) {
  auto carla_state = static_cast<carla::rpc::TrafficLightState>(state);
  const_cast<TrafficLight &>(traffic_light).SetState(carla_state);
}

float TrafficLight_GetElapsedTime(const TrafficLight &traffic_light) {
  return traffic_light.GetElapsedTime();
}

void TrafficLight_SetRedTime(const TrafficLight &traffic_light,
                             float red_time) {
  const_cast<TrafficLight &>(traffic_light).SetRedTime(red_time);
}

void TrafficLight_SetYellowTime(const TrafficLight &traffic_light,
                                float yellow_time) {
  const_cast<TrafficLight &>(traffic_light).SetYellowTime(yellow_time);
}

void TrafficLight_SetGreenTime(const TrafficLight &traffic_light,
                               float green_time) {
  const_cast<TrafficLight &>(traffic_light).SetGreenTime(green_time);
}

float TrafficLight_GetRedTime(const TrafficLight &traffic_light) {
  return traffic_light.GetRedTime();
}

float TrafficLight_GetYellowTime(const TrafficLight &traffic_light) {
  return traffic_light.GetYellowTime();
}

float TrafficLight_GetGreenTime(const TrafficLight &traffic_light) {
  return traffic_light.GetGreenTime();
}

void TrafficLight_Freeze(const TrafficLight &traffic_light, bool freeze) {
  const_cast<TrafficLight &>(traffic_light).Freeze(freeze);
}

bool TrafficLight_IsFrozen(const TrafficLight &traffic_light) {
  return traffic_light.IsFrozen();
}

} // namespace client
} // namespace carla
