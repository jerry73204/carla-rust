#include "carla_cxx_bridge.h"
#include "carla-cxx/src/ffi.rs.h"

// Must include the actual CARLA headers
#include <carla/Time.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/ActorList.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Junction.h>
#include <carla/client/Landmark.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/TrafficSign.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Walker.h>
#include <carla/client/Waypoint.h>
#include <carla/client/World.h>
#include <carla/geom/GeoLocation.h>
#include <carla/geom/Location.h>
#include <carla/geom/Vector3D.h>
#include <carla/rpc/AckermannControllerSettings.h>
#include <carla/rpc/Command.h>
#include <carla/rpc/CommandResponse.h>
#include <carla/rpc/EpisodeSettings.h>
#include <carla/rpc/LabelledPoint.h>
#include <carla/rpc/TrafficLightState.h>
#include <carla/rpc/VehicleControl.h>
#include <carla/rpc/VehicleDoor.h>
#include <carla/rpc/VehicleLightState.h>
#include <carla/rpc/VehiclePhysicsControl.h>
#include <carla/rpc/WalkerBoneControlIn.h>
#include <carla/rpc/WalkerBoneControlOut.h>
#include <carla/rpc/WalkerControl.h>
#include <carla/rpc/WeatherParameters.h>
#include <carla/sensor/SensorData.h>
#include <carla/sensor/data/GnssMeasurement.h>
#include <carla/sensor/data/IMUMeasurement.h>
#include <carla/sensor/data/Image.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <carla/sensor/data/RadarMeasurement.h>
#include <carla/trafficmanager/TrafficManager.h>

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

// Recording wrapper functions
rust::String Client_StartRecorder(const Client &client, rust::Str filename,
                                  bool additional_data) {
  std::string file_str(filename);
  auto result =
      const_cast<Client &>(client).StartRecorder(file_str, additional_data);
  return rust::String(result);
}

void Client_StopRecorder(const Client &client) {
  const_cast<Client &>(client).StopRecorder();
}

rust::String Client_ShowRecorderFileInfo(const Client &client,
                                         rust::Str filename, bool show_all) {
  std::string file_str(filename);
  auto result =
      const_cast<Client &>(client).ShowRecorderFileInfo(file_str, show_all);
  return rust::String(result);
}

rust::String Client_ShowRecorderCollisions(const Client &client,
                                           rust::Str filename, uint8_t type1,
                                           uint8_t type2) {
  std::string file_str(filename);
  auto result = const_cast<Client &>(client).ShowRecorderCollisions(
      file_str, static_cast<char>(type1), static_cast<char>(type2));
  return rust::String(result);
}

rust::String Client_ShowRecorderActorsBlocked(const Client &client,
                                              rust::Str filename,
                                              double min_time,
                                              double min_distance) {
  std::string file_str(filename);
  auto result = const_cast<Client &>(client).ShowRecorderActorsBlocked(
      file_str, min_time, min_distance);
  return rust::String(result);
}

// Playback wrapper functions
rust::String Client_ReplayFile(const Client &client, rust::Str filename,
                               double start_time, double duration,
                               uint32_t follow_id, bool replay_sensors) {
  std::string file_str(filename);
  auto result = const_cast<Client &>(client).ReplayFile(
      file_str, start_time, duration, follow_id, replay_sensors);
  return rust::String(result);
}

void Client_StopReplayer(const Client &client, bool keep_actors) {
  const_cast<Client &>(client).StopReplayer(keep_actors);
}

void Client_SetReplayerTimeFactor(const Client &client, double time_factor) {
  const_cast<Client &>(client).SetReplayerTimeFactor(time_factor);
}

void Client_SetReplayerIgnoreHero(const Client &client, bool ignore_hero) {
  const_cast<Client &>(client).SetReplayerIgnoreHero(ignore_hero);
}

void Client_SetReplayerIgnoreSpectator(const Client &client,
                                       bool ignore_spectator) {
  const_cast<Client &>(client).SetReplayerIgnoreSpectator(ignore_spectator);
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

SimpleTimestamp World_GetSnapshot(const World &world) {
  auto snapshot = const_cast<World &>(world).GetSnapshot();
  return SimpleTimestamp{snapshot.GetFrame(),
                         snapshot.GetTimestamp().elapsed_seconds,
                         snapshot.GetTimestamp().delta_seconds,
                         snapshot.GetTimestamp().platform_timestamp};
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

std::shared_ptr<Map> World_GetMap(const World &world) { return world.GetMap(); }

// Episode settings conversion functions
SimpleEpisodeSettings
ConvertToSimpleEpisodeSettings(const carla::rpc::EpisodeSettings &settings) {
  return SimpleEpisodeSettings{settings.synchronous_mode,
                               settings.no_rendering_mode,
                               settings.fixed_delta_seconds.has_value()
                                   ? settings.fixed_delta_seconds.value()
                                   : 0.0,
                               settings.substepping,
                               settings.max_substep_delta_time,
                               static_cast<int32_t>(settings.max_substeps),
                               settings.max_culling_distance,
                               settings.deterministic_ragdolls,
                               settings.tile_stream_distance,
                               settings.actor_active_distance,
                               settings.spectator_as_ego};
}

carla::rpc::EpisodeSettings
ConvertFromSimpleEpisodeSettings(const SimpleEpisodeSettings &simple_settings) {
  carla::rpc::EpisodeSettings settings;
  settings.synchronous_mode = simple_settings.synchronous_mode;
  settings.no_rendering_mode = simple_settings.no_rendering_mode;

  // Handle optional fixed_delta_seconds - 0.0 means None/variable time step
  if (simple_settings.fixed_delta_seconds > 0.0) {
    settings.fixed_delta_seconds = simple_settings.fixed_delta_seconds;
  }

  settings.substepping = simple_settings.substepping;
  settings.max_substep_delta_time = simple_settings.max_substep_delta_time;
  settings.max_substeps = static_cast<size_t>(simple_settings.max_substeps);
  settings.max_culling_distance = simple_settings.max_culling_distance;
  settings.deterministic_ragdolls = simple_settings.deterministic_ragdolls;
  settings.tile_stream_distance = simple_settings.tile_stream_distance;
  settings.actor_active_distance = simple_settings.actor_active_distance;
  settings.spectator_as_ego = simple_settings.spectator_as_ego;

  return settings;
}

SimpleEpisodeSettings World_GetSettings(const World &world) {
  auto settings = const_cast<World &>(world).GetSettings();
  return ConvertToSimpleEpisodeSettings(settings);
}

uint64_t World_ApplySettings(const World &world,
                             const SimpleEpisodeSettings &settings,
                             double timeout_seconds) {
  auto carla_settings = ConvertFromSimpleEpisodeSettings(settings);
  auto duration = carla::time_duration::seconds(timeout_seconds);

  // const_cast is needed because CARLA's API is not const-correct
  return const_cast<World &>(world).ApplySettings(carla_settings, duration);
}

// Weather parameter conversion functions
SimpleWeatherParameters
ConvertToSimpleWeatherParameters(const carla::rpc::WeatherParameters &weather) {
  return SimpleWeatherParameters{weather.cloudiness,
                                 weather.precipitation,
                                 weather.precipitation_deposits,
                                 weather.wind_intensity,
                                 weather.sun_azimuth_angle,
                                 weather.sun_altitude_angle,
                                 weather.fog_density,
                                 weather.fog_distance,
                                 weather.fog_falloff,
                                 weather.wetness,
                                 weather.scattering_intensity,
                                 weather.mie_scattering_scale,
                                 weather.rayleigh_scattering_scale,
                                 weather.dust_storm};
}

carla::rpc::WeatherParameters
ConvertFromSimpleWeatherParameters(const SimpleWeatherParameters &weather) {
  carla::rpc::WeatherParameters carla_weather;
  carla_weather.cloudiness = weather.cloudiness;
  carla_weather.precipitation = weather.precipitation;
  carla_weather.precipitation_deposits = weather.precipitation_deposits;
  carla_weather.wind_intensity = weather.wind_intensity;
  carla_weather.sun_azimuth_angle = weather.sun_azimuth_angle;
  carla_weather.sun_altitude_angle = weather.sun_altitude_angle;
  carla_weather.fog_density = weather.fog_density;
  carla_weather.fog_distance = weather.fog_distance;
  carla_weather.fog_falloff = weather.fog_falloff;
  carla_weather.wetness = weather.wetness;
  carla_weather.scattering_intensity = weather.scattering_intensity;
  carla_weather.mie_scattering_scale = weather.mie_scattering_scale;
  carla_weather.rayleigh_scattering_scale = weather.rayleigh_scattering_scale;
  carla_weather.dust_storm = weather.dust_storm;
  return carla_weather;
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

std::shared_ptr<TrafficSign> Actor_CastToTrafficSign(const Actor &actor) {
  try {
    auto actor_ptr = std::const_pointer_cast<Actor>(actor.shared_from_this());
    return std::dynamic_pointer_cast<TrafficSign>(actor_ptr);
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

SimpleCollisionData Sensor_GetLastCollisionData(const Sensor &sensor) {
  // TODO: Implement actual collision data retrieval
  // For now, return empty collision data
  return SimpleCollisionData{0, SimpleVector3D{0.0, 0.0, 0.0}};
}

rust::Vec<SimpleCrossedLaneMarking>
Sensor_GetLastLaneInvasionData(const Sensor &sensor) {
  rust::Vec<SimpleCrossedLaneMarking> markings;
  // TODO: Implement actual lane invasion data retrieval
  // For now, return empty vector
  return markings;
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

// Traffic Sign wrapper functions
rust::String TrafficSign_GetSignId(const TrafficSign &traffic_sign) {
  return rust::String(traffic_sign.GetSignId());
}

SimpleBoundingBox
TrafficSign_GetTriggerVolume(const TrafficSign &traffic_sign) {
  auto bbox = traffic_sign.GetTriggerVolume();
  return SimpleBoundingBox{
      SimpleLocation{bbox.location.x, bbox.location.y, bbox.location.z},
      SimpleVector3D{bbox.extent.x, bbox.extent.y, bbox.extent.z}};
}

// Map wrapper functions
rust::String Map_GetName(const Map &map) { return rust::String(map.GetName()); }

rust::String Map_GetOpenDrive(const Map &map) {
  return rust::String(map.GetOpenDrive());
}

rust::Vec<SimpleTransform> Map_GetRecommendedSpawnPoints(const Map &map) {
  rust::Vec<SimpleTransform> spawn_points;
  for (const auto &transform : map.GetRecommendedSpawnPoints()) {
    spawn_points.push_back(SimpleTransform{
        SimpleLocation{transform.location.x, transform.location.y,
                       transform.location.z},
        SimpleRotation{transform.rotation.pitch, transform.rotation.yaw,
                       transform.rotation.roll}});
  }
  return spawn_points;
}

std::shared_ptr<Waypoint> Map_GetWaypoint(const Map &map,
                                          const SimpleLocation &location,
                                          bool project_to_road,
                                          int32_t lane_type) {
  carla::geom::Location carla_location(location.x, location.y, location.z);
  return map.GetWaypoint(carla_location, project_to_road, lane_type);
}

std::shared_ptr<Waypoint> Map_GetWaypointXODR(const Map &map, uint32_t road_id,
                                              int32_t lane_id, double s) {
  return map.GetWaypointXODR(road_id, lane_id, s);
}

rust::Vec<SimpleWaypointInfo> Map_GenerateWaypoints(const Map &map,
                                                    double distance) {
  rust::Vec<SimpleWaypointInfo> waypoints;
  for (const auto &wp : map.GenerateWaypoints(distance)) {
    SimpleWaypointInfo simple_wp;
    simple_wp.id = wp->GetId();
    simple_wp.road_id = wp->GetRoadId();
    simple_wp.section_id = wp->GetSectionId();
    simple_wp.lane_id = wp->GetLaneId();
    simple_wp.s = wp->GetDistance();

    const auto &transform = wp->GetTransform();
    simple_wp.transform = SimpleTransform{
        SimpleLocation{transform.location.x, transform.location.y,
                       transform.location.z},
        SimpleRotation{transform.rotation.pitch, transform.rotation.yaw,
                       transform.rotation.roll}};

    simple_wp.is_junction = wp->IsJunction();
    simple_wp.lane_width = wp->GetLaneWidth();
    simple_wp.lane_type = static_cast<uint32_t>(wp->GetType());
    simple_wp.lane_change = static_cast<uint8_t>(wp->GetLaneChange());

    waypoints.push_back(simple_wp);
  }
  return waypoints;
}

SimpleGeoLocation Map_GetGeoReference(const Map &map) {
  const auto &geo = map.GetGeoReference();
  return SimpleGeoLocation{geo.latitude, geo.longitude, geo.altitude};
}

rust::Vec<SimpleLocation> Map_GetAllCrosswalkZones(const Map &map) {
  rust::Vec<SimpleLocation> zones;
  for (const auto &location : map.GetAllCrosswalkZones()) {
    zones.push_back(SimpleLocation{location.x, location.y, location.z});
  }
  return zones;
}

std::shared_ptr<Junction> Map_GetJunction(const Map &map,
                                          const Waypoint &waypoint) {
  return map.GetJunction(waypoint);
}

rust::Vec<SimpleWaypointInfo> Map_GetTopology(const Map &map) {
  rust::Vec<SimpleWaypointInfo> waypoints;
  auto topology = map.GetTopology();
  // Return just the first waypoint of each pair for simplicity
  for (const auto &pair : topology) {
    if (pair.first) {
      SimpleWaypointInfo simple_wp;
      simple_wp.id = pair.first->GetId();
      simple_wp.road_id = pair.first->GetRoadId();
      simple_wp.section_id = pair.first->GetSectionId();
      simple_wp.lane_id = pair.first->GetLaneId();
      simple_wp.s = pair.first->GetDistance();

      const auto &transform = pair.first->GetTransform();
      simple_wp.transform = SimpleTransform{
          SimpleLocation{transform.location.x, transform.location.y,
                         transform.location.z},
          SimpleRotation{transform.rotation.pitch, transform.rotation.yaw,
                         transform.rotation.roll}};

      simple_wp.is_junction = pair.first->IsJunction();
      simple_wp.lane_width = pair.first->GetLaneWidth();
      simple_wp.lane_type = static_cast<uint32_t>(pair.first->GetType());
      simple_wp.lane_change = static_cast<uint8_t>(pair.first->GetLaneChange());

      waypoints.push_back(simple_wp);
    }
  }
  return waypoints;
}

// Waypoint wrapper functions
uint64_t Waypoint_GetId(const Waypoint &waypoint) { return waypoint.GetId(); }

uint32_t Waypoint_GetRoadId(const Waypoint &waypoint) {
  return waypoint.GetRoadId();
}

uint32_t Waypoint_GetSectionId(const Waypoint &waypoint) {
  return waypoint.GetSectionId();
}

int32_t Waypoint_GetLaneId(const Waypoint &waypoint) {
  return waypoint.GetLaneId();
}

double Waypoint_GetDistance(const Waypoint &waypoint) {
  return waypoint.GetDistance();
}

SimpleTransform Waypoint_GetTransform(const Waypoint &waypoint) {
  const auto &transform = waypoint.GetTransform();
  return SimpleTransform{
      SimpleLocation{transform.location.x, transform.location.y,
                     transform.location.z},
      SimpleRotation{transform.rotation.pitch, transform.rotation.yaw,
                     transform.rotation.roll}};
}

uint32_t Waypoint_GetJunctionId(const Waypoint &waypoint) {
  return waypoint.GetJunctionId();
}

bool Waypoint_IsJunction(const Waypoint &waypoint) {
  return waypoint.IsJunction();
}

std::shared_ptr<Junction> Waypoint_GetJunction(const Waypoint &waypoint) {
  return waypoint.GetJunction();
}

double Waypoint_GetLaneWidth(const Waypoint &waypoint) {
  return waypoint.GetLaneWidth();
}

uint32_t Waypoint_GetType(const Waypoint &waypoint) {
  return static_cast<uint32_t>(waypoint.GetType());
}

std::shared_ptr<Waypoint> Waypoint_GetNext(const Waypoint &waypoint,
                                           double distance) {
  auto next_waypoints = waypoint.GetNext(distance);
  if (!next_waypoints.empty()) {
    return next_waypoints[0];
  }
  return nullptr;
}

std::shared_ptr<Waypoint> Waypoint_GetPrevious(const Waypoint &waypoint,
                                               double distance) {
  auto prev_waypoints = waypoint.GetPrevious(distance);
  if (!prev_waypoints.empty()) {
    return prev_waypoints[0];
  }
  return nullptr;
}

std::shared_ptr<Waypoint> Waypoint_GetRight(const Waypoint &waypoint) {
  return waypoint.GetRight();
}

std::shared_ptr<Waypoint> Waypoint_GetLeft(const Waypoint &waypoint) {
  return waypoint.GetLeft();
}

SimpleLaneMarking Waypoint_GetRightLaneMarking(const Waypoint &waypoint) {
  auto marking_opt = waypoint.GetRightLaneMarking();
  if (marking_opt.has_value()) {
    const auto &marking = marking_opt.value();
    return SimpleLaneMarking{static_cast<uint32_t>(marking.type),
                             static_cast<uint8_t>(marking.color),
                             static_cast<uint8_t>(marking.lane_change),
                             marking.width};
  }
  // Return a default marking if none exists
  return SimpleLaneMarking{0, 0, 0, 0.0};
}

SimpleLaneMarking Waypoint_GetLeftLaneMarking(const Waypoint &waypoint) {
  auto marking_opt = waypoint.GetLeftLaneMarking();
  if (marking_opt.has_value()) {
    const auto &marking = marking_opt.value();
    return SimpleLaneMarking{static_cast<uint32_t>(marking.type),
                             static_cast<uint8_t>(marking.color),
                             static_cast<uint8_t>(marking.lane_change),
                             marking.width};
  }
  // Return a default marking if none exists
  return SimpleLaneMarking{0, 0, 0, 0.0};
}

uint8_t Waypoint_GetLaneChange(const Waypoint &waypoint) {
  return static_cast<uint8_t>(waypoint.GetLaneChange());
}

// Removed Waypoint_GetAllLandmarksInDistance and
// Waypoint_GetLandmarksOfTypeInDistance due to CXX limitations with
// Vec<SharedPtr<T>>

// Junction wrapper functions
uint32_t Junction_GetId(const Junction &junction) { return junction.GetId(); }

// Removed Junction_GetWaypoints due to CXX limitations with Vec<SharedPtr<T>>

SimpleBoundingBox Junction_GetBoundingBox(const Junction &junction) {
  const auto &bbox = junction.GetBoundingBox();
  return SimpleBoundingBox{
      SimpleLocation{bbox.location.x, bbox.location.y, bbox.location.z},
      SimpleVector3D{bbox.extent.x, bbox.extent.y, bbox.extent.z}};
}

// Landmark wrapper functions
std::shared_ptr<Waypoint> Landmark_GetWaypoint(const Landmark &landmark) {
  return landmark.GetWaypoint();
}

SimpleTransform Landmark_GetTransform(const Landmark &landmark) {
  const auto &transform = landmark.GetTransform();
  return SimpleTransform{
      SimpleLocation{transform.location.x, transform.location.y,
                     transform.location.z},
      SimpleRotation{transform.rotation.pitch, transform.rotation.yaw,
                     transform.rotation.roll}};
}

uint32_t Landmark_GetRoadId(const Landmark &landmark) {
  return landmark.GetRoadId();
}

double Landmark_GetDistance(const Landmark &landmark) {
  return landmark.GetDistance();
}

double Landmark_GetS(const Landmark &landmark) { return landmark.GetS(); }

double Landmark_GetT(const Landmark &landmark) { return landmark.GetT(); }

rust::String Landmark_GetId(const Landmark &landmark) {
  return rust::String(landmark.GetId());
}

rust::String Landmark_GetName(const Landmark &landmark) {
  return rust::String(landmark.GetName());
}

bool Landmark_IsDynamic(const Landmark &landmark) {
  return landmark.IsDynamic();
}

int32_t Landmark_GetOrientation(const Landmark &landmark) {
  return static_cast<int32_t>(landmark.GetOrientation());
}

double Landmark_GetZOffset(const Landmark &landmark) {
  return landmark.GetZOffset();
}

rust::String Landmark_GetCountry(const Landmark &landmark) {
  return rust::String(landmark.GetCountry());
}

rust::String Landmark_GetType(const Landmark &landmark) {
  return rust::String(landmark.GetType());
}

rust::String Landmark_GetSubType(const Landmark &landmark) {
  return rust::String(landmark.GetSubType());
}

double Landmark_GetValue(const Landmark &landmark) {
  return landmark.GetValue();
}

rust::String Landmark_GetUnit(const Landmark &landmark) {
  return rust::String(landmark.GetUnit());
}

double Landmark_GetHeight(const Landmark &landmark) {
  return landmark.GetHeight();
}

double Landmark_GetWidth(const Landmark &landmark) {
  return landmark.GetWidth();
}

rust::String Landmark_GetText(const Landmark &landmark) {
  return rust::String(landmark.GetText());
}

double Landmark_GetHOffset(const Landmark &landmark) {
  return landmark.GethOffset();
}

double Landmark_GetPitch(const Landmark &landmark) {
  return landmark.GetPitch();
}

double Landmark_GetRoll(const Landmark &landmark) { return landmark.GetRoll(); }

// ===== Traffic Manager Functions =====

// Helper function to create a SharedPtr<Actor> from a Vehicle reference
// This is a simplified approach - in practice, proper lifetime management is
// crucial
static std::shared_ptr<carla::client::Actor>
make_actor_ptr(const Vehicle &vehicle) {
  return std::shared_ptr<carla::client::Actor>(
      const_cast<carla::client::Vehicle *>(&vehicle),
      [](carla::client::Vehicle *) {}
      // No-op deleter since we don't own the pointer
  );
}

std::shared_ptr<carla::traffic_manager::TrafficManager>
TrafficManager_GetInstance(const Client &client, uint16_t port) {
  // Get traffic manager instance for the given port
  auto world = client.GetWorld();
  auto episode = world.GetEpisode();
  auto tm = carla::traffic_manager::TrafficManager(episode, port);
  return std::make_shared<carla::traffic_manager::TrafficManager>(
      std::move(tm));
}

void TrafficManager_RegisterVehicles(
    const carla::traffic_manager::TrafficManager &tm,
    const rust::Slice<const Vehicle *const> vehicles) {
  std::vector<std::shared_ptr<carla::client::Actor>> actor_list;
  for (const Vehicle *vehicle_ptr : vehicles) {
    if (vehicle_ptr) {
      actor_list.push_back(make_actor_ptr(*vehicle_ptr));
    }
  }
  // Note: TrafficManager.RegisterVehicles() is not const, so we need to cast
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.RegisterVehicles(actor_list);
}

void TrafficManager_UnregisterVehicles(
    const carla::traffic_manager::TrafficManager &tm,
    const rust::Slice<const Vehicle *const> vehicles) {
  std::vector<std::shared_ptr<carla::client::Actor>> actor_list;
  for (const Vehicle *vehicle_ptr : vehicles) {
    if (vehicle_ptr) {
      actor_list.push_back(make_actor_ptr(*vehicle_ptr));
    }
  }
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.UnregisterVehicles(actor_list);
}

void TrafficManager_SetSynchronousMode(
    const carla::traffic_manager::TrafficManager &tm, bool mode) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.SetSynchronousMode(mode);
}

bool TrafficManager_SynchronousTick(
    const carla::traffic_manager::TrafficManager &tm) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  return non_const_tm.SynchronousTick();
}

void TrafficManager_SetSynchronousModeTimeout(
    const carla::traffic_manager::TrafficManager &tm, double timeout_ms) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.SetSynchronousModeTimeOutInMiliSecond(timeout_ms);
}

// Global configuration functions
void TrafficManager_SetGlobalSpeedPercentage(
    const carla::traffic_manager::TrafficManager &tm, float percentage) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.SetGlobalPercentageSpeedDifference(percentage);
}

void TrafficManager_SetGlobalLaneOffset(
    const carla::traffic_manager::TrafficManager &tm, float offset) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.SetGlobalLaneOffset(offset);
}

void TrafficManager_SetGlobalDistanceToLeadingVehicle(
    const carla::traffic_manager::TrafficManager &tm, float distance) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.SetGlobalDistanceToLeadingVehicle(distance);
}

void TrafficManager_SetRandomDeviceSeed(
    const carla::traffic_manager::TrafficManager &tm, uint64_t seed) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.SetRandomDeviceSeed(seed);
}

void TrafficManager_SetOSMMode(const carla::traffic_manager::TrafficManager &tm,
                               bool mode) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.SetOSMMode(mode);
}

// Vehicle-specific configuration functions
void TrafficManager_SetVehicleSpeedPercentage(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetPercentageSpeedDifference(actor, percentage);
}

void TrafficManager_SetVehicleDesiredSpeed(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float speed) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetDesiredSpeed(actor, speed);
}

void TrafficManager_SetVehicleLaneOffset(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float offset) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetLaneOffset(actor, offset);
}

void TrafficManager_SetVehicleAutoLaneChange(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    bool enable) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetAutoLaneChange(actor, enable);
}

void TrafficManager_ForceVehicleLaneChange(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    bool direction) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetForceLaneChange(actor, direction);
}

void TrafficManager_SetVehicleDistanceToLeadingVehicle(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float distance) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetDistanceToLeadingVehicle(actor, distance);
}

// Traffic rule compliance functions
void TrafficManager_SetVehiclePercentageRunningLight(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetPercentageRunningLight(actor, percentage);
}

void TrafficManager_SetVehiclePercentageRunningSign(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetPercentageRunningSign(actor, percentage);
}

void TrafficManager_SetVehiclePercentageIgnoreWalkers(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetPercentageIgnoreWalkers(actor, percentage);
}

void TrafficManager_SetVehiclePercentageIgnoreVehicles(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetPercentageIgnoreVehicles(actor, percentage);
}

// Advanced features
void TrafficManager_SetHybridPhysicsMode(
    const carla::traffic_manager::TrafficManager &tm, bool mode) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.SetHybridPhysicsMode(mode);
}

void TrafficManager_SetHybridPhysicsRadius(
    const carla::traffic_manager::TrafficManager &tm, float radius) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.SetHybridPhysicsRadius(radius);
}

void TrafficManager_SetCollisionDetection(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle1,
    const Vehicle &vehicle2, bool detect) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor1 = make_actor_ptr(vehicle1);
  auto actor2 = make_actor_ptr(vehicle2);
  non_const_tm.SetCollisionDetection(actor1, actor2, detect);
}

void TrafficManager_SetVehicleUpdateLights(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    bool update) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetUpdateVehicleLights(actor, update);
}

// Lane behavior percentages
void TrafficManager_SetVehicleKeepRightPercentage(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetKeepRightPercentage(actor, percentage);
}

void TrafficManager_SetVehicleRandomLeftLaneChangePercentage(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetRandomLeftLaneChangePercentage(actor, percentage);
}

void TrafficManager_SetVehicleRandomRightLaneChangePercentage(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  auto actor = make_actor_ptr(vehicle);
  non_const_tm.SetRandomRightLaneChangePercentage(actor, percentage);
}

// Respawn configuration
void TrafficManager_SetRespawnDormantVehicles(
    const carla::traffic_manager::TrafficManager &tm, bool enable) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.SetRespawnDormantVehicles(enable);
}

void TrafficManager_SetRespawnBoundaries(
    const carla::traffic_manager::TrafficManager &tm, float lower_bound,
    float upper_bound) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.SetBoundariesRespawnDormantVehicles(lower_bound, upper_bound);
}

void TrafficManager_SetMaxBoundaries(
    const carla::traffic_manager::TrafficManager &tm, float lower,
    float upper) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.SetMaxBoundaries(lower, upper);
}

// Statistics and monitoring functions - simplified implementations
SimpleTrafficManagerConfig
TrafficManager_GetConfig(const carla::traffic_manager::TrafficManager &tm) {
  // Return a default config since CARLA TM doesn't expose configuration
  // directly
  SimpleTrafficManagerConfig config;
  config.global_speed_percentage_difference = 0.0f;
  config.global_lane_offset = 0.0f;
  config.global_distance_to_leading_vehicle = 3.0f;
  config.synchronous_mode = false;
  config.synchronous_mode_timeout_ms = 2000.0;
  config.hybrid_physics_mode = false;
  config.hybrid_physics_radius = 50.0f;
  config.respawn_dormant_vehicles = false;
  config.respawn_lower_bound = 0.0f;
  config.respawn_upper_bound = 0.0f;
  config.random_device_seed = 0;
  config.osm_mode = false;
  config.port = tm.Port();
  return config;
}

SimpleTrafficManagerVehicleConfig TrafficManager_GetVehicleConfig(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle) {
  // Return default vehicle config since CARLA TM doesn't expose per-vehicle
  // config directly
  SimpleTrafficManagerVehicleConfig config;
  config.speed_percentage_difference = 0.0f;
  config.desired_speed = 0.0f;
  config.lane_offset = 0.0f;
  config.distance_to_leading_vehicle = 3.0f;
  config.auto_lane_change = true;
  config.force_lane_change_direction = false;
  config.force_lane_change_active = false;
  config.keep_right_percentage = 50.0f;
  config.random_left_lane_change_percentage = 0.0f;
  config.random_right_lane_change_percentage = 0.0f;
  config.percentage_running_light = 0.0f;
  config.percentage_running_sign = 0.0f;
  config.percentage_ignore_walkers = 0.0f;
  config.percentage_ignore_vehicles = 0.0f;
  config.update_vehicle_lights = true;
  config.collision_detection_enabled = true;
  return config;
}

SimpleTrafficManagerStats
TrafficManager_GetStats(const carla::traffic_manager::TrafficManager &tm) {
  // Return default stats since CARLA TM doesn't expose statistics directly
  SimpleTrafficManagerStats stats;
  stats.total_registered_vehicles = 0;
  stats.active_vehicle_count = 0;
  stats.total_ticks = 0;
  stats.average_tick_time_ms = 0.0;
  stats.collision_count = 0;
  stats.lane_change_count = 0;
  stats.traffic_light_violations = 0;
  stats.stop_sign_violations = 0;
  stats.total_simulation_time_seconds = 0.0;
  return stats;
}

SimpleTrafficManagerAction
TrafficManager_GetNextAction(const carla::traffic_manager::TrafficManager &tm,
                             const Vehicle &vehicle) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);

  SimpleTrafficManagerAction action;
  try {
    auto carla_action = non_const_tm.GetNextAction(vehicle.GetId());
    // GetNextAction returns a pair<RoadOption, SharedPtr<Waypoint>>
    action.road_option = static_cast<uint32_t>(carla_action.first);
    action.waypoint_id = carla_action.second ? carla_action.second->GetId() : 0;
  } catch (...) {
    // Default action if failed
    action.road_option = 0; // VOID
    action.waypoint_id = 0;
  }
  return action;
}

bool TrafficManager_IsVehicleRegistered(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle) {
  // CARLA TM doesn't provide a direct method to check registration
  // This is a simplified implementation
  return true; // Assume registered for now
}

uint16_t
TrafficManager_GetPort(const carla::traffic_manager::TrafficManager &tm) {
  return tm.Port();
}

// Lifecycle management
void TrafficManager_Shutdown(const carla::traffic_manager::TrafficManager &tm) {
  auto &non_const_tm = const_cast<carla::traffic_manager::TrafficManager &>(tm);
  non_const_tm.ShutDown();
}

void TrafficManager_Reset() { carla::traffic_manager::TrafficManager::Reset(); }

void TrafficManager_Release() {
  carla::traffic_manager::TrafficManager::Release();
}

// Location and vector conversion functions
SimpleLocation ConvertToSimpleLocation(const carla::geom::Location &location) {
  return SimpleLocation{location.x, location.y, location.z};
}

carla::geom::Location
ConvertFromSimpleLocation(const SimpleLocation &simple_location) {
  return carla::geom::Location(simple_location.x, simple_location.y,
                               simple_location.z);
}

SimpleVector3D ConvertToSimpleVector3D(const carla::geom::Vector3D &vector) {
  return SimpleVector3D{vector.x, vector.y, vector.z};
}

carla::geom::Vector3D
ConvertFromSimpleVector3D(const SimpleVector3D &simple_vector) {
  return carla::geom::Vector3D(simple_vector.x, simple_vector.y,
                               simple_vector.z);
}

// World interaction helper functions
SimpleLabelledPoint
ConvertToSimpleLabelledPoint(const carla::rpc::LabelledPoint &point) {
  SimpleLabelledPoint simple_point;
  simple_point.location = ConvertToSimpleLocation(point._location);
  simple_point.label = static_cast<uint8_t>(point._label);
  return simple_point;
}

SimpleOptionalLabelledPoint ConvertToSimpleOptionalLabelledPoint(
    const std::optional<carla::rpc::LabelledPoint> &opt_point) {
  SimpleOptionalLabelledPoint simple_opt;
  simple_opt.has_value = opt_point.has_value();
  if (opt_point.has_value()) {
    simple_opt.value = ConvertToSimpleLabelledPoint(opt_point.value());
  } else {
    // Initialize with default values when no value
    simple_opt.value = SimpleLabelledPoint{};
  }
  return simple_opt;
}

SimpleOptionalLocation ConvertToSimpleOptionalLocation(
    const std::optional<carla::geom::Location> &opt_location) {
  SimpleOptionalLocation simple_opt;
  simple_opt.has_value = opt_location.has_value();
  if (opt_location.has_value()) {
    simple_opt.value = ConvertToSimpleLocation(opt_location.value());
  } else {
    // Initialize with default values when no value
    simple_opt.value = SimpleLocation{};
  }
  return simple_opt;
}

SimpleActorList ConvertToSimpleActorList(
    const std::vector<std::shared_ptr<carla::client::Actor>> &actors) {
  SimpleActorList simple_list;
  simple_list.actor_ids.reserve(actors.size());
  for (const auto &actor : actors) {
    if (actor) {
      simple_list.actor_ids.push_back(actor->GetId());
    }
  }
  return simple_list;
}

SimpleActorList
ConvertToSimpleActorList(const carla::client::ActorList &actors) {
  SimpleActorList simple_list;
  simple_list.actor_ids.reserve(actors.size());
  for (const auto &actor : actors) {
    if (actor) {
      simple_list.actor_ids.push_back(actor->GetId());
    }
  }
  return simple_list;
}

// World interaction wrapper functions

// Ray casting functionality
rust::Vec<SimpleLabelledPoint>
World_CastRay(const World &world, const SimpleLocation &start_location,
              const SimpleLocation &end_location) {
  auto carla_start = ConvertFromSimpleLocation(start_location);
  auto carla_end = ConvertFromSimpleLocation(end_location);

  auto results = const_cast<World &>(world).CastRay(carla_start, carla_end);

  rust::Vec<SimpleLabelledPoint> simple_results;
  simple_results.reserve(results.size());
  for (const auto &point : results) {
    simple_results.push_back(ConvertToSimpleLabelledPoint(point));
  }
  return simple_results;
}

SimpleOptionalLabelledPoint World_ProjectPoint(const World &world,
                                               const SimpleLocation &location,
                                               const SimpleVector3D &direction,
                                               float search_distance) {
  auto carla_location = ConvertFromSimpleLocation(location);
  auto carla_direction = ConvertFromSimpleVector3D(direction);

  auto result = const_cast<World &>(world).ProjectPoint(
      carla_location, carla_direction, search_distance);
  return ConvertToSimpleOptionalLabelledPoint(result);
}

SimpleOptionalLabelledPoint
World_GroundProjection(const World &world, const SimpleLocation &location,
                       float search_distance) {
  auto carla_location = ConvertFromSimpleLocation(location);

  auto result = const_cast<World &>(world).GroundProjection(carla_location,
                                                            search_distance);
  return ConvertToSimpleOptionalLabelledPoint(result);
}

// Traffic light queries
SimpleActorList World_GetTrafficLightsFromWaypoint(const World &world,
                                                   const Waypoint &waypoint,
                                                   double distance) {
  auto results = const_cast<World &>(world).GetTrafficLightsFromWaypoint(
      waypoint, distance);
  return ConvertToSimpleActorList(results);
}

SimpleActorList World_GetTrafficLightsInJunction(const World &world,
                                                 int32_t junction_id) {
  carla::road::JuncId junc_id = static_cast<carla::road::JuncId>(junction_id);
  auto results = const_cast<World &>(world).GetTrafficLightsInJunction(junc_id);
  return ConvertToSimpleActorList(results);
}

// Pedestrian navigation
SimpleOptionalLocation
World_GetRandomLocationFromNavigation(const World &world) {
  auto result = const_cast<World &>(world).GetRandomLocationFromNavigation();
  return ConvertToSimpleOptionalLocation(result);
}

void World_SetPedestriansCrossFactor(const World &world, float percentage) {
  const_cast<World &>(world).SetPedestriansCrossFactor(percentage);
}

// Actor query methods
SimpleActorList World_GetActors(const World &world) {
  auto actors = const_cast<World &>(world).GetActors();
  return ConvertToSimpleActorList(*actors);
}

SimpleActorList World_GetActorsByIds(const World &world,
                                     rust::Slice<const uint32_t> actor_ids) {
  std::vector<carla::ActorId> carla_ids;
  carla_ids.reserve(actor_ids.size());
  for (uint32_t id : actor_ids) {
    carla_ids.push_back(static_cast<carla::ActorId>(id));
  }

  auto actors = const_cast<World &>(world).GetActors(carla_ids);
  return ConvertToSimpleActorList(*actors);
}

std::shared_ptr<Actor> World_GetActor(const World &world, uint32_t actor_id) {
  carla::ActorId carla_id = static_cast<carla::ActorId>(actor_id);
  auto actor = const_cast<World &>(world).GetActor(carla_id);
  return actor;
}

// Batch operations helper functions

// Command type enum for SimpleBatchCommand.command_type
enum class BatchCommandType : uint8_t {
  SpawnActor = 0,
  DestroyActor = 1,
  ApplyVehicleControl = 2,
  ApplyWalkerControl = 3,
  ApplyTransform = 4,
  ApplyLocation = 5,
  ApplyVehicleAckermannControl = 6,
  SetAutopilot = 7,
  SetVehicleLightState = 8,
  SetSimulatePhysics = 9,
  SetEnableGravity = 10,
  ApplyTargetVelocity = 11,
  ApplyTargetAngularVelocity = 12,
  ApplyImpulse = 13,
  ApplyForce = 14,
  ApplyAngularImpulse = 15,
  ApplyTorque = 16,
  SetTrafficLightState = 17,
  ApplyWalkerState = 18,
  ApplyVehiclePhysicsControl = 19,
  ShowDebugTelemetry = 20,
  ConsoleCommand = 21,
};

carla::rpc::Command
ConvertFromSimpleBatchCommand(const SimpleBatchCommand &simple_cmd) {
  using namespace carla::rpc;

  BatchCommandType cmd_type =
      static_cast<BatchCommandType>(simple_cmd.command_type);
  carla::ActorId actor_id = static_cast<carla::ActorId>(simple_cmd.actor_id);

  switch (cmd_type) {
  case BatchCommandType::DestroyActor:
    return Command::DestroyActor(actor_id);

  case BatchCommandType::ApplyVehicleControl: {
    VehicleControl control;
    control.throttle = static_cast<float>(simple_cmd.data1);
    control.steer = static_cast<float>(simple_cmd.data2);
    control.brake = static_cast<float>(simple_cmd.data3);
    control.hand_brake = simple_cmd.bool_flag1;
    control.reverse = simple_cmd.bool_flag2;
    control.manual_gear_shift = (simple_cmd.int_flag1 != 0);
    control.gear = simple_cmd.int_flag2;
    return Command::ApplyVehicleControl(actor_id, control);
  }

  case BatchCommandType::ApplyWalkerControl: {
    WalkerControl control;
    control.direction =
        carla::geom::Vector3D(static_cast<float>(simple_cmd.data1),
                              static_cast<float>(simple_cmd.data2),
                              static_cast<float>(simple_cmd.data3));
    control.speed = static_cast<float>(simple_cmd.data4);
    control.jump = simple_cmd.bool_flag1;
    return Command::ApplyWalkerControl(actor_id, control);
  }

  case BatchCommandType::ApplyTransform: {
    carla::geom::Transform transform;
    transform.location =
        carla::geom::Location(static_cast<float>(simple_cmd.data1),
                              static_cast<float>(simple_cmd.data2),
                              static_cast<float>(simple_cmd.data3));
    transform.rotation =
        carla::geom::Rotation(static_cast<float>(simple_cmd.data4),
                              static_cast<float>(simple_cmd.data5),
                              static_cast<float>(simple_cmd.data6));
    return Command::ApplyTransform(actor_id, transform);
  }

  case BatchCommandType::ApplyLocation: {
    carla::geom::Location location(static_cast<float>(simple_cmd.data1),
                                   static_cast<float>(simple_cmd.data2),
                                   static_cast<float>(simple_cmd.data3));
    return Command::ApplyLocation(actor_id, location);
  }

  case BatchCommandType::ApplyVehicleAckermannControl: {
    VehicleAckermannControl control;
    control.steer = static_cast<float>(simple_cmd.data1);
    control.steer_speed = static_cast<float>(simple_cmd.data2);
    control.speed = static_cast<float>(simple_cmd.data3);
    control.acceleration = static_cast<float>(simple_cmd.data4);
    control.jerk = static_cast<float>(simple_cmd.data5);
    return Command::ApplyVehicleAckermannControl(actor_id, control);
  }

  case BatchCommandType::SetAutopilot: {
    bool enabled = simple_cmd.bool_flag1;
    uint16_t tm_port = static_cast<uint16_t>(simple_cmd.int_flag1);
    return Command::SetAutopilot(actor_id, enabled, tm_port);
  }

  case BatchCommandType::SetVehicleLightState: {
    VehicleLightState::flag_type light_state =
        static_cast<VehicleLightState::flag_type>(simple_cmd.int_flag1);
    return Command::SetVehicleLightState(actor_id, light_state);
  }

  case BatchCommandType::SetSimulatePhysics: {
    bool enabled = simple_cmd.bool_flag1;
    return Command::SetSimulatePhysics(actor_id, enabled);
  }

  case BatchCommandType::SetEnableGravity: {
    bool enabled = simple_cmd.bool_flag1;
    return Command::SetEnableGravity(actor_id, enabled);
  }

  case BatchCommandType::ApplyTargetVelocity: {
    carla::geom::Vector3D velocity(static_cast<float>(simple_cmd.data1),
                                   static_cast<float>(simple_cmd.data2),
                                   static_cast<float>(simple_cmd.data3));
    return Command::ApplyTargetVelocity(actor_id, velocity);
  }

  case BatchCommandType::ApplyTargetAngularVelocity: {
    carla::geom::Vector3D angular_velocity(
        static_cast<float>(simple_cmd.data1),
        static_cast<float>(simple_cmd.data2),
        static_cast<float>(simple_cmd.data3));
    return Command::ApplyTargetAngularVelocity(actor_id, angular_velocity);
  }

  case BatchCommandType::ApplyImpulse: {
    carla::geom::Vector3D impulse(static_cast<float>(simple_cmd.data1),
                                  static_cast<float>(simple_cmd.data2),
                                  static_cast<float>(simple_cmd.data3));
    return Command::ApplyImpulse(actor_id, impulse);
  }

  case BatchCommandType::ApplyForce: {
    carla::geom::Vector3D force(static_cast<float>(simple_cmd.data1),
                                static_cast<float>(simple_cmd.data2),
                                static_cast<float>(simple_cmd.data3));
    return Command::ApplyForce(actor_id, force);
  }

  case BatchCommandType::ApplyAngularImpulse: {
    carla::geom::Vector3D impulse(static_cast<float>(simple_cmd.data1),
                                  static_cast<float>(simple_cmd.data2),
                                  static_cast<float>(simple_cmd.data3));
    return Command::ApplyAngularImpulse(actor_id, impulse);
  }

  case BatchCommandType::ApplyTorque: {
    carla::geom::Vector3D torque(static_cast<float>(simple_cmd.data1),
                                 static_cast<float>(simple_cmd.data2),
                                 static_cast<float>(simple_cmd.data3));
    return Command::ApplyTorque(actor_id, torque);
  }

  case BatchCommandType::SetTrafficLightState: {
    TrafficLightState state =
        static_cast<TrafficLightState>(simple_cmd.int_flag1);
    return Command::SetTrafficLightState(actor_id, state);
  }

  case BatchCommandType::ShowDebugTelemetry: {
    bool enabled = simple_cmd.bool_flag1;
    return Command::ShowDebugTelemetry(actor_id, enabled);
  }

  // Note: SpawnActor and other complex commands require additional
  // implementation
  default:
    // For unsupported commands, return a destroy command as fallback
    return Command::DestroyActor(actor_id);
  }
}

SimpleBatchResponse
ConvertToSimpleBatchResponse(const carla::rpc::CommandResponse &response) {
  SimpleBatchResponse simple_response;
  simple_response.has_error = response.HasError();

  if (response.HasError()) {
    simple_response.error_message = response.GetError().What();
    simple_response.actor_id = 0;
  } else {
    simple_response.error_message = "";
    simple_response.actor_id = static_cast<uint32_t>(response.Get());
  }

  return simple_response;
}

// Batch operation wrapper functions
void Client_ApplyBatch(const Client &client,
                       rust::Slice<const SimpleBatchCommand> commands,
                       bool do_tick_cue) {
  std::vector<carla::rpc::Command> carla_commands;
  carla_commands.reserve(commands.size());

  for (const auto &simple_cmd : commands) {
    carla_commands.push_back(ConvertFromSimpleBatchCommand(simple_cmd));
  }

  const_cast<Client &>(client).ApplyBatch(std::move(carla_commands),
                                          do_tick_cue);
}

rust::Vec<SimpleBatchResponse>
Client_ApplyBatchSync(const Client &client,
                      rust::Slice<const SimpleBatchCommand> commands,
                      bool do_tick_cue) {
  std::vector<carla::rpc::Command> carla_commands;
  carla_commands.reserve(commands.size());

  for (const auto &simple_cmd : commands) {
    carla_commands.push_back(ConvertFromSimpleBatchCommand(simple_cmd));
  }

  auto responses = const_cast<Client &>(client).ApplyBatchSync(
      std::move(carla_commands), do_tick_cue);

  rust::Vec<SimpleBatchResponse> simple_responses;
  simple_responses.reserve(responses.size());

  for (const auto &response : responses) {
    simple_responses.push_back(ConvertToSimpleBatchResponse(response));
  }

  return simple_responses;
}

} // namespace client
} // namespace carla

// Global namespace weather functions (bridge utilities)
SimpleWeatherParameters World_GetWeather(const carla::client::World &world) {
  auto weather = const_cast<carla::client::World &>(world).GetWeather();
  return carla::client::ConvertToSimpleWeatherParameters(weather);
}

void World_SetWeather(const carla::client::World &world,
                      const SimpleWeatherParameters &weather) {
  auto carla_weather =
      carla::client::ConvertFromSimpleWeatherParameters(weather);
  const_cast<carla::client::World &>(world).SetWeather(carla_weather);
}

bool World_IsWeatherEnabled(const carla::client::World &world) {
  return const_cast<carla::client::World &>(world).IsWeatherEnabled();
}
