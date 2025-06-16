#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rust/cxx.h"

// Include the actual CARLA headers
#include <carla/client/Actor.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Junction.h>
#include <carla/client/Light.h>
#include <carla/client/LightManager.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/ServerSideSensor.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/TrafficSign.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Walker.h>
#include <carla/client/WalkerAIController.h>
#include <carla/client/Waypoint.h>
#include <carla/client/World.h>
#include <carla/geom/GeoLocation.h>
#include <carla/geom/Transform.h>
#include <carla/ros2/ROS2.h>
#include <carla/trafficmanager/TrafficManager.h>

// Forward declare our simple types from the generated header
struct SimpleVector2D;
struct SimpleVector3D;
struct SimpleLocation;
struct SimpleRotation;
struct SimpleTransform;
struct SimpleBoundingBox;
struct SimpleVehicleControl;
struct SimpleAckermannControl;
struct SimpleVehiclePhysicsControl;
struct SimpleWheelPhysicsControl;
struct SimpleGearPhysicsControl;
struct SimpleVehicleDoor;
struct SimpleVehicleTelemetryData;
struct SimpleWalkerControl;
struct SimpleWalkerDestination;
struct SimpleTrafficLightState;
struct SimpleLiDARPoint;
struct SimpleLiDARData;
struct SimpleRadarDetection;
struct SimpleIMUData;
struct SimpleGNSSData;
struct SimpleImageData;
struct SimpleCollisionData;
struct SimpleCrossedLaneMarking;
struct SimpleColor;
struct SimpleDebugPoint;
struct SimpleDebugLine;
struct SimpleDebugArrow;
struct SimpleDebugBox;
struct SimpleDebugString;
struct SimpleLaneMarking;
struct SimpleWaypointInfo;
struct SimpleJunction;
struct SimpleGeoLocation;
struct SimpleTrafficManagerConfig;
struct SimpleTrafficManagerVehicleConfig;
struct SimpleTrafficManagerAction;
struct SimpleTrafficManagerStats;
struct SimpleTimestamp;
struct SimpleEpisodeSettings;
struct SimpleWeatherParameters;
struct SimpleRecorderInfo;
struct SimpleRecorderCollision;
struct SimpleRecorderPosition;
struct SimpleLabelledPoint;
struct SimpleOptionalLabelledPoint;
struct SimpleOptionalLocation;
struct SimpleActorId;
struct SimpleActorList;
struct SimpleBlueprintList;
struct SimpleBatchCommand;
struct SimpleBatchResponse;
struct SimpleEnvironmentObject;
struct SimpleLandmark;
struct SimpleTextureFloatColor;
struct SimpleTextureColor;
struct SimpleLevelBoundingBox;
struct SimpleDVSEvent;
struct SimpleDVSEventArray;
struct SimpleObstacleDetectionEvent;
struct SimpleSemanticLidarDetection;
struct SimpleSemanticLidarData;
struct SimpleRssResponse;
struct SimpleLightState;
struct SimpleLight;
struct SimpleLightManager;
struct SimpleVehicleControlCommand;

// CXX Bridge functions
namespace carla {
namespace client {

// Create a new CARLA client
std::unique_ptr<Client> create_client(rust::Str host, uint16_t port,
                                      size_t worker_threads);

// Client wrapper functions
void Client_SetTimeout(Client &client, double timeout_seconds);
double Client_GetTimeout(Client &client);
rust::String Client_GetServerVersion(const Client &client);
std::shared_ptr<World> Client_GetWorld(const Client &client);

// World/Map management functions
rust::Vec<rust::String> Client_GetAvailableMaps(const Client &client);
std::shared_ptr<World> Client_LoadWorld(const Client &client,
                                        rust::Str map_name);
std::shared_ptr<World> Client_ReloadWorld(const Client &client,
                                          bool reset_settings);
std::shared_ptr<World> Client_GenerateOpenDriveWorld(const Client &client,
                                                     rust::Str opendrive);

// Recording functions
rust::String Client_StartRecorder(const Client &client, rust::Str filename,
                                  bool additional_data);
void Client_StopRecorder(const Client &client);
rust::String Client_ShowRecorderFileInfo(const Client &client,
                                         rust::Str filename, bool show_all);
rust::String Client_ShowRecorderCollisions(const Client &client,
                                           rust::Str filename, uint8_t type1,
                                           uint8_t type2);
rust::String Client_ShowRecorderActorsBlocked(const Client &client,
                                              rust::Str filename,
                                              double min_time,
                                              double min_distance);

// Playback functions
rust::String Client_ReplayFile(const Client &client, rust::Str filename,
                               double start_time, double duration,
                               uint32_t follow_id, bool replay_sensors);
void Client_StopReplayer(const Client &client, bool keep_actors);
void Client_SetReplayerTimeFactor(const Client &client, double time_factor);
void Client_SetReplayerIgnoreHero(const Client &client, bool ignore_hero);
void Client_SetReplayerIgnoreSpectator(const Client &client,
                                       bool ignore_spectator);

// Batch operation functions
void Client_ApplyBatch(const Client &client,
                       rust::Slice<const SimpleBatchCommand> commands,
                       bool do_tick_cue);
rust::Vec<SimpleBatchResponse>
Client_ApplyBatchSync(const Client &client,
                      rust::Slice<const SimpleBatchCommand> commands,
                      bool do_tick_cue);

// Debug drawing functions - on World, not Client
void World_DrawDebugPoint(const World &world, const SimpleDebugPoint &point);
void World_DrawDebugLine(const World &world, const SimpleDebugLine &line);
void World_DrawDebugArrow(const World &world, const SimpleDebugArrow &arrow);
void World_DrawDebugBox(const World &world, const SimpleDebugBox &box_shape);
void World_DrawDebugString(const World &world, const SimpleDebugString &string);

// World wrapper functions
uint64_t World_GetId(const World &world);
std::shared_ptr<BlueprintLibrary> World_GetBlueprintLibrary(const World &world);
std::shared_ptr<Actor> World_GetSpectator(const World &world);
uint64_t World_Tick(const World &world, double timeout_seconds);
SimpleTimestamp World_GetSnapshot(const World &world);
std::shared_ptr<Actor> World_SpawnActor(const World &world,
                                        const ActorBlueprint &blueprint,
                                        const SimpleTransform &transform,
                                        const Actor *parent);
std::shared_ptr<Actor> World_TrySpawnActor(const World &world,
                                           const ActorBlueprint &blueprint,
                                           const SimpleTransform &transform,
                                           const Actor *parent);
std::shared_ptr<Map> World_GetMap(const World &world);
SimpleEpisodeSettings World_GetSettings(const World &world);
uint64_t World_ApplySettings(const World &world,
                             const SimpleEpisodeSettings &settings,
                             double timeout_seconds);

// World interaction functions
// Ray casting functionality
rust::Vec<SimpleLabelledPoint>
World_CastRay(const World &world, const SimpleLocation &start_location,
              const SimpleLocation &end_location);
SimpleOptionalLabelledPoint World_ProjectPoint(const World &world,
                                               const SimpleLocation &location,
                                               const SimpleVector3D &direction,
                                               float search_distance);
SimpleOptionalLabelledPoint
World_GroundProjection(const World &world, const SimpleLocation &location,
                       float search_distance);

// Traffic light queries
SimpleActorList World_GetTrafficLightsFromWaypoint(const World &world,
                                                   const Waypoint &waypoint,
                                                   double distance);
SimpleActorList World_GetTrafficLightsInJunction(const World &world,
                                                 int32_t junction_id);

// Pedestrian navigation
SimpleOptionalLocation
World_GetRandomLocationFromNavigation(const World &world);
void World_SetPedestriansCrossFactor(const World &world, float percentage);

// Actor query methods
SimpleActorList World_GetActors(const World &world);
SimpleActorList World_GetActorsByIds(const World &world,
                                     rust::Slice<const uint32_t> actor_ids);
std::shared_ptr<Actor> World_GetActor(const World &world, uint32_t actor_id);

// Advanced world features - Map layer management
void World_LoadLevelLayer(const World &world, uint8_t map_layers);
void World_UnloadLevelLayer(const World &world, uint8_t map_layers);

// Environment object queries
rust::Vec<SimpleLevelBoundingBox> World_GetLevelBBs(const World &world,
                                                    uint8_t queried_tag);
rust::Vec<SimpleEnvironmentObject>
World_GetEnvironmentObjects(const World &world, uint8_t queried_tag);
void World_EnableEnvironmentObjects(const World &world,
                                    rust::Slice<const uint64_t> env_objects_ids,
                                    bool enable);

// Advanced traffic light management
void World_ResetAllTrafficLights(const World &world);
void World_FreezeAllTrafficLights(const World &world, bool frozen);
rust::Vec<SimpleBatchCommand> World_GetVehiclesLightStates(const World &world);

// Texture and material application
void World_ApplyColorTextureToObject(const World &world, rust::Str object_name,
                                     const SimpleTextureColor &texture,
                                     uint8_t material_type);
void World_ApplyFloatColorTextureToObject(
    const World &world, rust::Str object_name,
    const SimpleTextureFloatColor &texture, uint8_t material_type);
rust::Vec<rust::String> World_GetNamesOfAllObjects(const World &world);

// Pedestrian navigation
void World_SetPedestriansSeed(const World &world, uint32_t seed);

// Light management functions
std::shared_ptr<LightManager> World_GetLightManager(const World &world);
rust::Vec<SimpleLight>
LightManager_GetAllLights(const LightManager &light_manager, uint8_t group);
void LightManager_SetDayNightCycle(const LightManager &light_manager,
                                   bool active);
void LightManager_TurnOnLights(const LightManager &light_manager,
                               rust::Slice<const uint32_t> light_ids);
void LightManager_TurnOffLights(const LightManager &light_manager,
                                rust::Slice<const uint32_t> light_ids);
void LightManager_SetLightIntensities(const LightManager &light_manager,
                                      rust::Slice<const uint32_t> light_ids,
                                      float intensity);
void LightManager_SetLightColors(const LightManager &light_manager,
                                 rust::Slice<const uint32_t> light_ids,
                                 SimpleColor color);
void LightManager_SetLightStates(const LightManager &light_manager,
                                 rust::Slice<const uint32_t> light_ids,
                                 SimpleLightState state);

// Landmark and signal management functions
rust::Vec<SimpleLandmark> Map_GetAllLandmarks(const Map &map);
rust::Vec<SimpleLandmark> Map_GetLandmarksFromId(const Map &map, rust::Str id);
rust::Vec<SimpleLandmark> Map_GetAllLandmarksOfType(const Map &map,
                                                    rust::Str type_);
rust::Vec<SimpleLandmark>
Waypoint_GetAllLandmarksInDistance(const Waypoint &waypoint, double distance,
                                   bool stop_at_junction);
rust::Vec<SimpleLandmark>
Waypoint_GetLandmarksOfTypeInDistance(const Waypoint &waypoint, double distance,
                                      rust::Str filter_type,
                                      bool stop_at_junction);

// Individual landmark property functions are available in C++ but not exposed
// via FFI due to CXX limitations

// Actor wrapper functions
uint32_t Actor_GetId(const Actor &actor);
rust::String Actor_GetTypeId(const Actor &actor);
rust::String Actor_GetDisplayId(const Actor &actor);
SimpleLocation Actor_GetLocation(const Actor &actor);
SimpleTransform Actor_GetTransform(const Actor &actor);
void Actor_SetLocation(const Actor &actor, const SimpleLocation &location);
void Actor_SetTransform(const Actor &actor, const SimpleTransform &transform);
bool Actor_Destroy(const Actor &actor);
bool Actor_IsAlive(const Actor &actor);

// BlueprintLibrary wrapper functions
std::shared_ptr<ActorBlueprint>
BlueprintLibrary_Find(const BlueprintLibrary &library, rust::Str id);
size_t BlueprintLibrary_Size(const BlueprintLibrary &library);
SimpleBlueprintList BlueprintLibrary_GetAll(const BlueprintLibrary &library);
SimpleBlueprintList
BlueprintLibrary_FilterByTags(const BlueprintLibrary &library,
                              rust::Vec<rust::String> tags);
SimpleBlueprintList
BlueprintLibrary_FilterByAttribute(const BlueprintLibrary &library,
                                   rust::Str attribute_name,
                                   rust::Str attribute_value);
SimpleBlueprintList BlueprintLibrary_Search(const BlueprintLibrary &library,
                                            rust::Str search_term);

// ActorBlueprint wrapper functions
rust::String ActorBlueprint_GetId(const ActorBlueprint &blueprint);
rust::Vec<rust::String> ActorBlueprint_GetTags(const ActorBlueprint &blueprint);
bool ActorBlueprint_MatchTags(const ActorBlueprint &blueprint,
                              rust::Str wildcard_pattern);
bool ActorBlueprint_ContainsTag(const ActorBlueprint &blueprint, rust::Str tag);
bool ActorBlueprint_ContainsAttribute(const ActorBlueprint &blueprint,
                                      rust::Str id);
void ActorBlueprint_SetAttribute(const ActorBlueprint &blueprint, rust::Str id,
                                 rust::Str value);
rust::Vec<rust::String>
ActorBlueprint_GetAttributeIds(const ActorBlueprint &blueprint);

// Actor casting functions
std::shared_ptr<Vehicle> Actor_CastToVehicle(const Actor &actor);
std::shared_ptr<Actor> Vehicle_CastToActor(const Vehicle &vehicle);
std::shared_ptr<Walker> Actor_CastToWalker(const Actor &actor);
std::shared_ptr<Actor> Walker_CastToActor(const Walker &walker);
std::shared_ptr<WalkerAIController>
Actor_CastToWalkerAIController(const Actor &actor);
std::shared_ptr<Sensor> Actor_CastToSensor(const Actor &actor);
std::shared_ptr<Actor> Sensor_CastToActor(const Sensor &sensor);
std::shared_ptr<TrafficLight> Actor_CastToTrafficLight(const Actor &actor);
std::shared_ptr<TrafficSign> Actor_CastToTrafficSign(const Actor &actor);

// Vehicle wrapper functions
void Vehicle_ApplyControl(const Vehicle &vehicle,
                          const SimpleVehicleControl &control);
SimpleVehicleControl Vehicle_GetControl(const Vehicle &vehicle);
void Vehicle_SetAutopilot(const Vehicle &vehicle, bool enabled,
                          uint16_t tm_port);
float Vehicle_GetSpeed(const Vehicle &vehicle);
float Vehicle_GetSpeedLimit(const Vehicle &vehicle);
void Vehicle_SetLightState(const Vehicle &vehicle, uint32_t light_state);
uint32_t Vehicle_GetLightState(const Vehicle &vehicle);

// Vehicle telemetry
SimpleVehicleTelemetryData Vehicle_GetTelemetryData(const Vehicle &vehicle);

// Vehicle ID and actor properties
uint32_t Vehicle_GetId(const Vehicle &vehicle);
rust::String Vehicle_GetTypeId(const Vehicle &vehicle);
SimpleTransform Vehicle_GetTransform(const Vehicle &vehicle);
void Vehicle_SetTransform(const Vehicle &vehicle,
                          const SimpleTransform &transform);
bool Vehicle_IsAlive(const Vehicle &vehicle);
bool Vehicle_Destroy(const Vehicle &vehicle);

// Advanced vehicle control
void Vehicle_ApplyAckermannControl(const Vehicle &vehicle,
                                   const SimpleAckermannControl &control);
SimpleAckermannControl Vehicle_GetAckermannControl(const Vehicle &vehicle);
void Vehicle_ApplyPhysicsControl(const Vehicle &vehicle,
                                 const SimpleVehiclePhysicsControl &control);
SimpleVehiclePhysicsControl Vehicle_GetPhysicsControl(const Vehicle &vehicle);

// Vehicle telemetry
SimpleVector3D Vehicle_GetVelocity(const Vehicle &vehicle);
SimpleVector3D Vehicle_GetAngularVelocity(const Vehicle &vehicle);
SimpleVector3D Vehicle_GetAcceleration(const Vehicle &vehicle);
float Vehicle_GetTireFriction(const Vehicle &vehicle);
float Vehicle_GetEngineRpm(const Vehicle &vehicle);
float Vehicle_GetGearRatio(const Vehicle &vehicle);

// Vehicle doors (CARLA 0.10.0)
void Vehicle_OpenDoor(const Vehicle &vehicle, uint32_t door_type);
void Vehicle_CloseDoor(const Vehicle &vehicle, uint32_t door_type);
bool Vehicle_IsDoorOpen(const Vehicle &vehicle, uint32_t door_type);
rust::Vec<SimpleVehicleDoor> Vehicle_GetDoorStates(const Vehicle &vehicle);

// Wheel physics
rust::Vec<SimpleWheelPhysicsControl>
Vehicle_GetWheelPhysicsControls(const Vehicle &vehicle);
void Vehicle_SetWheelPhysicsControls(
    const Vehicle &vehicle,
    rust::Slice<const SimpleWheelPhysicsControl> wheels);

// Gear physics
rust::Vec<SimpleGearPhysicsControl>
Vehicle_GetGearPhysicsControls(const Vehicle &vehicle);
void Vehicle_SetGearPhysicsControls(
    const Vehicle &vehicle, rust::Slice<const SimpleGearPhysicsControl> gears);

// Wheel steer angle
float Vehicle_GetWheelSteerAngle(const Vehicle &vehicle,
                                 uint32_t wheel_location);

// Actor physics methods (from Actor base class)
void Vehicle_SetSimulatePhysics(const Vehicle &vehicle, bool enabled);
void Vehicle_AddImpulse(const Vehicle &vehicle, const SimpleVector3D &impulse);
void Vehicle_AddImpulseAtLocation(const Vehicle &vehicle,
                                  const SimpleVector3D &impulse,
                                  const SimpleVector3D &location);
void Vehicle_AddForce(const Vehicle &vehicle, const SimpleVector3D &force);
void Vehicle_AddForceAtLocation(const Vehicle &vehicle,
                                const SimpleVector3D &force,
                                const SimpleVector3D &location);
void Vehicle_AddTorque(const Vehicle &vehicle, const SimpleVector3D &torque);

// Walker wrapper functions
void Walker_ApplyControl(const Walker &walker,
                         const SimpleWalkerControl &control);
SimpleWalkerControl Walker_GetControl(const Walker &walker);
float Walker_GetSpeed(const Walker &walker);

// Walker pose control functions (simplified)
void Walker_BlendPose(const Walker &walker, float blend);
void Walker_ShowPose(const Walker &walker);
void Walker_HidePose(const Walker &walker);
void Walker_GetPoseFromAnimation(const Walker &walker);

// Walker Actor interface functions
rust::String Walker_GetTypeId(const Walker &walker);
SimpleTransform Walker_GetTransform(const Walker &walker);
void Walker_SetTransform(const Walker &walker,
                         const SimpleTransform &transform);
SimpleVector3D Walker_GetVelocity(const Walker &walker);
SimpleVector3D Walker_GetAngularVelocity(const Walker &walker);
SimpleVector3D Walker_GetAcceleration(const Walker &walker);
bool Walker_IsAlive(const Walker &walker);
bool Walker_Destroy(const Walker &walker);
void Walker_SetSimulatePhysics(const Walker &walker, bool enabled);
void Walker_AddImpulse(const Walker &walker, const SimpleVector3D &impulse);
void Walker_AddForce(const Walker &walker, const SimpleVector3D &force);
void Walker_AddTorque(const Walker &walker, const SimpleVector3D &torque);

// Walker AI Controller functions
void WalkerAIController_Start(const WalkerAIController &controller);
void WalkerAIController_Stop(const WalkerAIController &controller);
void WalkerAIController_SetMaxSpeed(const WalkerAIController &controller,
                                    float max_speed);
void WalkerAIController_GoToLocation(
    const WalkerAIController &controller,
    const SimpleWalkerDestination &destination);
SimpleWalkerDestination
WalkerAIController_GetRandomLocation(const WalkerAIController &controller);
bool WalkerAIController_HasValidDestination(
    const WalkerAIController &controller);

// Sensor wrapper functions
void Sensor_Stop(const Sensor &sensor);
bool Sensor_IsListening(const Sensor &sensor);
void Sensor_Listen(const Sensor &sensor);

// Sensor data retrieval functions
SimpleImageData Sensor_GetLastImageData(const Sensor &sensor);
bool Sensor_GetImageDataBuffer(const Sensor &sensor,
                               rust::Slice<uint8_t> buffer);

// Camera specific functions
bool Sensor_IsCamera(const Sensor &sensor);
uint8_t Sensor_GetCameraType(const Sensor &sensor);
rust::Vec<SimpleLiDARPoint> Sensor_GetLastLiDARData(const Sensor &sensor);
SimpleLiDARData Sensor_GetLastLiDARDataFull(const Sensor &sensor);
rust::Vec<SimpleRadarDetection> Sensor_GetLastRadarData(const Sensor &sensor);
SimpleIMUData Sensor_GetLastIMUData(const Sensor &sensor);
SimpleGNSSData Sensor_GetLastGNSSData(const Sensor &sensor);
SimpleCollisionData Sensor_GetLastCollisionData(const Sensor &sensor);
rust::Vec<SimpleCrossedLaneMarking>
Sensor_GetLastLaneInvasionData(const Sensor &sensor);
bool Sensor_HasNewData(const Sensor &sensor);

// Advanced sensor data retrieval functions
SimpleDVSEventArray Sensor_GetLastDVSData(const Sensor &sensor);
SimpleObstacleDetectionEvent
Sensor_GetLastObstacleDetectionData(const Sensor &sensor);
SimpleSemanticLidarData Sensor_GetLastSemanticLidarData(const Sensor &sensor);
SimpleRssResponse Sensor_GetLastRssData(const Sensor &sensor);

// Sensor Actor interface functions
rust::String Sensor_GetTypeId(const Sensor &sensor);
SimpleTransform Sensor_GetTransform(const Sensor &sensor);
void Sensor_SetTransform(const Sensor &sensor,
                         const SimpleTransform &transform);
SimpleVector3D Sensor_GetVelocity(const Sensor &sensor);
SimpleVector3D Sensor_GetAngularVelocity(const Sensor &sensor);
SimpleVector3D Sensor_GetAcceleration(const Sensor &sensor);
bool Sensor_IsAlive(const Sensor &sensor);
bool Sensor_Destroy(const Sensor &sensor);
void Sensor_SetSimulatePhysics(const Sensor &sensor, bool enabled);
void Sensor_AddImpulse(const Sensor &sensor, const SimpleVector3D &impulse);
void Sensor_AddForce(const Sensor &sensor, const SimpleVector3D &force);
void Sensor_AddTorque(const Sensor &sensor, const SimpleVector3D &torque);

// Traffic Light wrapper functions
uint32_t TrafficLight_GetState(const TrafficLight &traffic_light);
void TrafficLight_SetState(const TrafficLight &traffic_light, uint32_t state);
float TrafficLight_GetElapsedTime(const TrafficLight &traffic_light);
void TrafficLight_SetRedTime(const TrafficLight &traffic_light, float red_time);
void TrafficLight_SetYellowTime(const TrafficLight &traffic_light,
                                float yellow_time);
void TrafficLight_SetGreenTime(const TrafficLight &traffic_light,
                               float green_time);
float TrafficLight_GetRedTime(const TrafficLight &traffic_light);
float TrafficLight_GetYellowTime(const TrafficLight &traffic_light);
float TrafficLight_GetGreenTime(const TrafficLight &traffic_light);
void TrafficLight_Freeze(const TrafficLight &traffic_light, bool freeze);
bool TrafficLight_IsFrozen(const TrafficLight &traffic_light);

// Traffic Light Actor interface functions
rust::String TrafficLight_GetTypeId(const TrafficLight &traffic_light);
SimpleTransform TrafficLight_GetTransform(const TrafficLight &traffic_light);
void TrafficLight_SetTransform(const TrafficLight &traffic_light,
                               const SimpleTransform &transform);
SimpleVector3D TrafficLight_GetVelocity(const TrafficLight &traffic_light);
SimpleVector3D
TrafficLight_GetAngularVelocity(const TrafficLight &traffic_light);
SimpleVector3D TrafficLight_GetAcceleration(const TrafficLight &traffic_light);
bool TrafficLight_IsAlive(const TrafficLight &traffic_light);
bool TrafficLight_Destroy(const TrafficLight &traffic_light);
void TrafficLight_SetSimulatePhysics(const TrafficLight &traffic_light,
                                     bool enabled);
void TrafficLight_AddImpulse(const TrafficLight &traffic_light,
                             const SimpleVector3D &impulse);
void TrafficLight_AddForce(const TrafficLight &traffic_light,
                           const SimpleVector3D &force);
void TrafficLight_AddTorque(const TrafficLight &traffic_light,
                            const SimpleVector3D &torque);

// Traffic Sign wrapper functions
rust::String TrafficSign_GetSignId(const TrafficSign &traffic_sign);
SimpleBoundingBox TrafficSign_GetTriggerVolume(const TrafficSign &traffic_sign);

// Traffic Sign Actor interface functions
rust::String TrafficSign_GetTypeId(const TrafficSign &traffic_sign);
SimpleTransform TrafficSign_GetTransform(const TrafficSign &traffic_sign);
void TrafficSign_SetTransform(const TrafficSign &traffic_sign,
                              const SimpleTransform &transform);
SimpleVector3D TrafficSign_GetVelocity(const TrafficSign &traffic_sign);
SimpleVector3D TrafficSign_GetAngularVelocity(const TrafficSign &traffic_sign);
SimpleVector3D TrafficSign_GetAcceleration(const TrafficSign &traffic_sign);
bool TrafficSign_IsAlive(const TrafficSign &traffic_sign);
bool TrafficSign_Destroy(const TrafficSign &traffic_sign);
void TrafficSign_SetSimulatePhysics(const TrafficSign &traffic_sign,
                                    bool enabled);
void TrafficSign_AddImpulse(const TrafficSign &traffic_sign,
                            const SimpleVector3D &impulse);
void TrafficSign_AddForce(const TrafficSign &traffic_sign,
                          const SimpleVector3D &force);
void TrafficSign_AddTorque(const TrafficSign &traffic_sign,
                           const SimpleVector3D &torque);

// Geometry utility functions
double Vector2D_Length(const SimpleVector2D &vector);
double Vector2D_SquaredLength(const SimpleVector2D &vector);
double Vector2D_Distance(const SimpleVector2D &a, const SimpleVector2D &b);
double Vector2D_DistanceSquared(const SimpleVector2D &a,
                                const SimpleVector2D &b);
double Vector2D_Dot(const SimpleVector2D &a, const SimpleVector2D &b);

double Vector3D_Length(const SimpleVector3D &vector);
double Vector3D_SquaredLength(const SimpleVector3D &vector);
double Vector3D_Distance(const SimpleVector3D &a, const SimpleVector3D &b);
double Vector3D_DistanceSquared(const SimpleVector3D &a,
                                const SimpleVector3D &b);
double Vector3D_Dot(const SimpleVector3D &a, const SimpleVector3D &b);
SimpleVector3D Vector3D_Cross(const SimpleVector3D &a, const SimpleVector3D &b);

double Location_Distance(const SimpleLocation &a, const SimpleLocation &b);
double Location_DistanceSquared(const SimpleLocation &a,
                                const SimpleLocation &b);

SimpleLocation Transform_TransformPoint(const SimpleTransform &transform,
                                        const SimpleLocation &point);
SimpleLocation Transform_InverseTransformPoint(const SimpleTransform &transform,
                                               const SimpleLocation &point);
SimpleVector3D Transform_GetForwardVector(const SimpleTransform &transform);
SimpleVector3D Transform_GetRightVector(const SimpleTransform &transform);
SimpleVector3D Transform_GetUpVector(const SimpleTransform &transform);

bool BoundingBox_Contains(const SimpleBoundingBox &bbox,
                          const SimpleLocation &point);
rust::Vec<SimpleLocation>
BoundingBox_GetVertices(const SimpleBoundingBox &bbox);

// Map wrapper functions
rust::String Map_GetName(const Map &map);
rust::String Map_GetOpenDrive(const Map &map);
rust::Vec<SimpleTransform> Map_GetRecommendedSpawnPoints(const Map &map);
std::shared_ptr<Waypoint> Map_GetWaypoint(const Map &map,
                                          const SimpleLocation &location,
                                          bool project_to_road,
                                          int32_t lane_type);
std::shared_ptr<Waypoint> Map_GetWaypointXODR(const Map &map, uint32_t road_id,
                                              int32_t lane_id, double s);
rust::Vec<SimpleWaypointInfo> Map_GenerateWaypoints(const Map &map,
                                                    double distance);
SimpleGeoLocation Map_GetGeoReference(const Map &map);
rust::Vec<SimpleLocation> Map_GetAllCrosswalkZones(const Map &map);
std::shared_ptr<Junction> Map_GetJunction(const Map &map,
                                          const Waypoint &waypoint);
rust::Vec<SimpleWaypointInfo> Map_GetTopology(const Map &map);

// Waypoint wrapper functions
uint64_t Waypoint_GetId(const Waypoint &waypoint);
uint32_t Waypoint_GetRoadId(const Waypoint &waypoint);
uint32_t Waypoint_GetSectionId(const Waypoint &waypoint);
int32_t Waypoint_GetLaneId(const Waypoint &waypoint);
double Waypoint_GetDistance(const Waypoint &waypoint);
SimpleTransform Waypoint_GetTransform(const Waypoint &waypoint);
uint32_t Waypoint_GetJunctionId(const Waypoint &waypoint);
bool Waypoint_IsJunction(const Waypoint &waypoint);
std::shared_ptr<Junction> Waypoint_GetJunction(const Waypoint &waypoint);
double Waypoint_GetLaneWidth(const Waypoint &waypoint);
uint32_t Waypoint_GetType(const Waypoint &waypoint);
std::shared_ptr<Waypoint> Waypoint_GetNext(const Waypoint &waypoint,
                                           double distance);
std::shared_ptr<Waypoint> Waypoint_GetPrevious(const Waypoint &waypoint,
                                               double distance);
std::shared_ptr<Waypoint> Waypoint_GetRight(const Waypoint &waypoint);
std::shared_ptr<Waypoint> Waypoint_GetLeft(const Waypoint &waypoint);
SimpleLaneMarking Waypoint_GetRightLaneMarking(const Waypoint &waypoint);
SimpleLaneMarking Waypoint_GetLeftLaneMarking(const Waypoint &waypoint);
uint8_t Waypoint_GetLaneChange(const Waypoint &waypoint);

// Junction wrapper functions
uint32_t Junction_GetId(const Junction &junction);
SimpleBoundingBox Junction_GetBoundingBox(const Junction &junction);

// Traffic Manager functions
std::shared_ptr<carla::traffic_manager::TrafficManager>
TrafficManager_GetInstance(const Client &client, uint16_t port);
void TrafficManager_RegisterVehicles(
    const carla::traffic_manager::TrafficManager &tm,
    const rust::Slice<const Vehicle *const> vehicles);
void TrafficManager_UnregisterVehicles(
    const carla::traffic_manager::TrafficManager &tm,
    const rust::Slice<const Vehicle *const> vehicles);
void TrafficManager_SetSynchronousMode(
    const carla::traffic_manager::TrafficManager &tm, bool mode);
bool TrafficManager_SynchronousTick(
    const carla::traffic_manager::TrafficManager &tm);
void TrafficManager_SetSynchronousModeTimeout(
    const carla::traffic_manager::TrafficManager &tm, double timeout_ms);

// Global configuration functions
void TrafficManager_SetGlobalSpeedPercentage(
    const carla::traffic_manager::TrafficManager &tm, float percentage);
void TrafficManager_SetGlobalLaneOffset(
    const carla::traffic_manager::TrafficManager &tm, float offset);
void TrafficManager_SetGlobalDistanceToLeadingVehicle(
    const carla::traffic_manager::TrafficManager &tm, float distance);
void TrafficManager_SetRandomDeviceSeed(
    const carla::traffic_manager::TrafficManager &tm, uint64_t seed);
void TrafficManager_SetOSMMode(const carla::traffic_manager::TrafficManager &tm,
                               bool mode);

// Vehicle-specific configuration functions
void TrafficManager_SetVehicleSpeedPercentage(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage);
void TrafficManager_SetVehicleDesiredSpeed(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float speed);
void TrafficManager_SetVehicleLaneOffset(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float offset);
void TrafficManager_SetVehicleAutoLaneChange(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    bool enable);
void TrafficManager_ForceVehicleLaneChange(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    bool direction);
void TrafficManager_SetVehicleDistanceToLeadingVehicle(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float distance);

// Traffic rule compliance functions
void TrafficManager_SetVehiclePercentageRunningLight(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage);
void TrafficManager_SetVehiclePercentageRunningSign(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage);
void TrafficManager_SetVehiclePercentageIgnoreWalkers(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage);
void TrafficManager_SetVehiclePercentageIgnoreVehicles(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage);

// Advanced features
void TrafficManager_SetHybridPhysicsMode(
    const carla::traffic_manager::TrafficManager &tm, bool mode);
void TrafficManager_SetHybridPhysicsRadius(
    const carla::traffic_manager::TrafficManager &tm, float radius);
void TrafficManager_SetCollisionDetection(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle1,
    const Vehicle &vehicle2, bool detect);
void TrafficManager_SetVehicleUpdateLights(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    bool update);

// Lane behavior percentages
void TrafficManager_SetVehicleKeepRightPercentage(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage);
void TrafficManager_SetVehicleRandomLeftLaneChangePercentage(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage);
void TrafficManager_SetVehicleRandomRightLaneChangePercentage(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle,
    float percentage);

// Respawn configuration
void TrafficManager_SetRespawnDormantVehicles(
    const carla::traffic_manager::TrafficManager &tm, bool enable);
void TrafficManager_SetRespawnBoundaries(
    const carla::traffic_manager::TrafficManager &tm, float lower_bound,
    float upper_bound);
void TrafficManager_SetMaxBoundaries(
    const carla::traffic_manager::TrafficManager &tm, float lower, float upper);

// Statistics and monitoring functions
SimpleTrafficManagerConfig
TrafficManager_GetConfig(const carla::traffic_manager::TrafficManager &tm);
SimpleTrafficManagerVehicleConfig TrafficManager_GetVehicleConfig(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle);
SimpleTrafficManagerStats
TrafficManager_GetStats(const carla::traffic_manager::TrafficManager &tm);
SimpleTrafficManagerAction
TrafficManager_GetNextAction(const carla::traffic_manager::TrafficManager &tm,
                             const Vehicle &vehicle);
bool TrafficManager_IsVehicleRegistered(
    const carla::traffic_manager::TrafficManager &tm, const Vehicle &vehicle);
uint16_t
TrafficManager_GetPort(const carla::traffic_manager::TrafficManager &tm);

// Lifecycle management
void TrafficManager_Shutdown(const carla::traffic_manager::TrafficManager &tm);
void TrafficManager_Reset();
void TrafficManager_Release();

} // namespace client
} // namespace carla

// Global namespace weather functions (bridge utilities)
SimpleWeatherParameters World_GetWeather(const carla::client::World &world);
void World_SetWeather(const carla::client::World &world,
                      const SimpleWeatherParameters &weather);
bool World_IsWeatherEnabled(const carla::client::World &world);

// Global namespace ROS2 integration functions (simplified sensor control only)
void Sensor_EnableForROS(const carla::client::Sensor &sensor);
void Sensor_DisableForROS(const carla::client::Sensor &sensor);
bool Sensor_IsEnabledForROS(const carla::client::Sensor &sensor);
