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
#include <carla/client/Sensor.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Walker.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>

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
struct SimpleWalkerControl;
struct SimpleTrafficLightState;
struct SimpleLiDARPoint;
struct SimpleRadarDetection;
struct SimpleIMUData;
struct SimpleGNSSData;
struct SimpleImageData;

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

// World wrapper functions
uint64_t World_GetId(const World &world);
std::shared_ptr<BlueprintLibrary> World_GetBlueprintLibrary(const World &world);
std::shared_ptr<Actor> World_GetSpectator(const World &world);
uint64_t World_Tick(const World &world, double timeout_seconds);
std::shared_ptr<Actor> World_SpawnActor(const World &world,
                                        const ActorBlueprint &blueprint,
                                        const SimpleTransform &transform,
                                        const Actor *parent);
std::shared_ptr<Actor> World_TrySpawnActor(const World &world,
                                           const ActorBlueprint &blueprint,
                                           const SimpleTransform &transform,
                                           const Actor *parent);

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

// Actor casting functions
std::shared_ptr<Vehicle> Actor_CastToVehicle(const Actor &actor);
std::shared_ptr<Walker> Actor_CastToWalker(const Actor &actor);
std::shared_ptr<Sensor> Actor_CastToSensor(const Actor &actor);
std::shared_ptr<TrafficLight> Actor_CastToTrafficLight(const Actor &actor);

// Vehicle wrapper functions
void Vehicle_ApplyControl(const Vehicle &vehicle,
                          const SimpleVehicleControl &control);
SimpleVehicleControl Vehicle_GetControl(const Vehicle &vehicle);
void Vehicle_SetAutopilot(const Vehicle &vehicle, bool enabled);
float Vehicle_GetSpeed(const Vehicle &vehicle);
float Vehicle_GetSpeedLimit(const Vehicle &vehicle);
void Vehicle_SetLightState(const Vehicle &vehicle, uint32_t light_state);
uint32_t Vehicle_GetLightState(const Vehicle &vehicle);

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

// Walker wrapper functions
void Walker_ApplyControl(const Walker &walker,
                         const SimpleWalkerControl &control);
SimpleWalkerControl Walker_GetControl(const Walker &walker);
float Walker_GetSpeed(const Walker &walker);

// Sensor wrapper functions
void Sensor_Stop(const Sensor &sensor);
bool Sensor_IsListening(const Sensor &sensor);
void Sensor_Listen(const Sensor &sensor);

// Sensor data retrieval functions
SimpleImageData Sensor_GetLastImageData(const Sensor &sensor);
bool Sensor_GetImageDataBuffer(const Sensor &sensor,
                               rust::Slice<uint8_t> buffer);
rust::Vec<SimpleLiDARPoint> Sensor_GetLastLiDARData(const Sensor &sensor);
rust::Vec<SimpleRadarDetection> Sensor_GetLastRadarData(const Sensor &sensor);
SimpleIMUData Sensor_GetLastIMUData(const Sensor &sensor);
SimpleGNSSData Sensor_GetLastGNSSData(const Sensor &sensor);
bool Sensor_HasNewData(const Sensor &sensor);

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

} // namespace client
} // namespace carla
