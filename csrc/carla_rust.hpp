#pragma once

#include <memory>
#include <string>
#include "rust/cxx.h"
#include "carla/Time.h"
#include "carla/client/Client.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/World.h"
#include "carla/client/Map.h"
#include "carla/client/Actor.h"
#include "carla/client/ActorBlueprint.h"
#include "carla/client/ActorAttribute.h"

using namespace std;

using carla::time_duration;
using carla::client::Client;
using carla::client::Map;
using carla::client::World;
using carla::client::BlueprintLibrary;
using carla::client::Actor;
using carla::client::ActorBlueprint;
using carla::client::ActorAttribute;
using carla::geom::Transform;
using carla::geom::Location;
using carla::geom::Rotation;
using carla::geom::Vector2D;
using carla::geom::Vector3D;
using SharedMap = carla::SharedPtr<Map>;
using SharedBlueprintLibrary = carla::SharedPtr<BlueprintLibrary>;
using SharedActor = carla::SharedPtr<Actor>;

// time_duration
// unique_ptr<time_duration> time_duration_from_millis(size_t millis);

// Client
unique_ptr<Client> client_new(const string &host, uint16_t port);
rust::String client_get_client_version(const Client &client);
rust::String client_get_server_version(const Client &client);
unique_ptr<World> client_load_world(const Client &client, const string &map_name);
unique_ptr<World> client_get_world(const Client &client);
size_t client_get_timeout_millis(Client &client);
void client_set_timeout_millis(Client &client, size_t millis);

// World
unique_ptr<SharedMap> world_get_map(const World &world);
unique_ptr<SharedBlueprintLibrary> world_get_blueprint_library(const World &world);
unique_ptr<SharedActor> world_get_spectator(const World &world);
unique_ptr<SharedActor> world_try_spawn_actor(World &world,
                                              const ActorBlueprint &blueprint,
                                              const Transform &transform,
                                              const SharedActor *parent);
// BlueprintLibrary
unique_ptr<SharedBlueprintLibrary> bp_filter(const SharedBlueprintLibrary &bp, const string &wildcard_pattern);
const BlueprintLibrary *bp_to_raw(const SharedBlueprintLibrary &bp);

// Actor
unique_ptr<Location> actor_get_location(const SharedActor &actor);
unique_ptr<Transform> actor_get_transform(const SharedActor &actor);
unique_ptr<Vector3D> actor_get_velocity(const SharedActor &actor);
unique_ptr<Vector3D> actor_get_acceleration(const SharedActor &actor);
unique_ptr<Vector3D> actor_get_angular_velocity(const SharedActor &actor);

// ActorBlueprint
unique_ptr<ActorBlueprint> actor_bp_copy(const ActorBlueprint &from);
void actor_bp_set_attribute(ActorBlueprint &actor_bp, const string &id, const string &value);

// ActorAttribute

// Vector2D
unique_ptr<Vector2D> vec2d_new(float x, float y);
float vec2d_x(const Vector2D &vec2d);
float vec2d_y(const Vector2D &vec2d);

// Vector3D
unique_ptr<Vector3D> vec3d_new(float x, float y, float z);
float vec3d_x(const Vector3D &vec3d);
float vec3d_y(const Vector3D &vec3d);
float vec3d_z(const Vector3D &vec3d);

// Location
unique_ptr<Location> location_from_xyz(float x, float, float z);
const Vector3D& location_as_vec3d(const Location &loc);


// Rotation
unique_ptr<Rotation> rotation_from_pitch_yaw_roll(float p, float y, float r);
float rotation_roll(const Rotation &rot);
float rotation_pitch(const Rotation &rot);
float rotation_yaw(const Rotation &rot);

// Transform
unique_ptr<Transform> transform_new(const Location &loc, const Rotation &rot);
const Location &transform_get_location(const Transform &trans);
const Rotation &transform_get_rotation(const Transform &trans);
