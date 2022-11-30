// #include <memory>
// #include "carla/Time.h"
#include "carla_rust.hpp"

// using carla::time_duration;

// CarlaClient::CarlaClient(const string &host, uint16_t port,
//                          size_t worker_threads)
//     : carla::client::Client(host, port, worker_threads)
// {}

// // Client
// unique_ptr<Client> client_new(const string &host, uint16_t port) {
//     return make_unique<Client>(host, port);
// }

// rust::String client_get_client_version(const Client &client) {
//     return client.GetClientVersion();
// }

// rust::String client_get_server_version(const Client &client) {
//     return client.GetServerVersion();
// }

// unique_ptr<World> client_load_world(const Client &client, const string &map_name) {
//     World world = client.LoadWorld(map_name);
//     return make_unique<World>(world);
// }

// unique_ptr<World> client_get_world(const Client &client) {
//     World world = client.GetWorld();
//     return make_unique<World>(world);
// }

// size_t client_get_timeout_millis(Client &client) {
//     return client.GetTimeout().milliseconds();
// }

// void client_set_timeout_millis(Client &client, size_t millis) {
//     auto dur = time_duration::milliseconds(millis);
//     client.SetTimeout(dur);
// }

// // World
// unique_ptr<SharedMap> world_get_map(const World &world) {
//     auto map = world.GetMap();
//     return make_unique<SharedMap>(map);
// }

// unique_ptr<SharedBlueprintLibrary>
// world_get_blueprint_library(const World &world) {
//     auto bp = world.GetBlueprintLibrary();
//     return make_unique<SharedBlueprintLibrary>(bp);
// }

// unique_ptr<SharedActor>
// world_get_spectator(const World &world) {
//     auto actor = world.GetSpectator();
//     return make_unique<SharedActor>(actor);
// }

// unique_ptr<SharedActor> world_try_spawn_actor(World &world,
//                                               const ActorBlueprint &blueprint,
//                                               const Transform &transform,
//                                               const SharedActor *parent) {
//     Actor *parent_ptr = nullptr;
//     if (parent != nullptr) {
//         parent_ptr = parent->get();
//     }

//     auto actor = world.TrySpawnActor(blueprint, transform, parent_ptr);
//     return make_unique<SharedActor>(actor);
// }

// // BlueprintLibrary
// unique_ptr<SharedBlueprintLibrary>
// bp_filter(const SharedBlueprintLibrary &bp, const string &wildcard_pattern) {
//     auto new_bp = bp->Filter(wildcard_pattern);
//     return make_unique<SharedBlueprintLibrary>(new_bp);
// }

// const BlueprintLibrary *bp_to_raw(const SharedBlueprintLibrary &bp) {
//     return bp.get();
// }


// // Actor
// unique_ptr<Location> actor_get_location(const SharedActor &actor) {
//     return make_unique<Location>(actor->GetLocation());
// }

// unique_ptr<Transform> actor_get_transform(const SharedActor &actor) {
//     return make_unique<Transform>(actor->GetTransform());
// }

// unique_ptr<Vector3D> actor_get_velocity(const SharedActor &actor) {
//     return make_unique<Vector3D>(actor->GetVelocity());
// }

// unique_ptr<Vector3D> actor_get_acceleration(const SharedActor &actor) {
//     return make_unique<Vector3D>(actor->GetAcceleration());
// }

// unique_ptr<Vector3D> actor_get_angular_velocity(const SharedActor &actor) {
//     return make_unique<Vector3D>(actor->GetAngularVelocity());
// }

// // ActorBlueprint
// unique_ptr<ActorBlueprint> actor_bp_copy(const ActorBlueprint &from) {
//     return make_unique<ActorBlueprint>(from);
// }
// void actor_bp_set_attribute(ActorBlueprint &actor_bp, const string &id,
//                             const string &value) {
//     actor_bp.SetAttribute(id, value);
// }

// // Vector2D
// unique_ptr<Vector2D> vec2d_new(float x, float y) {
//     return make_unique<Vector2D>(x, y);
// }

// float vec2d_x(const Vector2D &vec2d) {
//     return vec2d.x;
// }

// float vec2d_y(const Vector2D &vec2d) {
//     return vec2d.y;
// }


// // Vector3D
// unique_ptr<Vector3D> vec3d_new(float x, float y, float z) {
//     return make_unique<Vector3D>(x, y, z);
// }

// float vec3d_x(const Vector3D &vec3d) {
//     return vec3d.x;
// }

// float vec3d_y(const Vector3D &vec3d) {
//     return vec3d.y;
// }

// float vec3d_z(const Vector3D &vec3d) {
//     return vec3d.z;
// }

// // Location
// unique_ptr<Location> location_from_xyz(float x, float y, float z) {
//     return make_unique<Location>(x, y, z);
// }

// const Vector3D& location_as_vec3d(const Location &loc) {
//     return static_cast<const Vector3D&>(loc);
// }

// // Rotation
// unique_ptr<Rotation> rotation_from_pitch_yaw_roll(float p, float y, float r) {
//     return make_unique<Rotation>(p, y, r);
// }

// float rotation_roll(const Rotation &rot) {
//     return rot.roll;
// }

// float rotation_pitch(const Rotation &rot) {
//     return rot.pitch;
// }

// float rotation_yaw(const Rotation &rot) {
//     return rot.yaw;
// }

// // Transform
// unique_ptr<Transform> transform_new(const Location &loc, const Rotation &rot) {
//     return make_unique<Transform>(loc, rot);
// }

// const Location &transform_get_location(const Transform &trans) {
//     return trans.location;
// }

// const Rotation &transform_get_rotation(const Transform &trans) {
//     return trans.rotation;
// }
