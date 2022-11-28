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

using namespace std;

using carla::time_duration;
using carla::client::Client;
using carla::client::Map;
using carla::client::World;
using carla::client::BlueprintLibrary;
using carla::client::Actor;
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

// BlueprintLibrary
unique_ptr<SharedBlueprintLibrary> bp_filter(const SharedBlueprintLibrary &bp, const string &wildcard_pattern);

// Actor
