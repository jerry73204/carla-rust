#include <memory>
#include "carla/Time.h"
#include "carla_rust.hpp"

using carla::time_duration;

// Client

unique_ptr<Client> client_new(const string &host, uint16_t port) {
    return make_unique<Client>(host, port);
}

rust::String client_get_client_version(const Client &client) {
    return client.GetClientVersion();
}

rust::String client_get_server_version(const Client &client) {
    return client.GetServerVersion();
}

unique_ptr<World> client_load_world(const Client &client, const string &map_name) {
    World world = client.LoadWorld(map_name);
    return unique_ptr<World>(new World(world));
}

unique_ptr<World> client_get_world(const Client &client) {
    World world = client.GetWorld();
    return unique_ptr<World>(new World(world));
}

size_t client_get_timeout_millis(Client &client) {
    return client.GetTimeout().milliseconds();
}

void client_set_timeout_millis(Client &client, size_t millis) {
    auto dur = time_duration::milliseconds(millis);
    client.SetTimeout(dur);
}



// World

unique_ptr<SharedMap> world_get_map(const World &world) {
    auto map = world.GetMap();
    return unique_ptr<SharedMap>(new SharedMap(map));
}

unique_ptr<SharedBlueprintLibrary>
world_get_blueprint_library(const World &world) {
    auto bp = world.GetBlueprintLibrary();
    return unique_ptr<SharedBlueprintLibrary>(new SharedBlueprintLibrary(bp));
}

unique_ptr<SharedActor>
world_get_spectator(const World &world) {
    auto actor = world.GetSpectator();
    return unique_ptr<SharedActor>(new SharedActor(actor));
}

// BlueprintLibrary

unique_ptr<SharedBlueprintLibrary>
bp_filter(const SharedBlueprintLibrary &bp, const string &wildcard_pattern) {
    auto new_bp = bp->Filter(wildcard_pattern);
    return unique_ptr<SharedBlueprintLibrary>(new SharedBlueprintLibrary(new_bp));
}
