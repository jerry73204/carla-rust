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
#include <carla/client/World.h>
#include <carla/geom/Transform.h>

// Forward declare our simple types from the generated header
struct SimpleLocation;
struct SimpleRotation;
struct SimpleTransform;

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

} // namespace client
} // namespace carla
