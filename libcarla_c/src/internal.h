#ifndef CARLA_C_INTERNAL_H
#define CARLA_C_INTERNAL_H

#include "carla_c/types.h"
#include "carla/client/Client.h"
#include "carla/client/World.h"
#include "carla/client/Map.h"
#include "carla/client/Actor.h"
#include "carla/client/ActorList.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/ActorBlueprint.h"
#include <memory>
#include <vector>

// Internal structure definitions shared across all C wrapper files

// Client structure
struct carla_client {
    std::unique_ptr<carla::client::Client> client;
    std::shared_ptr<carla::client::World> current_world;
};

// World structure
struct carla_world {
    std::shared_ptr<carla::client::World> world;
};

// Map structure
struct carla_map {
    std::shared_ptr<carla::client::Map> map;
};

// Actor structure
struct carla_actor {
    std::shared_ptr<carla::client::Actor> actor;
};

// Actor list structure
struct carla_actor_list {
    carla::SharedPtr<carla::client::ActorList> list;
};

// Blueprint library structure
struct carla_blueprint_library {
    std::shared_ptr<carla::client::BlueprintLibrary> library;
};

// Actor blueprint structure (defined per-file as needed)

// Transform list structure
struct carla_transform_list {
    std::vector<carla::geom::Transform> transforms;
};

// String list structure
struct carla_string_list {
    char** strings;
    size_t count;
};

#endif // CARLA_C_INTERNAL_H