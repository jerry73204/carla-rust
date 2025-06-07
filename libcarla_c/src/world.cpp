#include "carla_c/world.h"
#include "carla_c/map.h"
#include "carla_c/actor.h"
#include "internal.h"
#include "carla/rpc/WeatherParameters.h"
#include <cstring>

// Note: carla_actor_blueprint definition is customized for this file
struct carla_actor_blueprint {
    const carla::client::ActorBlueprint* blueprint;
    std::shared_ptr<carla::client::BlueprintLibrary> library; // Keep library alive
};

// Helper functions
static carla_transform_t to_c_transform(const carla::geom::Transform& t) {
    carla_transform_t result;
    result.location.x = t.location.x;
    result.location.y = t.location.y;
    result.location.z = t.location.z;
    result.rotation.pitch = t.rotation.pitch;
    result.rotation.yaw = t.rotation.yaw;
    result.rotation.roll = t.rotation.roll;
    return result;
}

static carla::geom::Transform from_c_transform(const carla_transform_t* t) {
    carla::geom::Transform result;
    result.location.x = t->location.x;
    result.location.y = t->location.y;
    result.location.z = t->location.z;
    result.rotation.pitch = t->rotation.pitch;
    result.rotation.yaw = t->rotation.yaw;
    result.rotation.roll = t->rotation.roll;
    return result;
}

// World lifecycle
void carla_world_free(carla_world_t* world) {
    delete world;
}

// World properties
uint64_t carla_world_get_id(const carla_world_t* world) {
    if (world && world->world) {
        return world->world->GetId();
    }
    return 0;
}

carla_map_t* carla_world_get_map(const carla_world_t* world) {
    if (world && world->world) {
        try {
            auto map = world->world->GetMap();
            return new carla_map{map};
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_blueprint_library_t* carla_world_get_blueprint_library(const carla_world_t* world) {
    if (world && world->world) {
        try {
            auto library = world->world->GetBlueprintLibrary();
            return new carla_blueprint_library{library};
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_actor_t* carla_world_get_spectator(carla_world_t* world) {
    if (world && world->world) {
        try {
            auto spectator = world->world->GetSpectator();
            return new carla_actor{spectator};
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

// Simulation control
uint64_t carla_world_tick(carla_world_t* world, uint64_t timeout_ms) {
    if (world && world->world) {
        try {
            auto result = world->world->Tick(std::chrono::milliseconds(timeout_ms));
            return result;
        } catch (...) {
            return 0;
        }
    }
    return 0;
}

void carla_world_wait_for_tick(carla_world_t* world, uint64_t timeout_ms) {
    if (world && world->world) {
        try {
            world->world->WaitForTick(std::chrono::milliseconds(timeout_ms));
        } catch (...) {
            // Silently ignore errors
        }
    }
}

// Actor management
carla_actor_list_t* carla_world_get_actors(const carla_world_t* world) {
    if (world && world->world) {
        try {
            auto actors = world->world->GetActors();
            return new carla_actor_list{actors};
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_actor_list_t* carla_world_get_actors_by_id(const carla_world_t* world, 
                                                  const uint32_t* ids, size_t count) {
    if (world && world->world && ids && count > 0) {
        try {
            std::vector<carla::ActorId> actor_ids(ids, ids + count);
            auto actors = world->world->GetActors(actor_ids);
            return new carla_actor_list{actors};
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_actor_t* carla_world_get_actor(const carla_world_t* world, uint32_t id) {
    if (world && world->world) {
        try {
            auto actor = world->world->GetActor(id);
            if (actor) {
                return new carla_actor{actor};
            }
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

// Actor spawning
carla_spawn_result_t carla_world_spawn_actor(carla_world_t* world,
                                             const carla_actor_blueprint_t* blueprint,
                                             const carla_transform_t* transform,
                                             carla_actor_t* attach_to) {
    carla_spawn_result_t result = {nullptr, CARLA_ERROR_UNKNOWN};
    
    if (world && world->world && blueprint && blueprint->blueprint && transform) {
        try {
            auto cpp_transform = from_c_transform(transform);
            carla::client::Actor* parent = attach_to ? attach_to->actor.get() : nullptr;
            
            auto actor = world->world->SpawnActor(*blueprint->blueprint, cpp_transform, parent);
            if (actor) {
                result.actor = new carla_actor{actor};
                result.error = CARLA_ERROR_NONE;
            }
        } catch (const std::exception&) {
            result.error = CARLA_ERROR_UNKNOWN;
        }
    } else {
        result.error = CARLA_ERROR_INVALID_ARGUMENT;
    }
    
    return result;
}

carla_spawn_result_t carla_world_try_spawn_actor(carla_world_t* world,
                                                 const carla_actor_blueprint_t* blueprint,
                                                 const carla_transform_t* transform,
                                                 carla_actor_t* attach_to) {
    carla_spawn_result_t result = {nullptr, CARLA_ERROR_UNKNOWN};
    
    if (world && world->world && blueprint && blueprint->blueprint && transform) {
        try {
            auto cpp_transform = from_c_transform(transform);
            carla::client::Actor* parent = attach_to ? attach_to->actor.get() : nullptr;
            
            auto actor = world->world->TrySpawnActor(*blueprint->blueprint, cpp_transform, parent);
            if (actor) {
                result.actor = new carla_actor{actor};
                result.error = CARLA_ERROR_NONE;
            } else {
                result.error = CARLA_ERROR_UNKNOWN;
            }
        } catch (const std::exception&) {
            result.error = CARLA_ERROR_UNKNOWN;
        }
    } else {
        result.error = CARLA_ERROR_INVALID_ARGUMENT;
    }
    
    return result;
}

// Transform queries
carla_transform_list_t* carla_world_get_random_location_from_navigation(carla_world_t* world) {
    if (world && world->world) {
        try {
            auto transform = world->world->GetRandomLocationFromNavigation();
            if (transform.has_value()) {
                auto* list = new carla_transform_list();
                list->transforms.push_back(transform.value());
                return list;
            }
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_transform_t carla_world_get_spectator_transform(const carla_world_t* world) {
    carla_transform_t result = {{0, 0, 0}, {0, 0, 0}};
    if (world && world->world) {
        try {
            auto spectator = world->world->GetSpectator();
            if (spectator) {
                auto transform = spectator->GetTransform();
                result = to_c_transform(transform);
            }
        } catch (...) {
            // Return default transform
        }
    }
    return result;
}

void carla_world_set_spectator_transform(carla_world_t* world, const carla_transform_t* transform) {
    if (world && world->world && transform) {
        try {
            auto spectator = world->world->GetSpectator();
            if (spectator) {
                auto cpp_transform = from_c_transform(transform);
                spectator->SetTransform(cpp_transform);
            }
        } catch (...) {
            // Silently ignore errors
        }
    }
}

// Weather control
carla_weather_parameters_t carla_world_get_weather(const carla_world_t* world) {
    carla_weather_parameters_t result = {0};
    if (world && world->world) {
        try {
            auto weather = world->world->GetWeather();
            result.cloudiness = weather.cloudiness;
            result.precipitation = weather.precipitation;
            result.precipitation_deposits = weather.precipitation_deposits;
            result.wind_intensity = weather.wind_intensity;
            result.sun_azimuth_angle = weather.sun_azimuth_angle;
            result.sun_altitude_angle = weather.sun_altitude_angle;
            result.fog_density = weather.fog_density;
            result.fog_distance = weather.fog_distance;
            result.wetness = weather.wetness;
            result.fog_falloff = weather.fog_falloff;
            result.scattering_intensity = weather.scattering_intensity;
            result.mie_scattering_scale = weather.mie_scattering_scale;
            result.rayleigh_scattering_scale = weather.rayleigh_scattering_scale;
        } catch (...) {
            // Return default weather
        }
    }
    return result;
}

void carla_world_set_weather(carla_world_t* world, const carla_weather_parameters_t* weather) {
    if (world && world->world && weather) {
        try {
            carla::rpc::WeatherParameters params;
            params.cloudiness = weather->cloudiness;
            params.precipitation = weather->precipitation;
            params.precipitation_deposits = weather->precipitation_deposits;
            params.wind_intensity = weather->wind_intensity;
            params.sun_azimuth_angle = weather->sun_azimuth_angle;
            params.sun_altitude_angle = weather->sun_altitude_angle;
            params.fog_density = weather->fog_density;
            params.fog_distance = weather->fog_distance;
            params.wetness = weather->wetness;
            params.fog_falloff = weather->fog_falloff;
            params.scattering_intensity = weather->scattering_intensity;
            params.mie_scattering_scale = weather->mie_scattering_scale;
            params.rayleigh_scattering_scale = weather->rayleigh_scattering_scale;
            
            world->world->SetWeather(params);
        } catch (...) {
            // Silently ignore errors
        }
    }
}

// Snapshot
carla_world_snapshot_t carla_world_get_snapshot(const carla_world_t* world) {
    carla_world_snapshot_t result = {0, 0, 0.0};
    if (world && world->world) {
        try {
            auto snapshot = world->world->GetSnapshot();
            auto timestamp = snapshot.GetTimestamp();
            result.id = snapshot.GetId();
            result.frame = timestamp.frame;
            result.timestamp = timestamp.elapsed_seconds;
        } catch (...) {
            // Return default snapshot
        }
    }
    return result;
}

// Actor list operations
size_t carla_actor_list_size(const carla_actor_list_t* list) {
    if (list && list->list) {
        return list->list->size();
    }
    return 0;
}

carla_actor_t* carla_actor_list_get(carla_actor_list_t* list, size_t index) {
    if (list && list->list && index < list->list->size()) {
        try {
            auto actor = list->list->at(index);
            if (actor) {
                return new carla_actor{actor};
            }
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_actor_t* carla_actor_list_find(carla_actor_list_t* list, uint32_t id) {
    if (list && list->list) {
        try {
            auto actor = list->list->Find(id);
            if (actor) {
                return new carla_actor{actor};
            }
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

void carla_actor_list_free(carla_actor_list_t* list) {
    delete list;
}

// Transform list operations
size_t carla_transform_list_size(const carla_transform_list_t* list) {
    return list ? list->transforms.size() : 0;
}

carla_transform_t carla_transform_list_get(const carla_transform_list_t* list, size_t index) {
    carla_transform_t result = {{0, 0, 0}, {0, 0, 0}};
    if (list && index < list->transforms.size()) {
        result = to_c_transform(list->transforms[index]);
    }
    return result;
}

void carla_transform_list_free(carla_transform_list_t* list) {
    delete list;
}