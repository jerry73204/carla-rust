#include "carla_c/map.h"
#include "internal.h"
#include "carla/client/Waypoint.h"
#include "carla/road/Lane.h"
#include <vector>
#include <cstring>
#include <string>

// Additional structures specific to map functionality
struct carla_waypoint {
    std::shared_ptr<carla::client::Waypoint> waypoint;
};

struct carla_waypoint_list {
    std::vector<std::shared_ptr<carla::client::Waypoint>> waypoints;
};


// Helper functions
static carla::geom::Location from_c_location(const carla_vector3d_t* loc) {
    return carla::geom::Location(loc->x, loc->y, loc->z);
}

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

// Map lifecycle
void carla_map_free(carla_map_t* map) {
    delete map;
}

// Map properties
const char* carla_map_get_name(const carla_map_t* map) {
    if (map && map->map) {
        static thread_local std::string name_buffer;
        name_buffer = map->map->GetName();
        return name_buffer.c_str();
    }
    return nullptr;
}

double carla_map_get_lane_width(const carla_map_t* map) {
    // Note: LibCarla doesn't have a direct GetLaneWidth method on Map
    // This would need to be implemented differently or removed
    return 3.5; // Default lane width
}

// Waypoint queries
carla_waypoint_t* carla_map_get_waypoint(const carla_map_t* map, 
                                         const carla_vector3d_t* location,
                                         bool project_to_road,
                                         int32_t lane_type) {
    if (map && map->map && location) {
        try {
            auto loc = from_c_location(location);
            auto waypoint = map->map->GetWaypoint(loc, project_to_road, lane_type);
            if (waypoint) {
                return new carla_waypoint{waypoint};
            }
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_waypoint_t* carla_map_get_waypoint_xodr(const carla_map_t* map,
                                              const char* road_id,
                                              int32_t lane_id, 
                                              float s) {
    if (map && map->map && road_id) {
        try {
            // Convert string road_id to unsigned int
            unsigned int road_id_uint = static_cast<unsigned int>(std::stoul(road_id));
            auto waypoint = map->map->GetWaypointXODR(road_id_uint, lane_id, s);
            if (waypoint) {
                return new carla_waypoint{waypoint};
            }
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_waypoint_list_t* carla_map_get_topology(const carla_map_t* map) {
    if (map && map->map) {
        try {
            auto topology = map->map->GetTopology();
            auto* list = new carla_waypoint_list();
            
            // Note: Topology returns pairs of waypoints, we'll flatten them
            for (const auto& pair : topology) {
                list->waypoints.push_back(pair.first);
                list->waypoints.push_back(pair.second);
            }
            return list;
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_waypoint_list_t* carla_map_generate_waypoints(const carla_map_t* map, double distance) {
    if (map && map->map) {
        try {
            auto waypoints = map->map->GenerateWaypoints(distance);
            auto* list = new carla_waypoint_list();
            list->waypoints = std::move(waypoints);
            return list;
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

// Spawn points
carla_transform_list_t* carla_map_get_spawn_points(const carla_map_t* map) {
    if (map && map->map) {
        try {
            auto spawn_points = map->map->GetRecommendedSpawnPoints();
            auto* list = new carla_transform_list();
            list->transforms = std::move(spawn_points);
            return list;
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

// Waypoint operations
void carla_waypoint_free(carla_waypoint_t* waypoint) {
    delete waypoint;
}

uint64_t carla_waypoint_get_id(const carla_waypoint_t* waypoint) {
    if (waypoint && waypoint->waypoint) {
        return waypoint->waypoint->GetId();
    }
    return 0;
}

uint32_t carla_waypoint_get_road_id(const carla_waypoint_t* waypoint) {
    if (waypoint && waypoint->waypoint) {
        return waypoint->waypoint->GetRoadId();
    }
    return 0;
}

uint32_t carla_waypoint_get_section_id(const carla_waypoint_t* waypoint) {
    if (waypoint && waypoint->waypoint) {
        return waypoint->waypoint->GetSectionId();
    }
    return 0;
}

int32_t carla_waypoint_get_lane_id(const carla_waypoint_t* waypoint) {
    if (waypoint && waypoint->waypoint) {
        return waypoint->waypoint->GetLaneId();
    }
    return 0;
}

double carla_waypoint_get_s(const carla_waypoint_t* waypoint) {
    if (waypoint && waypoint->waypoint) {
        return waypoint->waypoint->GetDistance();
    }
    return 0.0;
}

bool carla_waypoint_is_junction(const carla_waypoint_t* waypoint) {
    if (waypoint && waypoint->waypoint) {
        return waypoint->waypoint->IsJunction();
    }
    return false;
}

double carla_waypoint_get_lane_width(const carla_waypoint_t* waypoint) {
    if (waypoint && waypoint->waypoint) {
        return waypoint->waypoint->GetLaneWidth();
    }
    return 0.0;
}

int32_t carla_waypoint_get_lane_change(const carla_waypoint_t* waypoint) {
    if (waypoint && waypoint->waypoint) {
        return static_cast<int32_t>(waypoint->waypoint->GetLaneChange());
    }
    return 0;
}

int32_t carla_waypoint_get_lane_type(const carla_waypoint_t* waypoint) {
    if (waypoint && waypoint->waypoint) {
        return static_cast<int32_t>(waypoint->waypoint->GetType());
    }
    return 0;
}

carla_transform_t carla_waypoint_get_transform(const carla_waypoint_t* waypoint) {
    carla_transform_t result = {{0, 0, 0}, {0, 0, 0}};
    if (waypoint && waypoint->waypoint) {
        auto transform = waypoint->waypoint->GetTransform();
        result = to_c_transform(transform);
    }
    return result;
}

// Waypoint navigation
carla_waypoint_t* carla_waypoint_get_next(const carla_waypoint_t* waypoint, double distance) {
    if (waypoint && waypoint->waypoint) {
        try {
            auto next_waypoints = waypoint->waypoint->GetNext(distance);
            if (!next_waypoints.empty()) {
                return new carla_waypoint{next_waypoints[0]};
            }
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_waypoint_list_t* carla_waypoint_get_next_list(const carla_waypoint_t* waypoint, double distance) {
    if (waypoint && waypoint->waypoint) {
        try {
            auto next_waypoints = waypoint->waypoint->GetNext(distance);
            auto* list = new carla_waypoint_list();
            list->waypoints = std::move(next_waypoints);
            return list;
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_waypoint_t* carla_waypoint_get_previous(const carla_waypoint_t* waypoint, double distance) {
    if (waypoint && waypoint->waypoint) {
        try {
            auto prev_waypoints = waypoint->waypoint->GetPrevious(distance);
            if (!prev_waypoints.empty()) {
                return new carla_waypoint{prev_waypoints[0]};
            }
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_waypoint_list_t* carla_waypoint_get_previous_list(const carla_waypoint_t* waypoint, double distance) {
    if (waypoint && waypoint->waypoint) {
        try {
            auto prev_waypoints = waypoint->waypoint->GetPrevious(distance);
            auto* list = new carla_waypoint_list();
            list->waypoints = std::move(prev_waypoints);
            return list;
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_waypoint_t* carla_waypoint_get_right(const carla_waypoint_t* waypoint) {
    if (waypoint && waypoint->waypoint) {
        try {
            auto right = waypoint->waypoint->GetRight();
            if (right) {
                return new carla_waypoint{right};
            }
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

carla_waypoint_t* carla_waypoint_get_left(const carla_waypoint_t* waypoint) {
    if (waypoint && waypoint->waypoint) {
        try {
            auto left = waypoint->waypoint->GetLeft();
            if (left) {
                return new carla_waypoint{left};
            }
        } catch (...) {
            return nullptr;
        }
    }
    return nullptr;
}

// Waypoint list operations  
size_t carla_waypoint_list_size(const carla_waypoint_list_t* list) {
    return list ? list->waypoints.size() : 0;
}

carla_waypoint_t* carla_waypoint_list_get(carla_waypoint_list_t* list, size_t index) {
    if (list && index < list->waypoints.size()) {
        return new carla_waypoint{list->waypoints[index]};
    }
    return nullptr;
}

void carla_waypoint_list_free(carla_waypoint_list_t* list) {
    delete list;
}