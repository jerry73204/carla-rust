#pragma once

#include "carla/Memory.h"
#include "carla/client/Map.h"
#include "carla/geom/GeoLocation.h"
#include "carla_rust/geom.hpp"
#include "carla_rust/client/landmark_list.hpp"
#include "carla_rust/client/transform_list.hpp"
#include "carla_rust/client/waypoint_list.hpp"
#include "carla_rust/client/waypoint_pair.hpp"
#include "carla_rust/client/result.hpp"
#include "junction.hpp"
#include "waypoint_list.hpp"
#include <memory>

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::Map;
using carla_rust::client::FfiLandmarkList;
using carla_rust::client::FfiTransformList;
using carla_rust::client::FfiWaypointList;
using carla_rust::client::FfiWaypointPair;
using carla_rust::geom::FfiGeoLocation;
using carla_rust::geom::FfiLocation;

// Map
class FfiMap {
public:
    FfiMap(SharedPtr<Map>&& base) noexcept : inner_(std::move(base)) {}

    FfiMap(FfiMap&& from) = default;

    const std::string& GetName() const noexcept { return inner_->GetName(); }

    const std::string& GetOpenDrive() const noexcept { return inner_->GetOpenDrive(); }

    FfiTransformList GetRecommendedSpawnPoints(FfiError& error) const {
        return ffi_call(error, FfiTransformList(), [&]() {
            auto orig = inner_->GetRecommendedSpawnPoints();
            return FfiTransformList(std::move(orig));
        });
    }

    std::shared_ptr<FfiWaypoint> GetWaypoint(const FfiLocation& location, bool project_to_road,
                                             int32_t lane_type, FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiWaypoint>(), [&]() {
            auto ptr = inner_->GetWaypoint(location.as_native(), project_to_road, lane_type);
            if (ptr == nullptr) {
                return std::shared_ptr<FfiWaypoint>();
            } else {
                return std::make_shared<FfiWaypoint>(std::move(ptr));
            }
        });
    }

    std::shared_ptr<FfiWaypoint> GetWaypointXODR(RoadId road_id, LaneId lane_id, float s,
                                                 FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiWaypoint>(nullptr), [&]() {
            auto ptr = inner_->GetWaypointXODR(road_id, lane_id, s);
            if (ptr == nullptr) {
                return std::shared_ptr<FfiWaypoint>(nullptr);
            } else {
                return std::make_shared<FfiWaypoint>(std::move(ptr));
            }
        });
    }

    std::vector<FfiWaypointPair> GetTopology(FfiError& error) const {
        return ffi_call(error, std::vector<FfiWaypointPair>(), [&]() {
            auto topology = inner_->GetTopology();
            std::vector<FfiWaypointPair> result;
            result.reserve(topology.size());
            for (const auto& pair : topology) {
                result.push_back(FfiWaypointPair(pair));
            }
            return result;
        });
    }

    std::unique_ptr<FfiWaypointList> GenerateWaypoints(double distance, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWaypointList>(nullptr), [&]() {
            auto waypoints = inner_->GenerateWaypoints(distance);
            return std::make_unique<FfiWaypointList>(std::move(waypoints));
        });
    }

    // std::vector<road::element::LaneMarking> CalculateCrossedLanes(
    // const geom::Location &origin,

    FfiGeoLocation GetGeoReference(FfiError& error) const {
        return ffi_call(error, FfiGeoLocation(),
                        [&]() { return FfiGeoLocation(inner_->GetGeoReference()); });
    }

    std::vector<FfiLocation> GetAllCrosswalkZones(FfiError& error) const {
        return ffi_call(error, std::vector<FfiLocation>(), [&]() {
            auto zones = inner_->GetAllCrosswalkZones();
            std::vector<FfiLocation> result;
            result.reserve(zones.size());
            for (auto& loc : zones) {
                result.push_back(FfiLocation(std::move(loc)));
            }
            return result;
        });
    }

    void CookInMemoryMap(std::string path, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->CookInMemoryMap(path); });
    }

    std::shared_ptr<FfiJunction> GetJunction(const FfiWaypoint& waypoint, FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiJunction>(nullptr), [&]() {
            const auto& wp = waypoint.inner();
            auto junction = inner_->GetJunction(*wp.get());
            if (junction) {
                return std::make_shared<FfiJunction>(std::move(junction));
            } else {
                return std::shared_ptr<FfiJunction>(nullptr);
            }
        });
    }

    // std::vector<std::pair<SharedPtr<Waypoint>, SharedPtr<Waypoint>>>
    // GetJunctionWaypoints( road::JuncId id, road::Lane::LaneType type) const;

    std::unique_ptr<FfiLandmarkList> GetAllLandmarks(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLandmarkList>(nullptr), [&]() {
            auto orig = inner_->GetAllLandmarks();
            return std::make_unique<FfiLandmarkList>(std::move(orig));
        });
    }

    std::unique_ptr<FfiLandmarkList> GetLandmarksFromId(std::string id, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLandmarkList>(nullptr), [&]() {
            auto orig = inner_->GetLandmarksFromId(std::move(id));
            return std::make_unique<FfiLandmarkList>(std::move(orig));
        });
    }

    std::unique_ptr<FfiLandmarkList> GetAllLandmarksOfType(std::string type,
                                                           FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLandmarkList>(nullptr), [&]() {
            auto orig = inner_->GetAllLandmarksOfType(std::move(type));
            return std::make_unique<FfiLandmarkList>(std::move(orig));
        });
    }

    std::unique_ptr<FfiLandmarkList> GetLandmarkGroup(const FfiLandmark& landmark,
                                                      FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLandmarkList>(nullptr), [&]() {
            const SharedPtr<Landmark>& shared = landmark.inner();
            const Landmark& ptr = *(shared.get());
            auto orig = inner_->GetLandmarkGroup(ptr);
            return std::make_unique<FfiLandmarkList>(std::move(orig));
        });
    }

private:
    SharedPtr<Map> inner_;
};
}  // namespace client
}  // namespace carla_rust
