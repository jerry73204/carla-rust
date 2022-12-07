#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/client/Map.h"
#include "carla_rust/client/transform_list.hpp"
#include "carla_rust/client/landmark_list.hpp"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::client::Map;
        using carla_rust::client::FfiTransformList;
        using carla_rust::client::FfiLandmarkList;

        // Map
        class FfiMap {
        public:
            FfiMap(SharedPtr<Map> &&base)
                : inner_(std::move(base))
            {}

            FfiMap(FfiMap &&from) = default;

            const std::string &GetName() const {
                return inner_->GetName();
            }

            const std::string &GetOpenDrive() const {
                return inner_->GetOpenDrive();
            }

            FfiTransformList GetRecommendedSpawnPoints() const {
                auto orig = inner_->GetRecommendedSpawnPoints();
                return FfiTransformList(std::move(orig));
            }

            std::shared_ptr<FfiWaypoint> GetWaypoint(const FfiLocation &location,
                                            bool project_to_road = true,
                                            int32_t lane_type = static_cast<uint32_t>(Lane::LaneType::Driving)) const
            {
                auto ptr = inner_->GetWaypoint(location.as_native(), project_to_road, lane_type);
                if (ptr == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiWaypoint>(std::move(ptr));
                }
            }

            std::shared_ptr<FfiWaypoint> GetWaypointXODR(RoadId road_id,
                                                         LaneId lane_id,
                                                         float s) const
            {
                auto ptr = inner_->GetWaypointXODR(road_id, lane_id, s);
                if (ptr == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiWaypoint>(std::move(ptr));
                }
            }

            FfiLandmarkList GetAllLandmarks() const {
                auto orig = inner_->GetAllLandmarks();
                return FfiLandmarkList(std::move(orig));
            }

            FfiLandmarkList GetLandmarksFromId(std::string id) const {
                auto orig = inner_->GetLandmarksFromId(id);
                return FfiLandmarkList(std::move(orig));
            }

            FfiLandmarkList GetAllLandmarksOfType(std::string type) const {
                auto orig = inner_->GetAllLandmarksOfType(type);
                return FfiLandmarkList(std::move(orig));
            }

            FfiLandmarkList GetLandmarkGroup(const FfiLandmark &landmark) const {
                const SharedPtr<Landmark>& shared = landmark.inner();
                const Landmark& ptr = *(shared.get());
                auto orig = inner_->GetLandmarkGroup(ptr);
                return FfiLandmarkList(std::move(orig));
            }

        private:
            SharedPtr<Map> inner_;
        };
    }
} // namespace carla_rust
