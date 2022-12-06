#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/client/Map.h"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::client::Map;

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

            std::vector<FfiTransform> GetRecommendedSpawnPoints() const {
                auto orig = inner_->GetRecommendedSpawnPoints();
                std::vector<FfiTransform> new_;

                for (auto&& trans: orig) {
                    new_.push_back(FfiTransform(std::move(trans)));
                }

                return new_;
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

            // std::vector<FfiLandmark> GetAllLandmarks() const {
            //     auto orig = inner_->GetAllLandmarks();
            //     std::vector<FfiLandmark> new_;

            //     for (auto&& ptr: orig) {
            //         new_.push_back(FfiLandmark(std::move(ptr)));
            //     }

            //     return new_;
            // }

            // std::vector<std::shared_ptr<FfiLandmark>> GetLandmarksFromId(std::string id) const;

            // std::vector<std::shared_ptr<FfiLandmark>> GetAllLandmarksOfType(std::string type) const;

            // std::vector<std::shared_ptr<FfiLandmark>> GetLandmarkGroup(const Landmark &landmark) const;


        private:
            SharedPtr<Map> inner_;
        };
    }
} // namespace carla_rust
