#pragma once

#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/road/RoadTypes.h"
#include "carla/road/Lane.h"
#include "carla_rust/client/waypoint_list.hpp"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::client::Waypoint;
        using carla::road::RoadId;
        using carla::road::SectionId;
        using carla::road::LaneId;
        using carla::road::JuncId;
        using carla::road::Lane;
        using carla_rust::client::FfiWaypointList;

        class FfiWaypoint {
        public:
            FfiWaypoint(SharedPtr<Waypoint> &&base)
                : inner_(std::move(base))
            {}

            uint64_t GetId() const {
                return inner_->GetId();
            }

            RoadId GetRoadId() const {
                return inner_->GetRoadId();
            }

            SectionId GetSectionId() const {
                return inner_->GetRoadId();
            }

            LaneId GetLaneId() const {
                return inner_->GetLaneId();
            }

            double GetDistance() const {
                return inner_->GetDistance();
            }

            const FfiTransform GetTransform() const {
                auto trans = inner_->GetTransform();
                return FfiTransform(std::move(trans));
            }

            JuncId GetJunctionId() const {
                return inner_->GetJunctionId();
            }

            bool IsJunction() const {
                return inner_->IsJunction();
            }

            // SharedPtr<Junction> GetJunction() const {
            // }

            double GetLaneWidth() const {
                return inner_->GetLaneWidth();
            }

            Lane::LaneType GetType() const {
                return inner_->GetType();
            }

            FfiWaypointList GetNext(double distance) const {
                auto orig = inner_->GetNext(distance);
                return FfiWaypointList(std::move(orig));
            }

            FfiWaypointList GetPrevious(double distance) const {
                auto orig = inner_->GetPrevious(distance);
                return FfiWaypointList(std::move(orig));
            }


            FfiWaypointList GetNextUntilLaneEnd(double distance) const {
                auto orig = inner_->GetPrevious(distance);
                return FfiWaypointList(std::move(orig));
            }

            FfiWaypointList GetPreviousUntilLaneStart(double distance) const {
                auto orig = inner_->GetPreviousUntilLaneStart(distance);
                return FfiWaypointList(std::move(orig));
            }

            std::shared_ptr<FfiWaypoint> GetRight() const {
                auto ptr = inner_->GetRight();
                return std::make_shared<FfiWaypoint>(std::move(ptr));
            }

            std::shared_ptr<FfiWaypoint> GetLeft() const {
                auto ptr = inner_->GetLeft();
                return std::make_shared<FfiWaypoint>(std::move(ptr));
            }

            // boost::optional<LaneMarking> GetRightLaneMarking() const;

            // boost::optional<LaneMarking> GetLeftLaneMarking() const;

            // LaneMarking::LaneChange GetLaneChange() const;

            // std::vector<SharedPtr<Landmark>> GetAllLandmarksInDistance(double distance, bool stop_at_junction = false) const;

            // std::vector<SharedPtr<Landmark>> GetLandmarksOfTypeInDistance(double distance, std::string filter_type, bool stop_at_junction = false) const;

            const SharedPtr<Waypoint>& inner() const {
                return inner_;
            }

        private:
            SharedPtr<Waypoint> inner_;
        };

    }
} // namespace carla_rust
