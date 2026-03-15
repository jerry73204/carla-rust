#pragma once

#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/road/RoadTypes.h"
#include "carla/road/Lane.h"
#include "carla/road/element/LaneMarking.h"
#include "carla_rust/client/waypoint_list.hpp"
#include "carla_rust/client/junction.hpp"
#include "carla_rust/client/landmark_list.hpp"
#include "carla_rust/client/result.hpp"
#include "carla_rust/road.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::Waypoint;
using carla::road::JuncId;
using carla::road::Lane;
using carla::road::LaneId;
using carla::road::RoadId;
using carla::road::SectionId;
using carla::road::element::LaneMarking;
using carla_rust::client::FfiJunction;
using carla_rust::client::FfiWaypointList;
using carla_rust::road::element::FfiLaneMarking;

class FfiWaypoint {
public:
    FfiWaypoint(SharedPtr<Waypoint>&& base) noexcept : inner_(std::move(base)) {}

    FfiWaypoint(const SharedPtr<Waypoint>& base) noexcept : inner_(base) {}

    uint64_t GetId() const noexcept { return inner_->GetId(); }

    RoadId GetRoadId() const noexcept { return inner_->GetRoadId(); }

    SectionId GetSectionId() const noexcept { return inner_->GetSectionId(); }

    LaneId GetLaneId() const noexcept { return inner_->GetLaneId(); }

    double GetDistance() const noexcept { return inner_->GetDistance(); }

    const FfiTransform GetTransform() const noexcept {
        auto trans = inner_->GetTransform();
        return FfiTransform(std::move(trans));
    }

    JuncId GetJunctionId() const noexcept { return inner_->GetJunctionId(); }

    bool IsJunction() const noexcept { return inner_->IsJunction(); }

    std::shared_ptr<FfiJunction> GetJunction(FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiJunction>(nullptr), [&]() {
            auto orig = inner_->GetJunction();
            if (orig) {
                return std::make_shared<FfiJunction>(std::move(orig));
            } else {
                return std::shared_ptr<FfiJunction>(nullptr);
            }
        });
    }

    double GetLaneWidth() const noexcept { return inner_->GetLaneWidth(); }

    Lane::LaneType GetType() const noexcept { return inner_->GetType(); }

    std::unique_ptr<FfiWaypointList> GetNext(double distance, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWaypointList>(nullptr), [&]() {
            auto orig = inner_->GetNext(distance);
            return std::make_unique<FfiWaypointList>(std::move(orig));
        });
    }

    std::unique_ptr<FfiWaypointList> GetPrevious(double distance, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWaypointList>(nullptr), [&]() {
            auto orig = inner_->GetPrevious(distance);
            return std::make_unique<FfiWaypointList>(std::move(orig));
        });
    }

    std::unique_ptr<FfiWaypointList> GetNextUntilLaneEnd(double distance, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWaypointList>(nullptr), [&]() {
            auto orig = inner_->GetNextUntilLaneEnd(distance);
            return std::make_unique<FfiWaypointList>(std::move(orig));
        });
    }

    std::unique_ptr<FfiWaypointList> GetPreviousUntilLaneStart(double distance,
                                                               FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWaypointList>(nullptr), [&]() {
            auto orig = inner_->GetPreviousUntilLaneStart(distance);
            return std::make_unique<FfiWaypointList>(std::move(orig));
        });
    }

    std::shared_ptr<FfiWaypoint> GetRight(FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiWaypoint>(nullptr), [&]() {
            auto ptr = inner_->GetRight();
            if (ptr) {
                return std::make_shared<FfiWaypoint>(std::move(ptr));
            } else {
                return std::shared_ptr<FfiWaypoint>(nullptr);
            }
        });
    }

    std::shared_ptr<FfiWaypoint> GetLeft(FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiWaypoint>(nullptr), [&]() {
            auto ptr = inner_->GetLeft();
            if (ptr) {
                return std::make_shared<FfiWaypoint>(std::move(ptr));
            } else {
                return std::shared_ptr<FfiWaypoint>(nullptr);
            }
        });
    }

    std::unique_ptr<FfiLaneMarking> GetRightLaneMarking(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLaneMarking>(nullptr), [&]() {
            auto orig = inner_->GetRightLaneMarking();
            if (orig) {
                return std::make_unique<FfiLaneMarking>(std::move(*orig));
            } else {
                return std::unique_ptr<FfiLaneMarking>(nullptr);
            }
        });
    }

    std::unique_ptr<FfiLaneMarking> GetLeftLaneMarking(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLaneMarking>(nullptr), [&]() {
            auto orig = inner_->GetLeftLaneMarking();
            if (orig) {
                return std::make_unique<FfiLaneMarking>(std::move(*orig));
            } else {
                return std::unique_ptr<FfiLaneMarking>(nullptr);
            }
        });
    }

    LaneMarking::LaneChange GetLaneChange() const noexcept { return inner_->GetLaneChange(); }

    std::unique_ptr<FfiLandmarkList> GetAllLandmarksInDistance(double distance,
                                                               bool stop_at_junction,
                                                               FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLandmarkList>(nullptr), [&]() {
            auto orig = inner_->GetAllLandmarksInDistance(distance, stop_at_junction);
            return std::make_unique<FfiLandmarkList>(std::move(orig));
        });
    }

    std::unique_ptr<FfiLandmarkList> GetLandmarksOfTypeInDistance(double distance,
                                                                  std::string filter_type,
                                                                  bool stop_at_junction,
                                                                  FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLandmarkList>(nullptr), [&]() {
            auto orig = inner_->GetLandmarksOfTypeInDistance(distance, std::move(filter_type),
                                                             stop_at_junction);
            return std::make_unique<FfiLandmarkList>(std::move(orig));
        });
    }

    const SharedPtr<Waypoint>& inner() const noexcept { return inner_; }

private:
    SharedPtr<Waypoint> inner_;
};

}  // namespace client
}  // namespace carla_rust
