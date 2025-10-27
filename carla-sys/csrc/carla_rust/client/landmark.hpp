#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/client/Landmark.h"
#include "carla/road/RoadTypes.h"
#include "carla/road/Signal.h"
#include "carla_rust/geom.hpp"
#include "carla_rust/client/waypoint.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::Landmark;
using carla::road::RoadId;
using carla::road::SignalOrientation;
using carla_rust::client::FfiWaypoint;
using carla_rust::geom::FfiTransform;

// Landmark
class FfiLandmark {
public:
    FfiLandmark(SharedPtr<Landmark>&& base) : inner_(std::move(base)) {}

    std::shared_ptr<FfiWaypoint> GetWaypoint() const {
        auto orig = inner_->GetWaypoint();
        return std::make_shared<FfiWaypoint>(std::move(orig));
    }

    const FfiTransform& GetTransform() const {
        auto& orig = inner_->GetTransform();
        return reinterpret_cast<const FfiTransform&>(orig);
    }

    RoadId GetRoadId() const { return inner_->GetRoadId(); }

    double GetDistance() const { return inner_->GetDistance(); }

    double GetS() const { return inner_->GetS(); }

    double GetT() const { return inner_->GetT(); }

    std::string GetId() const { return inner_->GetId(); }

    std::string GetName() const { return inner_->GetName(); }

    bool IsDynamic() const { return inner_->IsDynamic(); }

    SignalOrientation GetOrientation() const { return inner_->GetOrientation(); }

    double GetZOffset() const { return inner_->GetZOffset(); }

    std::string GetCountry() const { return inner_->GetCountry(); }

    std::string GetType() const { return inner_->GetType(); }

    std::string GetSubType() const { return inner_->GetSubType(); }

    double GetValue() const { return inner_->GetValue(); }

    std::string GetUnit() const { return inner_->GetUnit(); }

    double GetHeight() const { return inner_->GetHeight(); }

    double GetWidth() const { return inner_->GetWidth(); }

    std::string GetText() const { return inner_->GetText(); }

    double GethOffset() const { return inner_->GethOffset(); }

    double GetPitch() const { return inner_->GetPitch(); }

    double GetRoll() const { return inner_->GetRoll(); }

    // const auto &GetValidities() const {
    // }

    const SharedPtr<Landmark>& inner() const { return inner_; }

private:
    SharedPtr<Landmark> inner_;
};
}  // namespace client
}  // namespace carla_rust
