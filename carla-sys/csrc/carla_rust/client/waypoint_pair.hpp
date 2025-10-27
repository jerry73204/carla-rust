#pragma once

#include <memory>
#include <utility>
#include "carla/Memory.h"
#include "carla/client/Waypoint.h"
#include "carla_rust/client/waypoint.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::Waypoint;
using carla_rust::client::FfiWaypoint;

class FfiWaypointPair {
public:
    std::shared_ptr<FfiWaypoint> first_;
    std::shared_ptr<FfiWaypoint> second_;

    FfiWaypointPair(const std::pair<SharedPtr<Waypoint>, SharedPtr<Waypoint>>& pair) {
        first_ = std::make_shared<FfiWaypoint>(pair.first);
        second_ = std::make_shared<FfiWaypoint>(pair.second);
    }

    std::shared_ptr<FfiWaypoint> first() const { return first_; }

    std::shared_ptr<FfiWaypoint> second() const { return second_; }

private:
};
}  // namespace client
}  // namespace carla_rust
