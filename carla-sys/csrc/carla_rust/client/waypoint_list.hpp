#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/client/Waypoint.h"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::client::Waypoint;

        class FfiWaypoint;

        class FfiWaypointList {
        public:
            FfiWaypointList(std::vector<SharedPtr<Waypoint>> &&vec)
                : inner_(std::move(vec))
            {}

            size_t len() const {
                return inner_.size();
            }

            std::shared_ptr<FfiWaypoint> get(size_t index) const {
                auto orig = inner_.at(index);
                return std::make_shared<FfiWaypoint>(std::move(orig));
            }

        private:
            std::vector<SharedPtr<Waypoint>> inner_;
        };
    }
}
