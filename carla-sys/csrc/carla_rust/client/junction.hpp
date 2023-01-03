#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/client/Junction.h"
#include "carla/road/RoadTypes.h"
#include "carla/road/Lane.h"
#include "carla_rust/client/waypoint.hpp"
#include "carla_rust/client/waypoint_pair.hpp"
#include "carla_rust/geom.hpp"
#include "waypoint_pair.hpp"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::client::Junction;
        using carla::road::JuncId;
        using carla::road::RoadId;
        using carla::road::Lane;
        using carla_rust::client::FfiWaypoint;
        using carla_rust::client::FfiWaypointPair;
        using carla_rust::geom::FfiBoundingBox;

        class FfiJunction {
        public:
            FfiJunction(SharedPtr<Junction> &&base)
                : inner_(std::move(base))
            {}

            JuncId GetId() const {
                return inner_->GetId();
            }

            std::vector<FfiWaypointPair> GetWaypoints(Lane::LaneType type) const {
                auto orig = inner_->GetWaypoints(type);
                std::vector<FfiWaypointPair> new_;

                for (const auto& pair: orig) {
                    new_.push_back(FfiWaypointPair(pair));
                }

                return new_;
            }

            FfiBoundingBox GetBoundingBox() const {
                auto orig = inner_->GetBoundingBox();
                return FfiBoundingBox(std::move(orig));
            }

        private:
            SharedPtr<Junction> inner_;
        };
    }
} // namespace carla_rust
