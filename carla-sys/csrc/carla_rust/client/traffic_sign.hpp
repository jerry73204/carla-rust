#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/road/RoadTypes.h"
#include "carla/geom/BoundingBox.h"
#include "carla_rust/geom.hpp"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::road::SignId;
        using carla::geom::BoundingBox;
        using carla_rust::geom::FfiBoundingBox;

        // TrafficSign
        class FfiTrafficSign {
        public:
            FfiTrafficSign(SharedPtr<TrafficSign> &&base)
                : inner_(std::move(base))
            {}

            const FfiBoundingBox &GetTriggerVolume() const {
                const BoundingBox& orig = inner_->GetTriggerVolume();
                const FfiBoundingBox& new_ = reinterpret_cast<const FfiBoundingBox&>(orig);
                return new_;
            }

            SignId GetSignId() const {
                return inner_->GetSignId();
            }

            std::shared_ptr<FfiActor> to_actor() const {
                SharedPtr<Actor> ptr = boost::static_pointer_cast<Actor>(inner_);
                return std::make_shared<FfiActor>(std::move(ptr));
            }

        private:
            SharedPtr<TrafficSign> inner_;
        };
    }
} // namespace carla_rust
