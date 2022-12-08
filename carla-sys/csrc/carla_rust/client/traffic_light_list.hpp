#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/geom/Transform.h"
#include "carla/client/TrafficLight.h"
#include "carla_rust/geom.hpp"

namespace carla_rust
{
    namespace client {
        using carla::client::TrafficLight;

        class FfiTrafficLight;

        class FfiTrafficLightList {
        public:
            FfiTrafficLightList(std::vector<SharedPtr<TrafficLight>> &&vec)
                : inner_(std::move(vec))
            {}

            size_t len() const {
                return inner_.size();
            }

            std::shared_ptr<FfiTrafficLight> get(size_t index) const {
                auto orig = inner_.at(index);
                return std::make_shared<FfiTrafficLight>(std::move(orig));
            }

        private:
            std::vector<SharedPtr<TrafficLight>> inner_;
        };
    }
}
