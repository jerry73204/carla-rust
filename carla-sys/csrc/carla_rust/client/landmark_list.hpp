#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/client/Landmark.h"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::client::Landmark;

        class FfiLandmark;

        class FfiLandmarkList {
        public:
            FfiLandmarkList(std::vector<SharedPtr<Landmark>> &&vec)
                : inner_(std::move(vec))
            {}

            size_t len() const {
                return inner_.size();
            }

            std::shared_ptr<FfiLandmark> get(size_t index) const {
                auto orig = inner_.at(index);
                return std::make_shared<FfiLandmark>(std::move(orig));
            }

        private:
            std::vector<SharedPtr<Landmark>> inner_;
        };
    }
}
