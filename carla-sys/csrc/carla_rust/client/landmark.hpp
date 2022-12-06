#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/client/Landmark.h"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::client::Landmark;

        // Landmark
        class FfiLandmark {
        public:
            FfiLandmark(SharedPtr<Landmark> &&base)
                : inner_(std::move(base))
            {}

            const SharedPtr<Landmark>& inner() const {
                return inner_;
            }

        private:
            SharedPtr<Landmark> inner_;
        };
    }
} // namespace carla_rust
