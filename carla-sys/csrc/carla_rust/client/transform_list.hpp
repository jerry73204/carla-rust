#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/geom/Transform.h"
#include "carla_rust/geom.hpp"

namespace carla_rust
{
    namespace client {
        using carla::geom::Transform;
        using carla_rust::geom::FfiTransform;

        class FfiTransformList {
        public:
            FfiTransformList(std::vector<Transform> &&vec)
                : inner_(std::move(vec))
            {}

            size_t len() const {
                return inner_.size();
            }

            const FfiTransform* data() const {
                return reinterpret_cast<const FfiTransform*>(inner_.data());
            }

        private:
            std::vector<Transform> inner_;
        };
    }
}
