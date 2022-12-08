#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/geom/BoundingBox.h"
#include "carla_rust/geom.hpp"

namespace carla_rust
{
    namespace client {
        using carla::geom::BoundingBox;
        using carla_rust::geom::FfiBoundingBox;

        class FfiBoundingBoxList {
        public:
            FfiBoundingBoxList(std::vector<BoundingBox> &&vec)
                : inner_(std::move(vec))
            {}

            size_t len() const {
                return inner_.size();
            }

            const FfiBoundingBox* data() const {
                return reinterpret_cast<const FfiBoundingBox*>(inner_.data());
            }

        private:
            std::vector<BoundingBox> inner_;
        };
    }
}
