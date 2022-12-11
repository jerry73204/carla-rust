#pragma once

#include <cstdint>
#include "carla/rpc/ObjectLabel.h"
#include "carla/rpc/EnvironmentObject.h"
#include "carla/geom/Transform.h"
#include "carla_rust/geom.hpp"

namespace carla_rust
{
    namespace rpc {
        using carla::rpc::EnvironmentObject;
        using carla::rpc::CityObjectLabel;
        using carla::geom::Transform;
        using carla::geom::BoundingBox;
        using carla_rust::geom::FfiTransform;
        using carla_rust::geom::FfiBoundingBox;

        struct FfiEnvironmentObjectRef {
        public:
            FfiEnvironmentObjectRef(const EnvironmentObject &orig)
                : inner_(orig)
            {}

            const FfiTransform& transform() const {
                const Transform& orig = inner_.transform;
                return static_cast<const FfiTransform&>(orig);
            }

            const FfiBoundingBox& bounding_box() const {
                const BoundingBox& orig = inner_.bounding_box;
                return static_cast<const FfiBoundingBox&>(orig);
            }

            uint64_t id() const {
                return inner_.id;
            }

            const std::string& name() const {
                return inner_.name;
            }

            CityObjectLabel type() const {
                return inner_.type;
            }

        private:
            const EnvironmentObject& inner_;
        };
    }
}
