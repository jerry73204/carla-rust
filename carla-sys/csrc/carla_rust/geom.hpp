#pragma once

#include <vector>
#include "carla/geom/Transform.h"
#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"
#include "carla/geom/Vector2D.h"
#include "carla/geom/Vector3D.h"
#include "carla/geom/BoundingBox.h"


namespace carla_rust
{
    namespace geom {
        using carla::geom::Transform;
        using carla::geom::Location;
        using carla::geom::Rotation;
        using carla::geom::Vector3D;
        using carla::geom::Vector2D;
        using carla::geom::BoundingBox;

        std::vector<Vector2D> new_vector_2d_vector() {
            return std::vector<Vector2D> {};
        }

        // Location
        class FfiLocation {
        public:
            float x;
            float y;
            float z;

            FfiLocation(Location &&base) {
                *this = std::move(reinterpret_cast<FfiLocation&&>(base));
            }

            const Location& as_native() const {
                return reinterpret_cast<const Location&>(*this);
            }
        };

        static_assert(sizeof(FfiLocation) == sizeof(Location), "FfiLocation and Location size mismatch");

        // Transform
        class FfiTransform {
        public:
            FfiLocation location;
            Rotation rotation;

            FfiTransform(Transform &&base)
                :
                location(reinterpret_cast<FfiLocation&&>(std::move(base.location))),
                rotation(std::move(base.rotation))

            {}

            FfiTransform(const Transform &base)
                :
                location(reinterpret_cast<const FfiLocation&>(base.location)),
                rotation(reinterpret_cast<const Rotation&>(base.rotation))
            {}

            const Transform& as_native() const {
                return reinterpret_cast<const Transform&>(*this);
            }
        };

        static_assert(sizeof(FfiTransform) == sizeof(Transform), "FfiTransform and Transform size mismatch");

        // BoundingBox
        class FfiBoundingBox {
        public:
            FfiLocation location;
            Vector3D extent;
            Rotation rotation;

            FfiBoundingBox(BoundingBox &&base)
                :
                location(reinterpret_cast<FfiLocation&&>(std::move(base.location))),
                extent(reinterpret_cast<Vector3D&&>(std::move(base.extent))),
                rotation(std::move(base.rotation))
            {}

            FfiBoundingBox(const BoundingBox &base)
                :
                location(reinterpret_cast<const FfiLocation&>(base.location)),
                extent(reinterpret_cast<const Vector3D&>(base.extent)),
                rotation(reinterpret_cast<const Rotation&>(base.rotation))
            {}

            bool Contains(const FfiLocation &in_world_point,
                          const FfiTransform &in_bbox_to_world_transform) const
            {
                return as_native().Contains(in_world_point.as_native(),
                                            in_bbox_to_world_transform.as_native());
            }

            std::vector<FfiLocation> GetLocalVertices() const {
                auto orig = as_native().GetLocalVertices();
                std::vector<FfiLocation> new_;

                for (Location& loc: orig) {
                    new_.push_back(FfiLocation(std::move(loc)));
                }

                return new_;
            }

            std::vector<FfiLocation> GetWorldVertices(const FfiTransform &in_bbox_to_world_tr) const {
                auto orig = as_native().GetWorldVertices(in_bbox_to_world_tr.as_native());
                std::vector<FfiLocation> new_;

                for (Location& loc: orig) {
                    new_.push_back(FfiLocation(std::move(loc)));
                }

                return new_;
            }

            const BoundingBox& as_native() const {
                return reinterpret_cast<const BoundingBox&>(*this);
            }
        };

        static_assert(sizeof(FfiBoundingBox) == sizeof(BoundingBox), "FfiBoundingBox and BoundingBox size mismatch");
    }
}
