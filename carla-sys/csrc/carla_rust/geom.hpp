#pragma once

#include <vector>
#include "carla/geom/Transform.h"
#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"
#include "carla/geom/Vector2D.h"
#include "carla/geom/Vector3D.h"
#include "carla/geom/BoundingBox.h"

namespace carla_rust {
namespace geom {
using carla::geom::BoundingBox;
using carla::geom::Location;
using carla::geom::Rotation;
using carla::geom::Transform;
using carla::geom::Vector2D;
using carla::geom::Vector3D;

// Location
class FfiLocation {
public:
    float x;
    float y;
    float z;

    FfiLocation(Location&& base) { *this = std::move(reinterpret_cast<FfiLocation&&>(base)); }

    const Location& as_native() const { return reinterpret_cast<const Location&>(*this); }
};

static_assert(sizeof(FfiLocation) == sizeof(Location), "FfiLocation and Location size mismatch");

// Alignment and offset verification for FfiLocation
static_assert(alignof(FfiLocation) == alignof(Location),
              "FfiLocation and Location alignment mismatch");
static_assert(alignof(FfiLocation) == 4, "FfiLocation must have 4-byte alignment (f32 alignment)");
static_assert(offsetof(FfiLocation, x) == 0, "FfiLocation x must be first field (offset 0)");
static_assert(offsetof(FfiLocation, y) == 4, "FfiLocation y must be second field (offset 4)");
static_assert(offsetof(FfiLocation, z) == 8, "FfiLocation z must be third field (offset 8)");

// Transform
class FfiTransform {
public:
    FfiLocation location;
    Rotation rotation;

    FfiTransform(Transform&& base)
        : location(reinterpret_cast<FfiLocation&&>(std::move(base.location))),
          rotation(std::move(base.rotation))

    {}

    FfiTransform(const Transform& base)
        : location(reinterpret_cast<const FfiLocation&>(base.location)),
          rotation(reinterpret_cast<const Rotation&>(base.rotation)) {}

    const Transform& as_native() const { return reinterpret_cast<const Transform&>(*this); }
};

static_assert(sizeof(FfiTransform) == sizeof(Transform),
              "FfiTransform and Transform size mismatch");

// Alignment verification (critical for repr(C) compatibility)
static_assert(alignof(FfiTransform) == alignof(Transform),
              "FfiTransform and Transform alignment mismatch");
static_assert(alignof(FfiTransform) == 4,
              "FfiTransform must have 4-byte alignment (f32 alignment)");

// Field offset verification (ensures field order matches)
static_assert(offsetof(FfiTransform, location) == offsetof(Transform, location),
              "FfiTransform location field offset mismatch");
static_assert(offsetof(FfiTransform, rotation) == offsetof(Transform, rotation),
              "FfiTransform rotation field offset mismatch");

// Field order verification (location must be first, rotation second)
static_assert(offsetof(FfiTransform, location) == 0,
              "FfiTransform location must be first field (offset 0)");
static_assert(offsetof(FfiTransform, rotation) == sizeof(FfiLocation),
              "FfiTransform rotation must immediately follow location");

// BoundingBox
class FfiBoundingBox {
public:
    FfiLocation location;
    Vector3D extent;
    Rotation rotation;
#ifdef CARLA_VERSION_0916
    uint32_t actor_id;
#endif

    FfiBoundingBox(BoundingBox&& base)
        : location(reinterpret_cast<FfiLocation&&>(std::move(base.location))),
          extent(reinterpret_cast<Vector3D&&>(std::move(base.extent))),
          rotation(std::move(base.rotation))
#ifdef CARLA_VERSION_0916
          ,
          actor_id(base.actor_id)
#endif
    {
    }

    FfiBoundingBox(const BoundingBox& base)
        : location(reinterpret_cast<const FfiLocation&>(base.location)),
          extent(reinterpret_cast<const Vector3D&>(base.extent)),
          rotation(reinterpret_cast<const Rotation&>(base.rotation))
#ifdef CARLA_VERSION_0916
          ,
          actor_id(base.actor_id)
#endif
    {
    }

    bool Contains(const FfiLocation& in_world_point,
                  const FfiTransform& in_bbox_to_world_transform) const {
        return as_native().Contains(in_world_point.as_native(),
                                    in_bbox_to_world_transform.as_native());
    }

    std::vector<FfiLocation> GetLocalVertices() const {
        auto orig = as_native().GetLocalVertices();
        std::vector<FfiLocation> new_;

        for (Location& loc : orig) {
            new_.push_back(FfiLocation(std::move(loc)));
        }

        return new_;
    }

    std::vector<FfiLocation> GetWorldVertices(const FfiTransform& in_bbox_to_world_tr) const {
        auto orig = as_native().GetWorldVertices(in_bbox_to_world_tr.as_native());
        std::vector<FfiLocation> new_;

        for (Location& loc : orig) {
            new_.push_back(FfiLocation(std::move(loc)));
        }

        return new_;
    }

    const BoundingBox& as_native() const { return reinterpret_cast<const BoundingBox&>(*this); }
};

// NOLINTNEXTLINE(clang-diagnostic-error)
static_assert(sizeof(FfiBoundingBox) == sizeof(BoundingBox),
              "FfiBoundingBox and BoundingBox size mismatch");
}  // namespace geom
}  // namespace carla_rust
