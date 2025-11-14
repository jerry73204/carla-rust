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

    FfiLocation() : x(0.0f), y(0.0f), z(0.0f) {}

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

// Rotation
class FfiRotation {
public:
    float pitch;
    float yaw;
    float roll;

    FfiRotation() : pitch(0.0f), yaw(0.0f), roll(0.0f) {}

    FfiRotation(Rotation&& base) { *this = std::move(reinterpret_cast<FfiRotation&&>(base)); }

    FfiRotation(const Rotation& base) { *this = reinterpret_cast<const FfiRotation&>(base); }

    const Rotation& as_native() const { return reinterpret_cast<const Rotation&>(*this); }

    Vector3D GetForwardVector() const { return as_native().GetForwardVector(); }

    Vector3D GetRightVector() const { return as_native().GetRightVector(); }

    Vector3D GetUpVector() const { return as_native().GetUpVector(); }

    Vector3D RotateVector(const Vector3D& point) const { return as_native().RotateVector(point); }

    Vector3D InverseRotateVector(const Vector3D& point) const {
        Vector3D result = point;
        as_native().InverseRotateVector(result);
        return result;
    }
};

static_assert(sizeof(FfiRotation) == sizeof(Rotation), "FfiRotation and Rotation size mismatch");
static_assert(alignof(FfiRotation) == alignof(Rotation),
              "FfiRotation and Rotation alignment mismatch");
static_assert(alignof(FfiRotation) == 4, "FfiRotation must have 4-byte alignment");
static_assert(offsetof(FfiRotation, pitch) == 0, "FfiRotation pitch must be first field");
static_assert(offsetof(FfiRotation, yaw) == 4, "FfiRotation yaw must be second field");
static_assert(offsetof(FfiRotation, roll) == 8, "FfiRotation roll must be third field");

// Transform
class FfiTransform {
public:
    FfiLocation location;
    FfiRotation rotation;

    FfiTransform() : location(), rotation() {}

    FfiTransform(Transform&& base)
        : location(reinterpret_cast<FfiLocation&&>(std::move(base.location))),
          rotation(reinterpret_cast<FfiRotation&&>(std::move(base.rotation)))

    {}

    FfiTransform(const Transform& base) {
        // Use memcpy to avoid strict-aliasing warnings
        // This is safe because we've verified layout compatibility via static_assert
        std::memcpy(&location, &base.location, sizeof(FfiLocation));
        std::memcpy(&rotation, &base.rotation, sizeof(FfiRotation));
    }

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

// Vector3D
class FfiVector3D {
public:
    float x;
    float y;
    float z;

    FfiVector3D() : x(0.0f), y(0.0f), z(0.0f) {}

    FfiVector3D(Vector3D&& base) { *this = std::move(reinterpret_cast<FfiVector3D&&>(base)); }

    FfiVector3D(const Vector3D& base) { *this = reinterpret_cast<const FfiVector3D&>(base); }

    const Vector3D& as_native() const { return reinterpret_cast<const Vector3D&>(*this); }
};

static_assert(sizeof(FfiVector3D) == sizeof(Vector3D), "FfiVector3D and Vector3D size mismatch");
static_assert(alignof(FfiVector3D) == alignof(Vector3D),
              "FfiVector3D and Vector3D alignment mismatch");
static_assert(alignof(FfiVector3D) == 4, "FfiVector3D must have 4-byte alignment");
static_assert(offsetof(FfiVector3D, x) == 0, "FfiVector3D x must be first field");
static_assert(offsetof(FfiVector3D, y) == 4, "FfiVector3D y must be second field");
static_assert(offsetof(FfiVector3D, z) == 8, "FfiVector3D z must be third field");

// Vector2D
class FfiVector2D {
public:
    float x;
    float y;

    FfiVector2D(Vector2D&& base) { *this = std::move(reinterpret_cast<FfiVector2D&&>(base)); }

    FfiVector2D(const Vector2D& base) { *this = reinterpret_cast<const FfiVector2D&>(base); }

    const Vector2D& as_native() const { return reinterpret_cast<const Vector2D&>(*this); }
};

static_assert(sizeof(FfiVector2D) == sizeof(Vector2D), "FfiVector2D and Vector2D size mismatch");
static_assert(alignof(FfiVector2D) == alignof(Vector2D),
              "FfiVector2D and Vector2D alignment mismatch");
static_assert(alignof(FfiVector2D) == 4, "FfiVector2D must have 4-byte alignment");
static_assert(offsetof(FfiVector2D, x) == 0, "FfiVector2D x must be first field");
static_assert(offsetof(FfiVector2D, y) == 4, "FfiVector2D y must be second field");

// GeoLocation
class FfiGeoLocation {
public:
    double latitude;
    double longitude;
    double altitude;

    FfiGeoLocation(carla::geom::GeoLocation&& base) {
        *this = std::move(reinterpret_cast<FfiGeoLocation&&>(base));
    }

    FfiGeoLocation(const carla::geom::GeoLocation& base) {
        *this = reinterpret_cast<const FfiGeoLocation&>(base);
    }

    const carla::geom::GeoLocation& as_native() const {
        return reinterpret_cast<const carla::geom::GeoLocation&>(*this);
    }
};

static_assert(sizeof(FfiGeoLocation) == sizeof(carla::geom::GeoLocation),
              "FfiGeoLocation and GeoLocation size mismatch");
static_assert(alignof(FfiGeoLocation) == alignof(carla::geom::GeoLocation),
              "FfiGeoLocation and GeoLocation alignment mismatch");
static_assert(alignof(FfiGeoLocation) == 8, "FfiGeoLocation must have 8-byte alignment (f64)");
static_assert(offsetof(FfiGeoLocation, latitude) == 0,
              "FfiGeoLocation latitude must be first field");
static_assert(offsetof(FfiGeoLocation, longitude) == 8,
              "FfiGeoLocation longitude must be second field");
static_assert(offsetof(FfiGeoLocation, altitude) == 16,
              "FfiGeoLocation altitude must be third field");

// BoundingBox
class FfiBoundingBox {
public:
    FfiLocation location;
    FfiVector3D extent;
    FfiRotation rotation;
#ifdef CARLA_VERSION_0916
    uint32_t actor_id;
#endif

    FfiBoundingBox(BoundingBox&& base)
        : location(reinterpret_cast<FfiLocation&&>(std::move(base.location))),
          extent(reinterpret_cast<FfiVector3D&&>(std::move(base.extent))),
          rotation(reinterpret_cast<FfiRotation&&>(std::move(base.rotation)))
#ifdef CARLA_VERSION_0916
          ,
          actor_id(base.actor_id)
#endif
    {
    }

    FfiBoundingBox(const BoundingBox& base)
#ifdef CARLA_VERSION_0916
        : actor_id(base.actor_id)
#endif
    {
        // Use memcpy to avoid strict-aliasing warnings
        // This is safe because we've verified layout compatibility via static_assert
        std::memcpy(&location, &base.location, sizeof(FfiLocation));
        std::memcpy(&extent, &base.extent, sizeof(FfiVector3D));
        std::memcpy(&rotation, &base.rotation, sizeof(FfiRotation));
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
