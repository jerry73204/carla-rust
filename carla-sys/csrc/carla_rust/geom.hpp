#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include "carla/Time.h"
#include "carla/Memory.h"
#include "carla/client/Client.h"
#include "carla/client/World.h"
#include "carla/client/Map.h"
#include "carla/client/Actor.h"
#include "carla/client/ActorBlueprint.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/Sensor.h"
#include "carla/client/Vehicle.h"
#include "carla/client/TimeoutException.h"
#include "carla/client/ActorList.h"
#include "carla/client/Landmark.h"
#include "carla/client/Waypoint.h"
#include "carla/rpc/AttachmentType.h"
#include "carla/rpc/MapLayer.h"
#include "carla/rpc/OpendriveGenerationParameters.h"
#include "carla/rpc/LabelledPoint.h"
#include "carla/rpc/VehicleControl.h"
#include "carla/rpc/VehiclePhysicsControl.h"
#include "carla/rpc/VehicleDoor.h"
#include "carla/rpc/VehicleWheels.h"
#include "carla/rpc/VehicleLightState.h"
#include "carla/rpc/TrafficLightState.h"
#include "carla/geom/Transform.h"
#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"
#include "carla/geom/Vector2D.h"
#include "carla/geom/Vector3D.h"
#include "carla/geom/BoundingBox.h"
#include "carla/sensor/SensorData.h"
#include "carla/sensor/data/Color.h"
#include "carla/sensor/data/Image.h"


namespace carla_rust
{
    namespace geom {
        using carla::geom::Transform;
        using carla::geom::Location;
        using carla::geom::Rotation;
        using carla::geom::Vector2D;
        using carla::geom::Vector3D;

        // Vector2D
        class FfiVector2D {
        public:
            FfiVector2D(Vector2D &&base)
                : inner_(std::move(base))
            {}

            FfiVector2D(float x, float y)
                : inner_(std::move(Vector2D(x, y)))
            {}

            float x() const {
                return inner_.x;
            }

            float y() const {
                return inner_.y;
            }

            const Vector2D& inner() const {
                return inner_;
            }

        private:
            Vector2D inner_;
        };

        // Vector3D
        class FfiVector3D {
        public:
            FfiVector3D(Vector3D &&base)
                : inner_(std::move(base))
            {}

            FfiVector3D(float x, float y, float z)
                : inner_(std::move(Vector3D(x, y, z)))
            {}

            float x() const {
                return inner_.x;
            }

            float y() const {
                return inner_.y;
            }

            float z() const {
                return inner_.z;
            }

            const Vector3D& inner() const {
                return inner_;
            }

        private:
            Vector3D inner_;
        };

        // Location
        class FfiLocation {
        public:
            FfiLocation(Location &&base)
                : inner_(std::move(base))
            {}

            FfiLocation(float x, float y, float z)
                : FfiLocation(std::move(Location(Vector3D(x, y, z))))
            {}


            float x() const {
                return inner_.x;
            }

            float y() const {
                return inner_.y;
            }

            float z() const {
                return inner_.z;
            }

            const Location &as_location() const {
                return inner_;
            }

            const Vector3D &as_vector_3d() const {
                return static_cast<const Vector3D&>(inner_);
            }

        private:
            Location inner_;
        };

        // Rotation
        class FfiRotation {
        public:
            FfiRotation(Rotation &&base)
                : inner_(std::move(base))
            {}

            FfiRotation(float p, float y, float r)
                : FfiRotation(std::move(Rotation(p, y, r)))
            {}


            float pitch() const {
                return inner_.pitch;
            }

            float yaw() const {
                return inner_.yaw;
            }

            float roll() const {
                return inner_.roll;
            }

            const Rotation &inner() const {
                return inner_;
            }

        private:
            Rotation inner_;
        };


        // Transform
        class FfiTransform {
        public:
            FfiTransform(Transform &&base)
                : inner_(std::move(base))
            {}


            FfiTransform(const FfiLocation &location, const FfiRotation &rotation)
                : inner_(location.as_location(), rotation.inner())
            {
            }

            const Transform& inner() const {
                return inner_;
            }

            FfiLocation location() const {
                Location loc = inner_.location;
                return FfiLocation(std::move(loc));
            }

            FfiRotation rotation() const {
                Rotation rot = inner_.rotation;
                return FfiRotation(std::move(rot));
            }

        private:
            Transform inner_;
        };
    }
}
