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
        using carla::geom::Vector3D;

        // Location
        class FfiLocation {
        public:
            float x;
            float y;
            float z;

            FfiLocation(Location &&base) {
                *this = std::move(reinterpret_cast<FfiLocation&&>(base));
            }

            const Location& as_location() const {
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
                rotation(std::move(base.rotation))

            {}

            const Transform& as_transform() const {
                return reinterpret_cast<const Transform&>(*this);
            }
        };

        static_assert(sizeof(FfiTransform) == sizeof(Transform), "FfiTransform and Transform size mismatch");
    }
}
