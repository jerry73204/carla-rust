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
#include "carla/client/detail/EpisodeProxy.h"
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
#include "carla/road/element/LaneMarking.h"
#include "carla/geom/Transform.h"
#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"
#include "carla/geom/Vector2D.h"
#include "carla/geom/Vector3D.h"
#include "carla/geom/BoundingBox.h"
#include "carla/sensor/SensorData.h"
#include "carla/sensor/data/Color.h"
#include "carla/sensor/data/Image.h"
#include "carla/sensor/data/LidarData.h"
#include "carla/sensor/data/SemanticLidarData.h"
#include "carla/sensor/data/IMUMeasurement.h"
#include "carla/sensor/data/GnssMeasurement.h"
#include "carla/sensor/data/ObstacleDetectionEvent.h"
#include "carla/sensor/data/CollisionEvent.h"
#include "carla/sensor/data/LaneInvasionEvent.h"
#include "carla_rust/road.hpp"

namespace carla_rust
{
    namespace sensor {
        namespace data {
            using carla::SharedPtr;
            using carla::client::detail::WeakEpisodeProxy;
            using carla::geom::Vector2D;
            using carla::geom::Vector3D;
            using carla::sensor::data::Image;
            using carla::sensor::data::Color;
            using carla::sensor::data::LidarDetection;
            using carla::sensor::data::SemanticLidarDetection;
            using carla::sensor::data::SemanticLidarData;
            using carla::sensor::data::IMUMeasurement;
            using carla::sensor::data::GnssMeasurement;
            using carla::sensor::data::CollisionEvent;
            using carla::sensor::data::LaneInvasionEvent;
            using carla::sensor::data::ObstacleDetectionEvent;
            using carla::road::element::LaneMarking;
            using carla_rust::client::FfiActor;
            using carla_rust::geom::FfiLocation;
            using carla_rust::geom::FfiTransform;
            using carla_rust::road::element::FfiLaneMarking;

            // Image
            class FfiImage {
            public:
                FfiImage(SharedPtr<Image> &&base) : inner_(std::move(base)) {}

                size_t GetWidth() const {
                    return inner_->GetWidth();
                }

                size_t GetHeight() const {
                    return inner_->GetHeight();
                }

                float GetFOVAngle() const {
                    return inner_->GetFOVAngle();
                }

                size_t size() const {
                    return inner_->size();
                }

                const FfiColor* data() const {
                    auto orig = inner_->data();
                    auto new_ = reinterpret_cast<FfiColor*>(orig);
                    return new_;
                }

                const FfiColor& at(size_t pos) const {
                    const Color& orig = inner_->at(pos);
                    const FfiColor& new_ = reinterpret_cast<const FfiColor&>(orig);
                    return new_;
                }

            private:
                SharedPtr<Image> inner_;
            };
       }
    }
}
