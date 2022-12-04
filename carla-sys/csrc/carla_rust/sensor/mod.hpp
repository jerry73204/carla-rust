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
#include "data/mod.hpp"


namespace carla_rust
{
    namespace sensor {
        using carla::SharedPtr;
        using carla::sensor::SensorData;
        using carla::sensor::data::Image;
        using carla::sensor::data::CollisionEvent;
        using carla::sensor::data::ObstacleDetectionEvent;
        using carla::sensor::data::LaneInvasionEvent;
        using carla::geom::Transform;
        using carla_rust::geom::FfiLocation;
        using carla_rust::geom::FfiTransform;
        using carla_rust::sensor::data::FfiImage;
        using carla_rust::sensor::data::FfiCollisionEvent;
        using carla_rust::sensor::data::FfiObstacleDetectionEvent;
        using carla_rust::sensor::data::FfiLaneInvasionEvent;

        class FfiSensorData {
        public:
            FfiSensorData(SharedPtr<SensorData> &&base)
                : inner_(std::move(base))
            {}

            size_t GetFrame() const {
                return inner_->GetFrame();
            }

            /// Simulation-time when the data was generated.
            double GetTimestamp() const {
                return inner_->GetTimestamp();
            }

            /// Sensor's transform when the data was generated.
            const FfiTransform GetSensorTransform() const {
                auto transform = inner_->GetSensorTransform();
                return FfiTransform(std::move(transform));
            }

            std::shared_ptr<FfiImage> to_image() const {
                auto ptr = boost::dynamic_pointer_cast<Image>(inner_);
                if (ptr == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiImage>(std::move(ptr));
                }
            }

            std::shared_ptr<FfiCollisionEvent> to_collision_event() const {
                auto ptr = boost::dynamic_pointer_cast<CollisionEvent>(inner_);
                if (ptr == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiCollisionEvent>(std::move(ptr));
                }
            }

            std::shared_ptr<FfiObstacleDetectionEvent> to_obstacle_detection_event() const {
                auto ptr = boost::dynamic_pointer_cast<ObstacleDetectionEvent>(inner_);
                if (ptr == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiObstacleDetectionEvent>(std::move(ptr));
                }
            }

            std::shared_ptr<FfiLaneInvasionEvent> to_lane_invasion_event() const {
                auto ptr = boost::dynamic_pointer_cast<LaneInvasionEvent>(inner_);
                if (ptr == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiLaneInvasionEvent>(std::move(ptr));
                }
            }

        private:
            SharedPtr<SensorData> inner_;
        };
    }

}
