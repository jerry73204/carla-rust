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
#include "geom.hpp"
#include "road.hpp"


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

            // Color
            class FfiColor {
            public:
                uint8_t b;
                uint8_t g;
                uint8_t r;
                uint8_t a;

                FfiColor(Color &&base)
                    :
                    b(std::move(base.b)),
                    g(std::move(base.g)),
                    r(std::move(base.r)),
                    a(std::move(base.a))
                {}

                Color& as_builtin() {
                    return reinterpret_cast<Color&>(*this);
                }
            };
            static_assert(sizeof(FfiColor) == sizeof(Color), "Invalid FfiColor size!");


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

            // LidarDetection
            class FfiLidarDetection {

            public:
                FfiLocation point;
                float intensity;


                FfiLidarDetection(LidarDetection &&base)
                    :
                    point(reinterpret_cast<FfiLocation&&>(std::move(base.point))),
                    intensity(std::move(base.intensity))
                {

                }

                LidarDetection& as_builtin() {
                    return reinterpret_cast<LidarDetection&>(*this);
                }
            };
            static_assert(sizeof(FfiLidarDetection) == sizeof(LidarDetection), "FfiLidarDetection and LidarDetection size mismatch");


            // SemanticLidarDetection
            class FfiSemanticLidarDetection {
            public:
                FfiLocation point;
                float cos_inc_angle;
                uint32_t object_idx;
                uint32_t object_tag;


                FfiSemanticLidarDetection(SemanticLidarDetection &&base)
                    :
                    point(reinterpret_cast<FfiLocation&&>(std::move(base.point))),
                    cos_inc_angle(std::move(base.cos_inc_angle)),
                    object_idx(std::move(base.object_idx)),
                    object_tag(std::move(base.object_tag))
                {

                }

                SemanticLidarDetection& as_builtin() {
                    return reinterpret_cast<SemanticLidarDetection&>(*this);
                }
            };
            static_assert(sizeof(FfiSemanticLidarDetection) == sizeof(SemanticLidarDetection), "FfiSemanticLidarDetection and SemanticLidarDetection size mismatch");


            // ObstacleDetectionEvent
            class FfiObstacleDetectionEvent {
            public:
                FfiObstacleDetectionEvent(SharedPtr<ObstacleDetectionEvent> &&base)
                    : inner_(std::move(base))
                {}

                std::shared_ptr<FfiActor> GetActor() const {
                    return std::make_shared<FfiActor>(std::move(inner_->GetActor()));
                }

                std::shared_ptr<FfiActor> GetOtherActor() const {
                    return std::make_shared<FfiActor>(std::move(inner_->GetOtherActor()));
                }

                float GetDistance() const {
                    return inner_->GetDistance();
                }


            private:
                SharedPtr<ObstacleDetectionEvent> inner_;
            };


            // CollisionEvent
            class FfiCollisionEvent {
            public:
                FfiCollisionEvent(SharedPtr<CollisionEvent> &&base)
                    : inner_(std::move(base))
                {}

                std::shared_ptr<FfiActor> GetActor() const {
                    return std::make_shared<FfiActor>(std::move(inner_->GetActor()));
                }

                std::shared_ptr<FfiActor> GetOtherActor() const {
                    return std::make_shared<FfiActor>(std::move(inner_->GetOtherActor()));
                }

                const Vector3D &GetNormalImpulse() const {
                    return inner_->GetNormalImpulse();
                }


            private:
                SharedPtr<CollisionEvent> inner_;
            };

            // LaneInvasionEvent
            class FfiLaneInvasionEvent {
            public:
                FfiLaneInvasionEvent(SharedPtr<LaneInvasionEvent> &&base)
                    : inner_(std::move(base))
                {}

                std::shared_ptr<FfiActor> GetActor() const {
                    return std::make_shared<FfiActor>(std::move(inner_->GetActor()));
                }

                const std::vector<FfiLaneMarking>& GetCrossedLaneMarkings() const {
                    const std::vector<LaneMarking>& orig = inner_->GetCrossedLaneMarkings();
                    const std::vector<FfiLaneMarking>& new_ = reinterpret_cast<const std::vector<FfiLaneMarking>&>(orig);
                    return new_;
                }

            private:
                SharedPtr<LaneInvasionEvent> inner_;
            };
       }

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
