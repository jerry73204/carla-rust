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
    namespace sensor {
        namespace data {
            using carla::SharedPtr;
            using carla::sensor::data::Image;
            using carla::sensor::data::Color;

            typedef struct FfiColor {
                uint8_t b;
                uint8_t g;
                uint8_t r;
                uint8_t a;
            } FfiColor;
            static_assert(sizeof(FfiColor) == sizeof(Color), "Invalid FfiColor size!");

            FfiColor to_ffi_color(Color &orig) {
                FfiColor color;
                color.r = orig.r;
                color.g = orig.g;
                color.b = orig.b;
                color.a = orig.a;
                return color;
            }

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

        using carla::SharedPtr;
        using carla::sensor::SensorData;
        using carla::sensor::data::Image;
        using carla::geom::Transform;
        using carla_rust::geom::FfiTransform;
        using carla_rust::sensor::data::FfiImage;

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
                SharedPtr<Image> ptr = boost::dynamic_pointer_cast<Image>(inner_);
                if (ptr == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiImage>(std::move(ptr));
                }
            }

        private:
            SharedPtr<SensorData> inner_;
        };
    }

}
