#pragma once

#include <memory>
#include "carla/client/ActorAttribute.h"
#include "carla/rpc/ActorAttribute.h"
#include "carla/sensor/data/Color.h"
#include "carla_rust/sensor/data/color.hpp"

namespace carla_rust
{
    namespace client {
        using carla::client::ActorAttributeValue;
        using carla::sensor::data::Color;
        using carla_rust::sensor::data::FfiColor;

        class FfiActorAttributeValue {
        public:
            const std::string &GetId() const {
                return inner_.GetId();
            }

            carla::rpc::ActorAttributeType GetType() const {
                return inner_.GetType();
            }

            // /// Serialize this object as a carla::rpc::ActorAttributeValue.
            // operator rpc::ActorAttributeValue() const{
            //     return inner_.GetType();
            // }

            const std::string &GetValue() const {
                return inner_.GetValue();
            }

            bool to_bool() const {
                return inner_.As<bool>();
            }

            int to_int() const {
                return inner_.As<int>();
            }

            float to_float() const {
                return inner_.As<float>();
            }

            std::string to_string() const {
                return inner_.As<std::string>();
            }

            FfiColor to_color() const {
                auto orig = inner_.As<Color>();
                return FfiColor(std::move(orig));
            }

        private:
            ActorAttributeValue inner_;
        };

        static_assert(sizeof(FfiActorAttributeValue) == sizeof(ActorAttributeValue), "FfiActorAttributeValue has invalid size");
    }
} // namespace carla_rust
