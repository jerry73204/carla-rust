#pragma once

#include <cstddef>
#include "carla/client/ActorAttribute.h"
#include "carla_rust/utils.hpp"
#include "carla_rust/client/actor_attribute.hpp"

namespace carla_rust
{
    namespace client {
        using carla::client::ActorAttributeValue;
        using carla_rust::utils::VectorRef;
        using carla_rust::client::FfiActorAttributeValue;

        class FfiActorAttributeValueList {
        public:
            FfiActorAttributeValueList(const std::vector<ActorAttributeValue> &ref)
                : inner_(ref)
            {}

            size_t len() const {
                return inner_.len();
            }

            const FfiActorAttributeValue* data() const {
                return reinterpret_cast<const FfiActorAttributeValue*>(inner_.data());
            }

        private:
            const VectorRef<ActorAttributeValue> inner_;
        };
    }
}
