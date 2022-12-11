#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/rpc/EnvironmentObject.h"
#include "carla_rust/rpc/environment_object.hpp"

namespace carla_rust
{
    namespace client {
        using carla::rpc::EnvironmentObject;
        using carla_rust::rpc::FfiEnvironmentObjectRef;

        class FfiEnvironmentObjectList {
        public:
            FfiEnvironmentObjectList(std::vector<EnvironmentObject> &&vec)
                : inner_(std::move(vec))
            {}

            size_t len() const {
                return inner_.size();
            }

            FfiEnvironmentObjectRef get(size_t index) const {
                auto& orig = inner_.at(index);
                return FfiEnvironmentObjectRef(orig);
            }

        private:
            std::vector<EnvironmentObject> inner_;
        };
    }
}
