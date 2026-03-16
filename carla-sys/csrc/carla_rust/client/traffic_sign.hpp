#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla_rust/compat.hpp"
#include "carla/road/RoadTypes.h"
#include "carla/geom/BoundingBox.h"
#include "carla/client/TrafficSign.h"
#include "carla_rust/geom.hpp"
#include "carla_rust/client/result.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::TrafficSign;
using carla::geom::BoundingBox;
using carla::road::SignId;
using carla_rust::client::ffi_call;
using carla_rust::client::FfiError;
using carla_rust::geom::FfiBoundingBox;

// TrafficSign
class FfiTrafficSign {
public:
    FfiTrafficSign(SharedPtr<TrafficSign>&& base) : inner_(std::move(base)) {}

    FfiBoundingBox GetTriggerVolume(FfiError& error) const {
        return ffi_call(error, FfiBoundingBox(), [&]() {
            const BoundingBox& orig = inner_->GetTriggerVolume();
            return reinterpret_cast<const FfiBoundingBox&>(orig);
        });
    }

    std::unique_ptr<std::string> GetSignId(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<std::string>(nullptr),
                        [&]() { return std::make_unique<std::string>(inner_->GetSignId()); });
    }

    std::shared_ptr<FfiActor> to_actor() const {
        SharedPtr<Actor> ptr = carla_static_pointer_cast<Actor>(inner_);
        return std::make_shared<FfiActor>(std::move(ptr));
    }

private:
    SharedPtr<TrafficSign> inner_;
};
}  // namespace client
}  // namespace carla_rust
