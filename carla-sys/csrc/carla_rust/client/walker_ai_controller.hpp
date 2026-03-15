#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla_rust/compat.hpp"
#include "carla/client/WalkerAIController.h"
#include "carla/geom/Location.h"
#include "carla_rust/geom.hpp"
#include "carla_rust/client/result.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::Actor;
using carla::client::WalkerAIController;
using carla::geom::Location;
using carla_rust::geom::FfiLocation;

// WalkerAIController
class FfiWalkerAIController {
public:
    FfiWalkerAIController(SharedPtr<WalkerAIController>&& base) noexcept
        : inner_(std::move(base)) {}

    void Start(FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->Start(); });
    }

    void Stop(FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->Stop(); });
    }

    std::unique_ptr<FfiLocation> GetRandomLocation(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLocation>(nullptr), [&]() {
            auto result = inner_->GetRandomLocation();
            if (result) {
                return std::make_unique<FfiLocation>(std::move(*result));
            } else {
                return std::unique_ptr<FfiLocation>(nullptr);
            }
        });
    }

    void GoToLocation(const FfiLocation& destination, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->GoToLocation(destination.as_native()); });
    }

    void SetMaxSpeed(float max_speed, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetMaxSpeed(max_speed); });
    }

    std::shared_ptr<FfiActor> to_actor() const noexcept {
        SharedPtr<Actor> ptr = carla_static_pointer_cast<Actor>(inner_);
        return std::make_shared<FfiActor>(std::move(ptr));
    }

private:
    SharedPtr<WalkerAIController> inner_;
};
}  // namespace client
}  // namespace carla_rust
