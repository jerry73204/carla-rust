#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/client/WalkerAIController.h"
#include "carla/geom/Location.h"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::Actor;
using carla::client::WalkerAIController;
using carla::geom::Location;

// WalkerAIController
class FfiWalkerAIController {
public:
    FfiWalkerAIController(SharedPtr<WalkerAIController>&& base) : inner_(std::move(base)) {}

    void Start() const { inner_->Start(); }

    void Stop() const { inner_->Stop(); }

    boost::optional<Location> GetRandomLocation() const { return inner_->GetRandomLocation(); }

    void GoToLocation(const Location& destination) const { inner_->GoToLocation(destination); }

    void SetMaxSpeed(float max_speed) const { inner_->SetMaxSpeed(max_speed); }

    std::shared_ptr<FfiActor> to_actor() const {
        SharedPtr<Actor> ptr = boost::static_pointer_cast<Actor>(inner_);
        return std::make_shared<FfiActor>(std::move(ptr));
    }

private:
    SharedPtr<WalkerAIController> inner_;
};
}  // namespace client
}  // namespace carla_rust
