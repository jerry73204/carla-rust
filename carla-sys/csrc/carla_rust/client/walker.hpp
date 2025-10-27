#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/client/Walker.h"
#include "carla/rpc/WalkerControl.h"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::Actor;
using carla::client::Walker;
using carla::rpc::WalkerControl;

// Walker
class FfiWalker {
public:
    FfiWalker(SharedPtr<Walker>&& base) : inner_(std::move(base)) {}

    void ApplyControl(const WalkerControl& control) const { inner_->ApplyControl(control); }

    WalkerControl GetWalkerControl() const { return inner_->GetWalkerControl(); }

    std::shared_ptr<FfiActor> to_actor() const {
        SharedPtr<Actor> ptr = boost::static_pointer_cast<Actor>(inner_);
        return std::make_shared<FfiActor>(std::move(ptr));
    }

private:
    SharedPtr<Walker> inner_;
};
}  // namespace client
}  // namespace carla_rust
