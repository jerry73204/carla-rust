#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/client/Walker.h"
#include "carla/rpc/WalkerControl.h"
#include "carla/rpc/WalkerBoneControlIn.h"
#include "carla/rpc/WalkerBoneControlOut.h"
#include "../rpc/walker_bone_control.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::Actor;
using carla::client::Walker;
using carla::rpc::WalkerBoneControlIn;
using carla::rpc::WalkerBoneControlOut;
using carla::rpc::WalkerControl;

// Walker
class FfiWalker {
public:
    FfiWalker(SharedPtr<Walker>&& base) : inner_(std::move(base)) {}

    void ApplyControl(const WalkerControl& control) const { inner_->ApplyControl(control); }

    WalkerControl GetWalkerControl() const { return inner_->GetWalkerControl(); }

    void SetBonesTransform(const WalkerBoneControlIn& bones) const {
        inner_->SetBonesTransform(bones);
    }

    // Wrapper version using FfiWalkerBoneControlIn
    void SetBonesTransformFfi(const rpc::FfiWalkerBoneControlIn& bones) const {
        inner_->SetBonesTransform(bones.ToCpp());
    }

    WalkerBoneControlOut GetBonesTransform() const { return inner_->GetBonesTransform(); }

    // Wrapper version using FfiWalkerBoneControlOut
    rpc::FfiWalkerBoneControlOut GetBonesTransformFfi() const {
        return rpc::FfiWalkerBoneControlOut::FromCpp(inner_->GetBonesTransform());
    }

    void BlendPose(float blend) const { inner_->BlendPose(blend); }

    std::shared_ptr<FfiActor> to_actor() const {
        SharedPtr<Actor> ptr = boost::static_pointer_cast<Actor>(inner_);
        return std::make_shared<FfiActor>(std::move(ptr));
    }

private:
    SharedPtr<Walker> inner_;
};
}  // namespace client
}  // namespace carla_rust
