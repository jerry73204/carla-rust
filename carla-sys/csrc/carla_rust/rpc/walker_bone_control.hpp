#pragma once

#include <memory>
#include <vector>
#include <string>
#include "carla/Memory.h"
#include "carla/geom/Transform.h"
#include "carla/rpc/Transform.h"
#include "carla/rpc/WalkerBoneControlIn.h"
#include "carla/rpc/WalkerBoneControlOut.h"

namespace carla_rust {
namespace rpc {

using carla::geom::Transform;
using carla::rpc::BoneTransformDataIn;
using carla::rpc::BoneTransformDataOut;
using carla::rpc::WalkerBoneControlIn;
using carla::rpc::WalkerBoneControlOut;

// Wrapper for BoneTransformDataIn with field access
// BoneTransformDataIn is std::pair<std::string, geom::Transform>
struct FfiBoneTransformDataIn {
    std::string bone_name;
    Transform transform;

    BoneTransformDataIn ToCpp() const {
        // Create the pair: first=bone_name, second=transform
        return std::make_pair(bone_name, transform);
    }
};

// Wrapper for BoneTransformDataOut with field access
class FfiBoneTransformDataOut {
public:
    FfiBoneTransformDataOut() = default;

    static FfiBoneTransformDataOut FromCpp(const BoneTransformDataOut& cpp) {
        FfiBoneTransformDataOut result;
        result.bone_name_ = cpp.bone_name;
        result.world_ = cpp.world;
        result.component_ = cpp.component;
        result.relative_ = cpp.relative;
        return result;
    }

    // Getters for Rust access
    std::string GetBoneName() const { return bone_name_; }
    Transform GetWorld() const { return world_; }
    Transform GetComponent() const { return component_; }
    Transform GetRelative() const { return relative_; }

private:
    std::string bone_name_;
    Transform world_;
    Transform component_;
    Transform relative_;
};

// Wrapper for WalkerBoneControlIn with vector conversion
class FfiWalkerBoneControlIn {
public:
    FfiWalkerBoneControlIn() = default;

    void AddBone(const FfiBoneTransformDataIn& bone) { bones_.push_back(bone); }

    // Helper to add bone with string and transform directly
    void AddBoneSimple(const std::string& name, const Transform& transform) {
        FfiBoneTransformDataIn bone;
        bone.bone_name = name;
        bone.transform = transform;
        bones_.push_back(bone);
    }

    WalkerBoneControlIn ToCpp() const {
        WalkerBoneControlIn result;
        for (const auto& bone : bones_) {
            result.bone_transforms.push_back(bone.ToCpp());
        }
        return result;
    }

private:
    std::vector<FfiBoneTransformDataIn> bones_;
};

// Wrapper for WalkerBoneControlOut with vector access
class FfiWalkerBoneControlOut {
public:
    FfiWalkerBoneControlOut() = default;

    static FfiWalkerBoneControlOut FromCpp(const WalkerBoneControlOut& cpp) {
        FfiWalkerBoneControlOut result;
        for (const auto& bone : cpp.bone_transforms) {
            result.bones_.push_back(FfiBoneTransformDataOut::FromCpp(bone));
        }
        return result;
    }

    size_t GetBoneCount() const { return bones_.size(); }

    FfiBoneTransformDataOut GetBone(size_t index) const {
        if (index >= bones_.size()) {
            throw std::out_of_range("Bone index out of range");
        }
        return bones_[index];
    }

private:
    std::vector<FfiBoneTransformDataOut> bones_;
};

}  // namespace rpc
}  // namespace carla_rust
