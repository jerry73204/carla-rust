#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/ActorBlueprint.h"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::ActorBlueprint;
using carla::client::BlueprintLibrary;

class FfiBlueprintLibrary {
public:
    FfiBlueprintLibrary(SharedPtr<BlueprintLibrary>&& base) : inner_(std::move(base)) {}

    std::shared_ptr<FfiBlueprintLibrary> filter(const std::string& pattern) const {
        auto lib = inner_->Filter(pattern);
        return std::make_shared<FfiBlueprintLibrary>(std::move(lib));
    }

    const ActorBlueprint* find(const std::string& key) const { return inner_->Find(key); }

    const ActorBlueprint* at(size_t pos) const {
        if (pos >= size()) {
            return nullptr;
        }
        return &inner_->at(pos);
    }

    bool is_empty() const { return inner_->empty(); }

    size_t size() const { return inner_->size(); }

private:
    SharedPtr<BlueprintLibrary> inner_;
};
}  // namespace client
}  // namespace carla_rust
