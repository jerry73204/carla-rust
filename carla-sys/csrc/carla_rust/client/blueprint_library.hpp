#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/ActorBlueprint.h"
#include "carla_rust/client/result.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::ActorBlueprint;
using carla::client::BlueprintLibrary;

class FfiBlueprintLibrary {
public:
    FfiBlueprintLibrary(SharedPtr<BlueprintLibrary>&& base) : inner_(std::move(base)) {}

    std::shared_ptr<FfiBlueprintLibrary> filter(const std::string& pattern, FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiBlueprintLibrary>(), [&]() {
            auto lib = inner_->Filter(pattern);
            return std::make_shared<FfiBlueprintLibrary>(std::move(lib));
        });
    }

#ifdef CARLA_VERSION_0915_PLUS
    std::shared_ptr<FfiBlueprintLibrary> filter_by_attribute(const std::string& name,
                                                             const std::string& value,
                                                             FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiBlueprintLibrary>(), [&]() {
            auto lib = inner_->FilterByAttribute(name, value);
            return std::make_shared<FfiBlueprintLibrary>(std::move(lib));
        });
    }
#endif

    const ActorBlueprint* find(const std::string& key, FfiError& error) const {
        return ffi_call(error, static_cast<const ActorBlueprint*>(nullptr),
                        [&]() { return inner_->Find(key); });
    }

    const ActorBlueprint* at(size_t pos, FfiError& error) const {
        return ffi_call(error, static_cast<const ActorBlueprint*>(nullptr),
                        [&]() -> const ActorBlueprint* {
                            if (pos >= inner_->size()) {
                                return nullptr;
                            }
                            return &inner_->at(pos);
                        });
    }

    bool is_empty() const {
        return inner_->empty();
    }

    size_t size() const {
        return inner_->size();
    }

private:
    SharedPtr<BlueprintLibrary> inner_;
};
}  // namespace client
}  // namespace carla_rust
