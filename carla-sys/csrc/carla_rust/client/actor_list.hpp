#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/client/ActorList.h"
#include "carla_rust/client/result.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::ActorList;

class FfiActorList {
public:
    FfiActorList(SharedPtr<ActorList>&& base) : inner_(std::move(base)) {}

    std::shared_ptr<FfiActor> Find(uint32_t actor_id, FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiActor>(), [&]() -> std::shared_ptr<FfiActor> {
            auto actor = inner_->Find(actor_id);
            if (actor == nullptr) {
                return nullptr;
            } else {
                return std::make_shared<FfiActor>(std::move(actor));
            }
        });
    }

    std::shared_ptr<FfiActorList> Filter(const std::string& wildcard_pattern,
                                         FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiActorList>(), [&]() {
            auto list = inner_->Filter(wildcard_pattern);
            return std::make_shared<FfiActorList>(std::move(list));
        });
    }

    std::shared_ptr<FfiActor> at(size_t pos, FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiActor>(), [&]() -> std::shared_ptr<FfiActor> {
            auto actor = inner_->at(pos);
            if (actor == nullptr) {
                return nullptr;
            } else {
                return std::make_shared<FfiActor>(std::move(actor));
            }
        });
    }

    bool empty() const { return inner_->empty(); }

    size_t size() const { return inner_->size(); }

private:
    SharedPtr<ActorList> inner_;
};
}  // namespace client
}  // namespace carla_rust
