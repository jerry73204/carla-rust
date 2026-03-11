#pragma once

#include <memory>
#include <string>
#include "carla_rust/error.hpp"

namespace carla_rust {
namespace client {

// Forward declarations (defined in client.hpp and world.hpp, included before this)
class FfiClient;
class FfiWorld;

/// Error container for fallible FFI operations.
class FfiError {
public:
    FfiError() : kind_(0) {}
    bool has_error() const { return kind_ != 0; }
    int32_t kind() const { return kind_; }
    std::string message() const { return message_; }
    void clear() {
        kind_ = 0;
        message_.clear();
    }
    void set(int32_t kind, const std::string& msg) {
        kind_ = kind;
        message_ = msg;
    }

private:
    int32_t kind_;
    std::string message_;
};

/// Try to connect to CARLA. Returns nullptr on failure, populates error.
inline std::unique_ptr<FfiClient> ffi_try_connect(const std::string& host, uint16_t port,
                                                  size_t worker_threads, FfiError& error) {
    error.clear();
    try {
        return std::make_unique<FfiClient>(host, port, worker_threads);
    } catch (const std::exception& e) {
        auto ek = carla_rust::error::classify_exception(e);
        error.set(static_cast<int32_t>(ek), e.what());
        return nullptr;
    } catch (...) {
        error.set(static_cast<int32_t>(carla_rust::error::ErrorKind::Unknown),
                  "Unknown C++ exception");
        return nullptr;
    }
}

/// Try to get world from client. Returns nullptr on failure, populates error.
inline std::unique_ptr<FfiWorld> ffi_try_get_world(const FfiClient& client, FfiError& error) {
    error.clear();
    try {
        auto world = client.GetWorld();
        return std::make_unique<FfiWorld>(std::move(world));
    } catch (const std::exception& e) {
        auto ek = carla_rust::error::classify_exception(e);
        error.set(static_cast<int32_t>(ek), e.what());
        return nullptr;
    } catch (...) {
        error.set(static_cast<int32_t>(carla_rust::error::ErrorKind::Unknown),
                  "Unknown C++ exception");
        return nullptr;
    }
}

}  // namespace client
}  // namespace carla_rust
