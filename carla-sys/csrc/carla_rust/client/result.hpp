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

// ============================================================================
// Exception-safe FFI call templates
// ============================================================================

/// Invoke a callable, catching any C++ exception and storing it in `error`.
///
/// On success, returns the callable's result.
/// On exception, populates `error` and returns `default_val`.
/// R is deduced from `default_val`.
///
/// Usage:
/// ```cpp
/// auto result = ffi_call(error, std::unique_ptr<FfiWorld>(nullptr), [&]() {
///     return std::make_unique<FfiWorld>(client.GetWorld());
/// });
/// ```
template <typename R, typename F>
R ffi_call(FfiError& error, R default_val, F fn) {
    error.clear();
    try {
        return fn();
    } catch (const std::exception& e) {
        auto ek = carla_rust::error::classify_exception(e);
        error.set(static_cast<int32_t>(ek), e.what());
        return default_val;
    } catch (...) {
        error.set(static_cast<int32_t>(carla_rust::error::ErrorKind::Unknown),
                  "Unknown C++ exception");
        return default_val;
    }
}

/// Invoke a void callable, catching any C++ exception and storing it in `error`.
///
/// Usage:
/// ```cpp
/// ffi_call_void(error, [&]() {
///     world.SetWeather(weather);
/// });
/// ```
template <typename F>
void ffi_call_void(FfiError& error, F fn) {
    error.clear();
    try {
        fn();
    } catch (const std::exception& e) {
        auto ek = carla_rust::error::classify_exception(e);
        error.set(static_cast<int32_t>(ek), e.what());
    } catch (...) {
        error.set(static_cast<int32_t>(carla_rust::error::ErrorKind::Unknown),
                  "Unknown C++ exception");
    }
}

}  // namespace client
}  // namespace carla_rust
