#pragma once

#include <exception>
#include <stdexcept>
#include <string>
#include <cstring>

namespace carla_rust {
namespace error {

/// Error classification for FFI boundary.
///
/// These error codes allow Rust code to classify C++ exceptions
/// and convert them to appropriate CarlaError variants.
enum class ErrorKind {
    Success = 0,
    Timeout = 1,
    NotFound = 2,
    InvalidArgument = 3,
    RuntimeError = 4,
    OutOfRange = 5,
    Unknown = 6,
};

/// Classify a C++ exception by analyzing its type and message.
///
/// This function attempts to determine the specific error category
/// by checking exception types and analyzing error messages.
inline ErrorKind classify_exception(const std::exception& e) {
    const char* what = e.what();
    std::string msg(what);

    // Check for carla::client::TimeoutException by checking type
    // Note: Can't use dynamic_cast across DSO boundaries reliably,
    // so we rely on message patterns

    // Timeout detection
    if (msg.find("timeout") != std::string::npos || msg.find("Timeout") != std::string::npos ||
        msg.find("timed out") != std::string::npos) {
        return ErrorKind::Timeout;
    }

    // Not found detection
    if (msg.find("not found") != std::string::npos || msg.find("Not found") != std::string::npos ||
        msg.find("does not exist") != std::string::npos ||
        msg.find("doesn't exist") != std::string::npos ||
        msg.find("unable to find") != std::string::npos) {
        return ErrorKind::NotFound;
    }

    // Out of range detection
    if (msg.find("out of range") != std::string::npos ||
        msg.find("Out of range") != std::string::npos || msg.find("index") != std::string::npos) {
        // Try dynamic_cast for std::out_of_range
        if (dynamic_cast<const std::out_of_range*>(&e)) {
            return ErrorKind::OutOfRange;
        }
    }

    // Invalid argument detection
    if (msg.find("invalid") != std::string::npos || msg.find("Invalid") != std::string::npos) {
        // Try dynamic_cast for std::invalid_argument
        if (dynamic_cast<const std::invalid_argument*>(&e)) {
            return ErrorKind::InvalidArgument;
        }
    }

    // Runtime error detection (broad category)
    if (dynamic_cast<const std::runtime_error*>(&e)) {
        return ErrorKind::RuntimeError;
    }

    // Unknown error
    return ErrorKind::Unknown;
}

/// Error information structure for passing across FFI boundary.
///
/// This struct contains all information needed to reconstruct
/// a CarlaError in Rust.
struct ErrorInfo {
    ErrorKind kind;
    char message[512];  // Fixed size for C compatibility

    ErrorInfo() : kind(ErrorKind::Success) { message[0] = '\0'; }

    ErrorInfo(ErrorKind k, const char* msg) : kind(k) {
        std::strncpy(message, msg, sizeof(message) - 1);
        message[sizeof(message) - 1] = '\0';
    }

    ErrorInfo(ErrorKind k, const std::string& msg) : kind(k) {
        std::strncpy(message, msg.c_str(), sizeof(message) - 1);
        message[sizeof(message) - 1] = '\0';
    }
};

/// Helper macro to wrap FFI functions with exception handling.
///
/// Usage:
/// ```cpp
/// CARLA_TRY {
///     // Your code that might throw
///     return some_operation();
/// } CARLA_CATCH(error_info_out);
/// return default_value;  // On error
/// ```
#define CARLA_TRY try

#define CARLA_CATCH(error_out)                                                             \
    catch (const std::exception& e) {                                                      \
        auto kind = carla_rust::error::classify_exception(e);                              \
        *(error_out) = carla_rust::error::ErrorInfo(kind, e.what());                       \
    }                                                                                      \
    catch (...) {                                                                          \
        *(error_out) = carla_rust::error::ErrorInfo(carla_rust::error::ErrorKind::Unknown, \
                                                    "Unknown C++ exception");              \
    }

/// Helper to create success ErrorInfo
inline ErrorInfo success() {
    return ErrorInfo();
}

/// Helper to check if ErrorInfo represents an error
inline bool is_error(const ErrorInfo& info) {
    return info.kind != ErrorKind::Success;
}

}  // namespace error
}  // namespace carla_rust
