#pragma once

#include <exception>
#include <stdexcept>
#include <string>

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

}  // namespace error
}  // namespace carla_rust
