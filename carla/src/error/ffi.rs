//! FFI error handling helpers.
//!
//! This module provides utilities for converting C++ exceptions captured at the FFI
//! boundary into structured Rust error types.

use super::*;
use std::time::Duration;

/// Error kind codes matching C++ ErrorKind enum.
///
/// These values must match the ErrorKind enum in carla-sys/csrc/carla_rust/error.hpp
#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FfiErrorKind {
    Success = 0,
    Timeout = 1,
    NotFound = 2,
    InvalidArgument = 3,
    RuntimeError = 4,
    OutOfRange = 5,
    Unknown = 6,
}

impl From<i32> for FfiErrorKind {
    fn from(code: i32) -> Self {
        match code {
            0 => FfiErrorKind::Success,
            1 => FfiErrorKind::Timeout,
            2 => FfiErrorKind::NotFound,
            3 => FfiErrorKind::InvalidArgument,
            4 => FfiErrorKind::RuntimeError,
            5 => FfiErrorKind::OutOfRange,
            6 => FfiErrorKind::Unknown,
            _ => FfiErrorKind::Unknown,
        }
    }
}

/// Parse FFI error kind and message into a CarlaError.
///
/// This function is the main entry point for converting C++ exceptions
/// into Rust error types.
///
/// # Arguments
///
/// * `kind` - Error kind code from C++
/// * `message` - Error message from C++ exception
/// * `operation` - Optional operation name for context
///
/// # Examples
///
/// ```ignore
/// let error = parse_ffi_error(
///     FfiErrorKind::Timeout as i32,
///     "Operation timed out after 5 seconds",
///     Some("spawn_actor")
/// );
/// ```
pub fn parse_ffi_error(kind: i32, message: &str, operation: Option<&str>) -> CarlaError {
    let ffi_kind = FfiErrorKind::from(kind);
    parse_ffi_error_with_kind(ffi_kind, message, operation)
}

/// Parse FFI error with already-converted error kind.
pub fn parse_ffi_error_with_kind(
    kind: FfiErrorKind,
    message: &str,
    operation: Option<&str>,
) -> CarlaError {
    match kind {
        FfiErrorKind::Success => {
            // This shouldn't happen, but if it does, treat as internal error
            InternalError::UnexpectedState {
                description: "FFI returned success code but error was expected".to_string(),
            }
            .into()
        }

        FfiErrorKind::Timeout => {
            // Try to extract duration from message if present
            let duration = extract_timeout_duration(message).unwrap_or(Duration::from_secs(30));

            ConnectionError::Timeout {
                operation: operation
                    .map(|s| s.to_string())
                    .unwrap_or_else(|| "unknown".to_string()),
                duration,
                source: Some(Box::new(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    message.to_string(),
                ))),
            }
            .into()
        }

        FfiErrorKind::NotFound => {
            // Try to extract resource type and identifier from message
            let (resource_type, identifier) = extract_resource_info(message);

            ResourceError::NotFound {
                resource_type,
                identifier,
                context: operation.map(|s| s.to_string()),
            }
            .into()
        }

        FfiErrorKind::InvalidArgument => ValidationError::InvalidConfiguration {
            setting: operation
                .map(|s| s.to_string())
                .unwrap_or_else(|| "parameter".to_string()),
            reason: message.to_string(),
        }
        .into(),

        FfiErrorKind::RuntimeError => {
            // Check if this looks like a spawn failure
            if message.contains("spawn") || message.contains("Spawn") {
                OperationError::SpawnFailed {
                    blueprint: extract_blueprint_from_message(message)
                        .unwrap_or_else(|| "unknown".to_string()),
                    transform: "unknown".to_string(),
                    reason: message.to_string(),
                    source: None,
                }
                .into()
            } else {
                OperationError::SimulationError {
                    message: message.to_string(),
                    source: None,
                }
                .into()
            }
        }

        FfiErrorKind::OutOfRange => ValidationError::OutOfBounds {
            field: operation
                .map(|s| s.to_string())
                .unwrap_or_else(|| "index".to_string()),
            value: "unknown".to_string(),
            min: "0".to_string(),
            max: "unknown".to_string(),
        }
        .into(),

        FfiErrorKind::Unknown => InternalError::FfiError {
            message: format!(
                "Unknown C++ exception{}: {}",
                operation
                    .map(|op| format!(" in {}", op))
                    .unwrap_or_default(),
                message
            ),
            source: None,
        }
        .into(),
    }
}

/// Extract timeout duration from error message.
///
/// Attempts to parse duration from messages like "timeout after 5 seconds".
fn extract_timeout_duration(message: &str) -> Option<Duration> {
    // Try to find patterns like "5 seconds", "5s", "5.0 seconds", etc.
    let lower = message.to_lowercase();

    // Pattern: "N seconds"
    if let Some(idx) = lower.find(" second") {
        if let Some(start) = lower[..idx].rfind(char::is_whitespace) {
            if let Ok(secs) = lower[start + 1..idx].trim().parse::<f64>() {
                return Some(Duration::from_secs_f64(secs));
            }
        }
    }

    // Pattern: "Ns"
    if let Some(idx) = lower.find("s ") {
        if let Some(start) = lower[..idx].rfind(char::is_whitespace) {
            if let Ok(secs) = lower[start + 1..idx].trim().parse::<u64>() {
                return Some(Duration::from_secs(secs));
            }
        }
    }

    None
}

/// Extract resource type and identifier from error message.
///
/// Attempts to identify what type of resource was not found.
fn extract_resource_info(message: &str) -> (ResourceType, String) {
    let lower = message.to_lowercase();

    // Check for resource type keywords
    let resource_type = if lower.contains("blueprint") {
        ResourceType::Blueprint
    } else if lower.contains("actor") {
        ResourceType::Actor
    } else if lower.contains("map") {
        ResourceType::Map
    } else if lower.contains("sensor") {
        ResourceType::Sensor
    } else if lower.contains("waypoint") {
        ResourceType::Waypoint
    } else if lower.contains("traffic light") || lower.contains("traffic_light") {
        ResourceType::TrafficLight
    } else {
        ResourceType::Actor // default
    };

    // Try to extract identifier (text after "'" or "\"")
    let identifier = if let Some(start) = message.find('\'') {
        if let Some(end) = message[start + 1..].find('\'') {
            message[start + 1..start + 1 + end].to_string()
        } else {
            "unknown".to_string()
        }
    } else if let Some(start) = message.find('"') {
        if let Some(end) = message[start + 1..].find('"') {
            message[start + 1..start + 1 + end].to_string()
        } else {
            "unknown".to_string()
        }
    } else {
        "unknown".to_string()
    };

    (resource_type, identifier)
}

/// Extract blueprint ID from error message.
fn extract_blueprint_from_message(message: &str) -> Option<String> {
    // Look for blueprint ID patterns (e.g., "vehicle.tesla.model3")
    for word in message.split_whitespace() {
        if word.contains('.') && (word.starts_with("vehicle") || word.starts_with("walker")) {
            return Some(
                word.trim_matches(|c: char| !c.is_alphanumeric() && c != '.')
                    .to_string(),
            );
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ffi_error_kind_conversion() {
        assert_eq!(FfiErrorKind::from(0), FfiErrorKind::Success);
        assert_eq!(FfiErrorKind::from(1), FfiErrorKind::Timeout);
        assert_eq!(FfiErrorKind::from(999), FfiErrorKind::Unknown);
    }

    #[test]
    fn test_timeout_duration_extraction() {
        assert_eq!(
            extract_timeout_duration("Operation timed out after 5 seconds"),
            Some(Duration::from_secs(5))
        );
        assert_eq!(
            extract_timeout_duration("timeout after 10.5 seconds"),
            Some(Duration::from_secs_f64(10.5))
        );
        assert_eq!(extract_timeout_duration("no duration here"), None);
    }

    #[test]
    fn test_resource_info_extraction() {
        let (rtype, id) = extract_resource_info("Blueprint 'vehicle.tesla.model3' not found");
        assert_eq!(rtype, ResourceType::Blueprint);
        assert_eq!(id, "vehicle.tesla.model3");

        let (rtype, id) = extract_resource_info("Actor not found");
        assert_eq!(rtype, ResourceType::Actor);
        assert_eq!(id, "unknown");
    }

    #[test]
    fn test_blueprint_extraction() {
        assert_eq!(
            extract_blueprint_from_message("Failed to spawn vehicle.tesla.model3 at location"),
            Some("vehicle.tesla.model3".to_string())
        );
        assert_eq!(
            extract_blueprint_from_message("walker.pedestrian.0001 not found"),
            Some("walker.pedestrian.0001".to_string())
        );
        assert_eq!(extract_blueprint_from_message("generic error"), None);
    }

    #[test]
    fn test_parse_timeout_error() {
        let error = parse_ffi_error(
            FfiErrorKind::Timeout as i32,
            "Operation timed out after 5 seconds",
            Some("spawn_actor"),
        );

        assert!(error.is_timeout());
        assert!(error.is_connection_error());
        assert!(matches!(
            error,
            CarlaError::Connection(ConnectionError::Timeout { .. })
        ));
    }

    #[test]
    fn test_parse_not_found_error() {
        let error = parse_ffi_error(
            FfiErrorKind::NotFound as i32,
            "Blueprint 'vehicle.tesla.model3' not found",
            Some("find_blueprint"),
        );

        assert!(error.is_not_found());
        assert!(matches!(
            error,
            CarlaError::Resource(ResourceError::NotFound { .. })
        ));
    }

    #[test]
    fn test_parse_spawn_failure() {
        let error = parse_ffi_error(
            FfiErrorKind::RuntimeError as i32,
            "Failed to spawn vehicle.tesla.model3 at location (0, 0, 0)",
            Some("spawn_actor"),
        );

        assert!(matches!(
            error,
            CarlaError::Operation(OperationError::SpawnFailed { .. })
        ));
    }
}
