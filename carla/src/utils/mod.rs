//! Utility functions and helpers for the CARLA Rust client.

/// Convert degrees to radians.
pub fn deg_to_rad(degrees: f64) -> f64 {
    degrees * std::f64::consts::PI / 180.0
}

/// Convert radians to degrees.
pub fn rad_to_deg(radians: f64) -> f64 {
    radians * 180.0 / std::f64::consts::PI
}

/// Clamp a value between min and max.
pub fn clamp<T: PartialOrd>(value: T, min: T, max: T) -> T {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

/// Type conversion helpers for CARLA types.
pub mod conversions {
    use crate::geom::{Location, Rotation, Transform, Vector3D};

    /// Convert a tuple of (x, y, z) to a Vector3D.
    pub fn tuple_to_vector3d(tuple: (f32, f32, f32)) -> Vector3D {
        Vector3D::new(tuple.0, tuple.1, tuple.2)
    }

    /// Convert a Vector3D to a tuple of (x, y, z).
    pub fn vector3d_to_tuple(vector: &Vector3D) -> (f32, f32, f32) {
        (vector.x, vector.y, vector.z)
    }

    /// Convert a tuple of (x, y, z) to a Location.
    pub fn tuple_to_location(tuple: (f64, f64, f64)) -> Location {
        Location::new(tuple.0, tuple.1, tuple.2)
    }

    /// Convert a Location to a tuple of (x, y, z).
    pub fn location_to_tuple(location: &Location) -> (f64, f64, f64) {
        (location.x, location.y, location.z)
    }

    /// Convert a tuple of (pitch, yaw, roll) to a Rotation.
    pub fn tuple_to_rotation(tuple: (f32, f32, f32)) -> Rotation {
        Rotation::new(tuple.0, tuple.1, tuple.2)
    }

    /// Convert a Rotation to a tuple of (pitch, yaw, roll).
    pub fn rotation_to_tuple(rotation: &Rotation) -> (f32, f32, f32) {
        (rotation.pitch, rotation.yaw, rotation.roll)
    }

    /// Create a Transform from location and rotation tuples.
    pub fn tuples_to_transform(location: (f64, f64, f64), rotation: (f32, f32, f32)) -> Transform {
        Transform::new(tuple_to_location(location), tuple_to_rotation(rotation))
    }
}

/// Error handling utilities.
pub mod error {
    use crate::error::CarlaResult;

    /// Retry a function up to a specified number of times.
    pub fn retry<F, T, E>(mut f: F, max_attempts: usize) -> Result<T, E>
    where
        F: FnMut() -> Result<T, E>,
    {
        let mut last_error = None;

        for _ in 0..max_attempts {
            match f() {
                Ok(result) => return Ok(result),
                Err(e) => last_error = Some(e),
            }
        }

        Err(last_error.unwrap())
    }

    /// Retry a function with exponential backoff.
    pub fn retry_with_backoff<F, T, E>(
        mut f: F,
        max_attempts: usize,
        initial_delay_ms: u64,
    ) -> Result<T, E>
    where
        F: FnMut() -> Result<T, E>,
    {
        let mut delay = initial_delay_ms;
        let mut last_error = None;

        for attempt in 0..max_attempts {
            match f() {
                Ok(result) => return Ok(result),
                Err(e) => {
                    last_error = Some(e);
                    if attempt < max_attempts - 1 {
                        std::thread::sleep(std::time::Duration::from_millis(delay));
                        delay *= 2; // Exponential backoff
                    }
                }
            }
        }

        Err(last_error.unwrap())
    }

    /// Convert an Option to a CarlaResult with a custom error message.
    pub fn option_to_result<T>(option: Option<T>, error_msg: &str) -> CarlaResult<T> {
        option.ok_or_else(|| {
            crate::error::CarlaError::Client(crate::error::ClientError::InvalidHost(
                error_msg.to_string(),
            ))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_angle_conversions() {
        let degrees = 180.0;
        let radians = deg_to_rad(degrees);
        assert!((radians - std::f64::consts::PI).abs() < f64::EPSILON);

        let back_to_degrees = rad_to_deg(radians);
        assert!((back_to_degrees - degrees).abs() < f64::EPSILON);
    }

    #[test]
    fn test_clamp() {
        assert_eq!(clamp(5, 0, 10), 5);
        assert_eq!(clamp(-1, 0, 10), 0);
        assert_eq!(clamp(15, 0, 10), 10);
    }

    #[test]
    fn test_conversions() {
        use conversions::*;

        let vector_tuple = (1.0f32, 2.0f32, 3.0f32);
        let vector = tuple_to_vector3d(vector_tuple);
        let back_to_tuple = vector3d_to_tuple(&vector);
        assert_eq!(vector_tuple, back_to_tuple);

        let location_tuple = (1.0f64, 2.0f64, 3.0f64);
        let location = tuple_to_location(location_tuple);
        let back_to_tuple = location_to_tuple(&location);
        assert_eq!(location_tuple, back_to_tuple);
    }

    #[test]
    fn test_error_retry() {
        use error::retry;

        let mut attempt_count = 0;
        let result = retry(
            || {
                attempt_count += 1;
                if attempt_count < 3 {
                    Err("failed")
                } else {
                    Ok("success")
                }
            },
            5,
        );

        assert_eq!(result, Ok("success"));
        assert_eq!(attempt_count, 3);
    }

    #[test]
    fn test_error_retry_failure() {
        use error::retry;

        let mut attempt_count = 0;
        let result: Result<&str, &str> = retry(
            || {
                attempt_count += 1;
                Err("always fails")
            },
            3,
        );

        assert_eq!(result, Err("always fails"));
        assert_eq!(attempt_count, 3);
    }
}
