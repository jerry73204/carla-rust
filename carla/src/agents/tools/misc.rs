//! Utility functions for agent operations.

use crate::{
    client::ActorBase,
    geom::{Location, Transform, Vector3D},
};

#[cfg(test)]
use crate::geom::Rotation;
use std::f32::consts::PI;

/// Calculates the speed of a vehicle in km/h.
///
/// # Arguments
/// * `vehicle` - The vehicle actor
///
/// # Returns
/// Speed in kilometers per hour
///
/// # Examples
/// ```no_run
/// use carla::agents::tools::get_speed;
/// # use carla::client::Vehicle;
/// # fn example(vehicle: &Vehicle) {
/// let speed_kmh = get_speed(vehicle);
/// println!("Speed: {:.1} km/h", speed_kmh);
/// # }
/// ```
pub fn get_speed<T: ActorBase>(vehicle: &T) -> f32 {
    let velocity = vehicle.velocity();
    let speed_ms = velocity.norm(); // m/s
    speed_ms * 3.6 // Convert to km/h
}

/// Computes Euclidean distance between two locations.
///
/// # Arguments
/// * `location_1` - First location
/// * `location_2` - Second location
///
/// # Returns
/// Distance in meters
pub fn compute_distance(location_1: &Location, location_2: &Location) -> f32 {
    let dx = location_1.x - location_2.x;
    let dy = location_1.y - location_2.y;
    let dz = location_1.z - location_2.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Computes 2D Euclidean distance (ignoring Z coordinate).
///
/// # Arguments
/// * `location_1` - First location
/// * `location_2` - Second location
///
/// # Returns
/// 2D distance in meters
pub fn compute_distance_2d(location_1: &Location, location_2: &Location) -> f32 {
    let dx = location_1.x - location_2.x;
    let dy = location_1.y - location_2.y;
    (dx * dx + dy * dy).sqrt()
}

/// Computes distance and relative angle to a target location.
///
/// Returns the Euclidean distance and the angle in radians between the
/// forward vector (determined by orientation) and the vector to the target.
///
/// # Arguments
/// * `target_location` - Target position
/// * `current_location` - Current position
/// * `orientation` - Current orientation (yaw in degrees)
///
/// # Returns
/// Tuple of (distance in meters, angle in radians)
pub fn compute_magnitude_angle(
    target_location: &Location,
    current_location: &Location,
    orientation: f32,
) -> (f32, f32) {
    // Compute distance
    let distance = compute_distance(target_location, current_location);

    // Compute direction vector to target
    let dx = target_location.x - current_location.x;
    let dy = target_location.y - current_location.y;

    // Compute angle
    let target_angle = dy.atan2(dx); // radians
    let orientation_rad = orientation * PI / 180.0;
    let mut angle = target_angle - orientation_rad;

    // Normalize angle to [-π, π]
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }

    (distance, angle)
}

/// Creates a normalized direction vector from one location to another.
///
/// # Arguments
/// * `location_1` - Starting location
/// * `location_2` - Target location
///
/// # Returns
/// Normalized 3D direction vector
pub fn vector(location_1: &Location, location_2: &Location) -> Vector3D {
    let dx = location_2.x - location_1.x;
    let dy = location_2.y - location_1.y;
    let dz = location_2.z - location_1.z;

    let magnitude = (dx * dx + dy * dy + dz * dz).sqrt();

    if magnitude > 0.0001 {
        Vector3D {
            x: dx / magnitude,
            y: dy / magnitude,
            z: dz / magnitude,
        }
    } else {
        Vector3D {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

/// Checks if a target is within a certain distance and angular range.
///
/// # Arguments
/// * `target_transform` - Transform of the target
/// * `reference_transform` - Reference transform (e.g., vehicle's transform)
/// * `max_distance` - Maximum distance in meters
/// * `angle_interval` - Angular range in degrees (symmetric around forward direction)
///
/// # Returns
/// `true` if target is within range, `false` otherwise
pub fn is_within_distance(
    target_transform: &Transform,
    reference_transform: &Transform,
    max_distance: f32,
    angle_interval: Option<f32>,
) -> bool {
    let target_loc = &target_transform.location;
    let reference_loc = &reference_transform.location;

    // Check distance
    let distance = compute_distance(target_loc, reference_loc);
    if distance > max_distance {
        return false;
    }

    // If no angle constraint, return true
    let angle_interval = match angle_interval {
        Some(a) => a,
        None => return true,
    };

    // Check angle
    let (_, angle) =
        compute_magnitude_angle(target_loc, reference_loc, reference_transform.rotation.yaw);

    let angle_deg = angle.abs() * 180.0 / PI;
    angle_deg <= angle_interval
}

/// Computes 2D distance from a waypoint to a vehicle transform.
///
/// This function computes distance in the XY plane only, ignoring Z coordinate.
///
/// # Arguments
/// * `waypoint_location` - Waypoint location
/// * `vehicle_transform` - Vehicle transform
///
/// # Returns
/// 2D distance in meters
pub fn distance_vehicle(waypoint_location: &Location, vehicle_transform: &Transform) -> f32 {
    compute_distance_2d(waypoint_location, &vehicle_transform.location)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_distance() {
        let loc1 = Location {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let loc2 = Location {
            x: 3.0,
            y: 4.0,
            z: 0.0,
        };
        let distance = compute_distance(&loc1, &loc2);
        assert!((distance - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_compute_distance_2d() {
        let loc1 = Location {
            x: 0.0,
            y: 0.0,
            z: 10.0,
        };
        let loc2 = Location {
            x: 3.0,
            y: 4.0,
            z: 20.0,
        };
        let distance = compute_distance_2d(&loc1, &loc2);
        assert!((distance - 5.0).abs() < 0.001); // Z difference ignored
    }

    #[test]
    fn test_vector() {
        let loc1 = Location {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let loc2 = Location {
            x: 3.0,
            y: 4.0,
            z: 0.0,
        };
        let vec = vector(&loc1, &loc2);
        assert!((vec.x - 0.6).abs() < 0.001);
        assert!((vec.y - 0.8).abs() < 0.001);
        assert!((vec.z - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_compute_magnitude_angle() {
        let current = Location {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let target = Location {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };

        // Facing forward (0 degrees = East)
        let (dist, angle) = compute_magnitude_angle(&target, &current, 0.0);
        assert!((dist - 1.0).abs() < 0.001);
        assert!(angle.abs() < 0.001); // Should be aligned

        // Facing backward (180 degrees)
        let (dist, angle) = compute_magnitude_angle(&target, &current, 180.0);
        assert!((dist - 1.0).abs() < 0.001);
        assert!((angle.abs() - PI).abs() < 0.001); // Should be 180 degrees off
    }

    #[test]
    fn test_is_within_distance_only() {
        let target = Transform {
            location: Location {
                x: 5.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };
        let reference = Transform {
            location: Location {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        assert!(is_within_distance(&target, &reference, 10.0, None));
        assert!(!is_within_distance(&target, &reference, 4.0, None));
    }

    #[test]
    fn test_is_within_distance_with_angle() {
        let target = Transform {
            location: Location {
                x: 5.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };
        let reference = Transform {
            location: Location {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        // Target is directly in front, angle should be ~0
        assert!(is_within_distance(&target, &reference, 10.0, Some(30.0)));

        // Reference facing away (180 degrees), target behind
        let reference_back = Transform {
            location: Location {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 180.0,
                roll: 0.0,
            },
        };
        assert!(!is_within_distance(
            &target,
            &reference_back,
            10.0,
            Some(30.0)
        ));
    }

    #[test]
    fn test_distance_vehicle() {
        let waypoint_loc = Location {
            x: 10.0,
            y: 10.0,
            z: 5.0,
        };
        let vehicle_transform = Transform {
            location: Location {
                x: 13.0,
                y: 14.0,
                z: 100.0, // Different Z, should be ignored
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        let distance = distance_vehicle(&waypoint_loc, &vehicle_transform);
        assert!((distance - 5.0).abs() < 0.001); // sqrt(3^2 + 4^2) = 5
    }
}
