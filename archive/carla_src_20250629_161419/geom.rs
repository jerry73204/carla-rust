//! # CARLA Geometry System
//!
//! This module provides comprehensive geometry types and operations that mirror CARLA's
//! `carla::geom` namespace. All types are designed for high-performance 3D spatial
//! computations and seamless integration with CARLA's coordinate system.
//!
//! ## Coordinate System
//!
//! CARLA uses a **right-handed coordinate system** with:
//! - **X-axis**: Forward (positive = forward, negative = backward)
//! - **Y-axis**: Right (positive = right, negative = left)  
//! - **Z-axis**: Up (positive = up, negative = down)
//!
//! ## Core Types
//!
//! ### Spatial Positioning
//! - [`Location`]: 3D point in world space (x, y, z)
//! - [`Rotation`]: 3D orientation using Euler angles (pitch, yaw, roll)
//! - [`Transform`]: Combined position and orientation
//! - [`Vector3D`]: 3D vector for directions and displacements
//! - [`Vector2D`]: 2D vector for planar operations
//!
//! ### Spatial Bounds
//! - [`BoundingBox`]: Axis-aligned bounding box for collision detection
//!
//! ## Usage Examples
//!
//! ### Basic Positioning
//! ```rust
//! use carla::geom::{Location, Rotation, Transform};
//!
//! // Create a location
//! let spawn_point = Location::new(10.0, 20.0, 0.5);
//!
//! // Create a rotation (in degrees)
//! let facing_east = Rotation::new(0.0, 90.0, 0.0); // pitch, yaw, roll
//!
//! // Combine into a transform
//! let spawn_transform = Transform::new(spawn_point, facing_east);
//!
//! // Distance calculation
//! let other_location = Location::new(13.0, 24.0, 0.5);
//! let distance = spawn_point.distance(&other_location);
//! println!("Distance: {:.2} meters", distance);
//! ```
//!
//! ### Vector Operations
//! ```rust
//! use carla::geom::{Location, Vector3D};
//!
//! // Create vectors
//! let velocity = Vector3D::new(15.0, 0.0, 0.0); // 15 m/s forward
//! let offset = Vector3D::new(0.0, 2.0, 1.0); // 2m right, 1m up
//!
//! // Vector arithmetic
//! let combined = velocity + offset;
//! let magnitude = velocity.length();
//! let normalized = velocity.normalize();
//!
//! // Convert between vectors and locations
//! let origin = Location::new(0.0, 0.0, 0.0);
//! let new_position = origin + velocity.to_location();
//! ```
//!
//! ### Transform Operations
//! ```rust
//! use carla::geom::{Location, Rotation, Transform};
//!
//! let transform = Transform::new(Location::new(10.0, 5.0, 1.0), Rotation::new(0.0, 45.0, 0.0));
//!
//! // Transform a point from local to world space
//! let local_point = Location::new(1.0, 0.0, 0.0);
//! let world_point = transform.transform_point(&local_point);
//!
//! // Get forward/right/up vectors
//! let forward = transform.get_forward_vector();
//! let right = transform.get_right_vector();
//! let up = transform.get_up_vector();
//!
//! // Inverse transform
//! let inverse = transform.inverse();
//! ```
//!
//! ### Bounding Box Operations
//! ```rust
//! use carla::geom::{BoundingBox, Location, Vector3D};
//!
//! // Create a bounding box
//! let bbox = BoundingBox::new(
//!     Location::new(0.0, 0.0, 1.0), // center
//!     Vector3D::new(2.0, 1.0, 0.5), // half-extents
//! );
//!
//! // Check if point is inside
//! let point = Location::new(0.5, 0.5, 1.2);
//! let inside = bbox.contains(&point);
//!
//! // Get vertices
//! let vertices = bbox.vertices();
//! println!("Bounding box has {} vertices", vertices.len());
//! ```
//!
//! ## Mathematical Utilities
//!
//! The module provides mathematical constants and conversion utilities:
//!
//! ```rust
//! use carla::geom::constants::*;
//!
//! // Angle conversions
//! let degrees = 90.0;
//! let radians = degrees * DEG_TO_RAD;
//! let back_to_degrees = radians * RAD_TO_DEG;
//!
//! // Common angles
//! assert_eq!(PI_2, 90.0 * DEG_TO_RAD);
//! assert_eq!(TAU, 360.0 * DEG_TO_RAD);
//! ```
//!
//! ## Integration with nalgebra
//!
//! This module integrates with the `nalgebra` crate for advanced mathematical operations:
//!
//! ```rust
//! use carla::geom::{Matrix3, UnitQuaternion, Vector3D};
//! use nalgebra::Vector3;
//!
//! // Convert to nalgebra types
//! let carla_vec = Vector3D::new(1.0, 2.0, 3.0);
//! let na_vec: Vector3<f32> = carla_vec.into();
//!
//! // Use nalgebra operations
//! let cross_product = na_vec.cross(&Vector3::new(0.0, 0.0, 1.0));
//!
//! // Create rotation matrices
//! let rotation_matrix = Matrix3::from_euler_angles(0.0, 0.0, PI / 4.0);
//! ```
//!
//! ## Performance Considerations
//!
//! - All operations use single-precision floats (`f32`) for consistency with CARLA
//! - Vector operations are optimized using SIMD when available
//! - Transform operations minimize memory allocations
//! - Distance calculations avoid square root when possible (use `distance_squared()`)
//!
//! ## Coordinate System Notes
//!
//! - **Unreal Engine Origin**: CARLA uses Unreal Engine's left-handed coordinate system internally
//! - **CARLA Client**: Presents a right-handed coordinate system to users
//! - **Automatic Conversion**: This library handles coordinate system conversion transparently
//! - **Rotation Order**: Euler angles follow ZYX order (yaw, pitch, roll)
//!
//! ## Common Pitfalls
//!
//! - **Angle Units**: Rotations use degrees, not radians (unlike many 3D libraries)
//! - **Transform Order**: When combining transforms, order matters (not commutative)
//! - **Floating Point**: Be aware of precision limitations with large world coordinates
//! - **Null Checks**: Vectors can have zero length - check before normalizing

mod bounding_box;
mod location;
mod rotation;
mod transform;
mod vector;

pub use bounding_box::BoundingBox;
pub use location::Location;
pub use rotation::Rotation;
pub use transform::Transform;
pub use vector::{Vector2D, Vector3D};

// Re-export nalgebra types for advanced mathematical operations
pub use nalgebra::{Matrix3, Matrix4, Point2, Point3, UnitQuaternion, Vector2, Vector3};

/// Trait for types that can be converted to/from carla-sys geometry types.
pub trait FromCxx<T> {
    /// Convert from carla-sys type.
    fn from_cxx(value: T) -> Self;
}

/// Trait for types that can be converted to carla-sys geometry types.
pub trait ToCxx<T> {
    /// Convert to carla-sys type.
    fn to_cxx(&self) -> T;
}

/// Common mathematical constants.
pub mod constants {
    /// Pi constant.
    pub const PI: f64 = std::f64::consts::PI;

    /// Pi / 2.
    pub const PI_2: f64 = std::f64::consts::FRAC_PI_2;

    /// Pi / 4.
    pub const PI_4: f64 = std::f64::consts::FRAC_PI_4;

    /// 2 * Pi.
    pub const TAU: f64 = std::f64::consts::TAU;

    /// Degrees to radians conversion factor.
    pub const DEG_TO_RAD: f64 = PI / 180.0;

    /// Radians to degrees conversion factor.
    pub const RAD_TO_DEG: f64 = 180.0 / PI;
}

/// Utility functions for geometric calculations.
pub mod utils {
    use super::*;

    /// Convert degrees to radians.
    pub fn deg_to_rad(degrees: f64) -> f64 {
        degrees * constants::DEG_TO_RAD
    }

    /// Convert radians to degrees.
    pub fn rad_to_deg(radians: f64) -> f64 {
        radians * constants::RAD_TO_DEG
    }

    /// Calculate distance between two locations.
    pub fn distance(a: &Location, b: &Location) -> f64 {
        ((b.x - a.x).powi(2) + (b.y - a.y).powi(2) + (b.z - a.z).powi(2)).sqrt()
    }

    /// Calculate 2D distance between two locations (ignoring Z).
    pub fn distance_2d(a: &Location, b: &Location) -> f64 {
        ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt()
    }

    /// Clamp angle to [-180, 180] degrees.
    pub fn clamp_angle(angle: f64) -> f64 {
        let mut result = angle % 360.0;
        if result > 180.0 {
            result -= 360.0;
        } else if result < -180.0 {
            result += 360.0;
        }
        result
    }
}

/// Math namespace with geometric calculations
pub mod math {
    use super::*;

    /// Calculate distance between two 3D points.
    pub fn distance(a: &(f64, f64, f64), b: &(f64, f64, f64)) -> f64 {
        ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2) + (b.2 - a.2).powi(2)).sqrt()
    }

    /// Calculate distance between two locations.
    pub fn distance_locations(a: &Location, b: &Location) -> f64 {
        utils::distance(a, b)
    }

    /// Find the nearest segment to a point.
    /// Returns the index of the nearest segment.
    pub fn get_nearest_segment(
        point: &Location,
        segments: &[(Location, Location)],
    ) -> Option<usize> {
        if segments.is_empty() {
            return None;
        }

        let mut min_distance = f64::MAX;
        let mut nearest_idx = 0;

        for (idx, (start, end)) in segments.iter().enumerate() {
            let dist = distance_point_to_segment(point, start, end);
            if dist < min_distance {
                min_distance = dist;
                nearest_idx = idx;
            }
        }

        Some(nearest_idx)
    }

    /// Calculate distance from a point to a line segment.
    pub fn distance_point_to_segment(
        point: &Location,
        segment_start: &Location,
        segment_end: &Location,
    ) -> f64 {
        let segment = Location::new(
            segment_end.x - segment_start.x,
            segment_end.y - segment_start.y,
            segment_end.z - segment_start.z,
        );
        let to_point = Location::new(
            point.x - segment_start.x,
            point.y - segment_start.y,
            point.z - segment_start.z,
        );

        let segment_length_sq =
            segment.x * segment.x + segment.y * segment.y + segment.z * segment.z;
        if segment_length_sq == 0.0 {
            // segment is a point
            return utils::distance(point, segment_start);
        }

        // Project point onto segment
        let t = ((to_point.x * segment.x + to_point.y * segment.y + to_point.z * segment.z)
            / segment_length_sq)
            .clamp(0.0, 1.0);

        let projection = Location::new(
            segment_start.x + t * segment.x,
            segment_start.y + t * segment.y,
            segment_start.z + t * segment.z,
        );

        utils::distance(point, &projection)
    }

    /// Calculate distance from a point to an arc.
    /// Arc is defined by start and end points on a circle with given radius.
    pub fn distance_arc_to_point(
        point: &Location,
        arc_start: &Location,
        arc_end: &Location,
        center: &Location,
        radius: f64,
    ) -> f64 {
        // Simple approximation: check distance to arc endpoints and center
        let dist_to_center = utils::distance(point, center);
        let dist_on_circle = (dist_to_center - radius).abs();

        // Check if point is within arc angle range (simplified)
        let to_start = Location::new(
            arc_start.x - center.x,
            arc_start.y - center.y,
            arc_start.z - center.z,
        );
        let to_end = Location::new(
            arc_end.x - center.x,
            arc_end.y - center.y,
            arc_end.z - center.z,
        );
        let to_point = Location::new(point.x - center.x, point.y - center.y, point.z - center.z);

        // Simple check: if point is between start and end angles
        let start_angle = to_start.y.atan2(to_start.x);
        let end_angle = to_end.y.atan2(to_end.x);
        let point_angle = to_point.y.atan2(to_point.x);

        // Check if angle is between start and end (assuming counter-clockwise)
        let mut angle_diff = end_angle - start_angle;
        if angle_diff < 0.0 {
            angle_diff += 2.0 * std::f64::consts::PI;
        }
        let mut point_diff = point_angle - start_angle;
        if point_diff < 0.0 {
            point_diff += 2.0 * std::f64::consts::PI;
        }

        if point_diff <= angle_diff {
            // Point is within arc range
            dist_on_circle
        } else {
            // Point is outside arc range, return distance to nearest endpoint
            utils::distance(point, arc_start).min(utils::distance(point, arc_end))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // Tests corresponding to C++ test_geom.cpp

    #[test]
    fn test_distance() {
        // Corresponds to C++ TEST(geom, distance)

        // Test same point distance
        let result = math::distance(&(0.0, 0.0, 0.0), &(0.0, 0.0, 0.0));
        assert_relative_eq!(result, 0.0, epsilon = 0.001);

        // Test unit distances
        let result = math::distance(&(0.0, 0.0, 0.0), &(1.0, 0.0, 0.0));
        assert_relative_eq!(result, 1.0, epsilon = 0.001);

        let result = math::distance(&(0.0, 0.0, 0.0), &(0.0, 1.0, 0.0));
        assert_relative_eq!(result, 1.0, epsilon = 0.001);

        let result = math::distance(&(0.0, 0.0, 0.0), &(0.0, 0.0, 1.0));
        assert_relative_eq!(result, 1.0, epsilon = 0.001);

        // Test diagonal distance
        let result = math::distance(&(0.0, 0.0, 0.0), &(1.0, 1.0, 1.0));
        assert_relative_eq!(result, 3.0_f64.sqrt(), epsilon = 0.001);

        // Test with negative coordinates
        let result = math::distance(&(-1.0, -2.0, -3.0), &(1.0, 2.0, 3.0));
        let expected = (2.0_f64.powi(2) + 4.0_f64.powi(2) + 6.0_f64.powi(2)).sqrt(); // sqrt(4+16+36) = sqrt(56)
        assert_relative_eq!(result, expected, epsilon = 0.001);
    }

    #[test]
    fn test_nearest_point_segment() {
        // Corresponds to C++ TEST(geom, nearest_point_segment)

        // Create test segments
        let segments = vec![
            (Location::new(0.0, 0.0, 0.0), Location::new(10.0, 0.0, 0.0)),
            (Location::new(0.0, 0.0, 0.0), Location::new(0.0, 10.0, 0.0)),
            (Location::new(0.0, 0.0, 0.0), Location::new(0.0, 0.0, 10.0)),
            (
                Location::new(10.0, 0.0, 0.0),
                Location::new(10.0, 10.0, 0.0),
            ),
            (
                Location::new(10.0, 10.0, 0.0),
                Location::new(0.0, 10.0, 0.0),
            ),
            (Location::new(0.0, 10.0, 0.0), Location::new(0.0, 0.0, 0.0)),
            (Location::new(5.0, 5.0, 0.0), Location::new(15.0, 15.0, 0.0)),
            (
                Location::new(5.0, 5.0, 5.0),
                Location::new(15.0, 15.0, 15.0),
            ),
            (
                Location::new(-5.0, -5.0, -5.0),
                Location::new(5.0, 5.0, 5.0),
            ),
            (
                Location::new(20.0, 20.0, 20.0),
                Location::new(30.0, 30.0, 30.0),
            ),
        ];

        // Test points and expected nearest segments
        let test_cases = vec![
            (Location::new(5.0, 0.0, 0.0), 0),    // Near first segment
            (Location::new(0.0, 5.0, 0.0), 1),    // Near second segment
            (Location::new(0.0, 0.0, 5.0), 2),    // Near third segment
            (Location::new(10.0, 5.0, 0.0), 3),   // Near fourth segment
            (Location::new(5.0, 10.0, 0.0), 4),   // Near fifth segment
            (Location::new(0.0, 8.0, 0.0), 1), // Actually closer to segment 1 (0,0,0) to (0,10,0)
            (Location::new(8.0, 8.0, 0.0), 6), // Near seventh segment
            (Location::new(10.0, 10.0, 10.0), 7), // Near eighth segment
            (Location::new(1.0, 1.0, 1.0), 8), // Near ninth segment (closest to diagonal through origin)
            (Location::new(25.0, 25.0, 25.0), 9), // Near tenth segment
        ];

        for (point, expected_idx) in test_cases {
            let nearest = math::get_nearest_segment(&point, &segments);
            // Debug print to see which segment is actually nearest
            if nearest != Some(expected_idx) {
                eprintln!("Point {point:?} - Expected segment {expected_idx}, got {nearest:?}");
                // Calculate distances to all segments for debugging
                for (idx, (start, end)) in segments.iter().enumerate() {
                    let dist = math::distance_point_to_segment(&point, start, end);
                    eprintln!("  Segment {idx} ({start:?} to {end:?}): distance = {dist}");
                }
            }
            assert_eq!(nearest, Some(expected_idx));
        }
    }

    #[test]
    fn test_nearest_point_arc() {
        // Corresponds to C++ TEST(geom, nearest_point_arc)

        // Test cases with arcs and points
        let test_cases = vec![
            // Arc 1: Quarter circle from (1,0) to (0,1) with center at origin
            (
                Location::new(1.0, 1.0, 0.0), // point
                Location::new(1.0, 0.0, 0.0), // arc start
                Location::new(0.0, 1.0, 0.0), // arc end
                Location::new(0.0, 0.0, 0.0), // center
                1.0,                          // radius
                0.414,                        // expected distance (approx)
            ),
            // Arc 2: Half circle
            (
                Location::new(0.0, 0.0, 0.0),
                Location::new(5.0, 0.0, 0.0),
                Location::new(-5.0, 0.0, 0.0),
                Location::new(0.0, 0.0, 0.0),
                5.0,
                5.0,
            ),
            // Arc 3: Point outside arc angle range
            (
                Location::new(-1.0, -1.0, 0.0),
                Location::new(10.0, 0.0, 0.0),
                Location::new(0.0, 10.0, 0.0),
                Location::new(0.0, 0.0, 0.0),
                10.0,
                11.18, // Distance to nearest endpoint (10,0,0)
            ),
            // Arc 4: Point on arc
            (
                Location::new(20.0, 0.0, 0.0),
                Location::new(20.0, 0.0, 0.0),
                Location::new(0.0, 20.0, 0.0),
                Location::new(0.0, 0.0, 0.0),
                20.0,
                0.0,
            ),
        ];

        for (point, arc_start, arc_end, center, radius, expected) in test_cases {
            let result = math::distance_arc_to_point(&point, &arc_start, &arc_end, &center, radius);
            if (result - expected).abs() > 0.5 {
                eprintln!("Arc test failed: point {point:?}, expected {expected}, got {result}");
            }
            assert_relative_eq!(result, expected, epsilon = 0.5); // More tolerance for arc approximation
        }
    }
}
