//! Geometry types and operations for CARLA.
//!
//! This module provides Rust equivalents of CARLA's geometry types from `carla::geom`,
//! with additional convenience methods and integration with the `nalgebra` crate.

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

// Re-export nalgebra types for convenience
pub use nalgebra::{Matrix3, Matrix4, Point2, Point3, UnitQuaternion, Vector2, Vector3};

/// Trait for types that can be converted to/from carla-cxx geometry types.
pub trait FromCxx<T> {
    /// Convert from carla-cxx type.
    fn from_cxx(value: T) -> Self;
}

/// Trait for types that can be converted to carla-cxx geometry types.
pub trait ToCxx<T> {
    /// Convert to carla-cxx type.
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
