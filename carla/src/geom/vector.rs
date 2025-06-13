//! Vector2D and Vector3D geometry types.

use nalgebra::{Vector2, Vector3};

// Re-export Location as Vector3D for compatibility
pub use super::transform::Location as Vector3D;

// For compatibility with existing code that expects these types
pub type Vector2D = super::transform::Location; // Reuse Location for now, need proper 2D vector type

/// Extension trait for [Vector2D].
pub trait Vector2DExt {
    /// Create from a nalgebra vector.
    fn from_na(from: &Vector2<f32>) -> Self;
    /// Convert to a nalgebra vector.
    fn to_na(&self) -> Vector2<f32>;
}

impl Vector2DExt for Vector2D {
    fn from_na(from: &Vector2<f32>) -> Self {
        Self {
            x: from.x,
            y: from.y,
            z: 0.0, // Use z=0 for 2D vectors stored as 3D
        }
    }

    fn to_na(&self) -> Vector2<f32> {
        let Self { x, y, .. } = *self;
        Vector2::new(x, y)
    }
}

/// Extension trait for [Vector3D] (alias for Location).
pub trait Vector3DExt {
    /// Create from a nalgebra vector.
    fn from_na(from: &Vector3<f32>) -> Self;
    /// Convert to a nalgebra vector.
    fn to_na(&self) -> Vector3<f32>;
    /// Create from a C vector (identity operation since it's already a C struct).
    fn from_c_vector(vector: carla_sys::carla_vector3d_t) -> Self;
}

impl Vector3DExt for Vector3D {
    fn from_na(from: &Vector3<f32>) -> Self {
        Self {
            x: from.x,
            y: from.y,
            z: from.z,
        }
    }

    fn to_na(&self) -> Vector3<f32> {
        let Self { x, y, z } = *self;
        Vector3::new(x, y, z)
    }

    fn from_c_vector(vector: carla_sys::carla_vector3d_t) -> Self {
        vector
    }
}
