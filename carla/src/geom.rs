//! Geometry types and utilities.

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector2, Vector3};
use static_assertions::assert_impl_all;

// Re-export C types with Rust-friendly names
pub use carla_sys::{
    carla_rotation_t as Rotation, carla_transform_t as Transform, carla_vector3d_t as Location,
};

// For compatibility with existing code that expects these types
pub type Vector3D = Location;
pub type Vector2D = Location; // Reuse Location for now, need proper 2D vector type
pub type BoundingBox = carla_sys::carla_transform_t; // Placeholder until we have proper bounding box

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
}

// Location and Vector3D are the same type, so we don't need a separate implementation

/// Extension trait for [Location].
pub trait LocationExt {
    /// Create from a nalgebra translation object.
    fn from_na_translation(from: &Translation3<f32>) -> Self;
    /// Create from a nalgebra point.
    fn from_na_point(from: &Point3<f32>) -> Self;
    /// Convert to a nalgebra translation object.
    fn to_na_translation(&self) -> Translation3<f32>;
    /// Convert to a nalgebra point.
    fn to_na_point(&self) -> Point3<f32>;
}

impl LocationExt for Location {
    fn from_na_translation(from: &Translation3<f32>) -> Self {
        Self {
            x: from.x,
            y: from.y,
            z: from.z,
        }
    }

    fn from_na_point(from: &Point3<f32>) -> Self {
        Self {
            x: from.x,
            y: from.y,
            z: from.z,
        }
    }

    fn to_na_translation(&self) -> Translation3<f32> {
        let Self { x, y, z } = *self;
        Translation3::new(x, y, z)
    }

    fn to_na_point(&self) -> Point3<f32> {
        let Self { x, y, z } = *self;
        Point3::new(x, y, z)
    }
}

/// Extension trait for [Rotation].
pub trait RotationExt {
    /// Create from a nalgebra quaternion vector.
    fn from_na(from: &UnitQuaternion<f32>) -> Self;
    /// Convert to a nalgebra quaternion vector.
    fn to_na(&self) -> UnitQuaternion<f32>;
}

impl RotationExt for Rotation {
    fn from_na(from: &UnitQuaternion<f32>) -> Self {
        let (roll, pitch, yaw) = from.euler_angles();
        Self {
            roll: roll.to_degrees(),
            pitch: pitch.to_degrees(),
            yaw: yaw.to_degrees(),
        }
    }

    fn to_na(&self) -> UnitQuaternion<f32> {
        let Self { roll, pitch, yaw } = *self;
        UnitQuaternion::from_euler_angles(roll.to_radians(), pitch.to_radians(), yaw.to_radians())
    }
}

/// Extension trait for [Transform].
pub trait TransformExt {
    /// Create from a nalgebra isometry object.
    fn from_na(pose: &Isometry3<f32>) -> Self;
    /// Convert to a nalgebra isometry object.
    fn to_na(&self) -> Isometry3<f32>;
    /// Convert to C transform (identity operation since it's already a C struct).
    fn to_c_transform(&self) -> Self;
    /// Create from C transform (identity operation since it's already a C struct).
    fn from_c_transform(transform: Self) -> Self;
}

impl TransformExt for Transform {
    fn from_na(pose: &Isometry3<f32>) -> Self {
        let Isometry3 {
            rotation,
            translation,
        } = pose;
        let location = Location::from_na_translation(translation);
        let rotation = Rotation::from_na(rotation);
        Self { location, rotation }
    }

    fn to_na(&self) -> Isometry3<f32> {
        Isometry3 {
            rotation: self.rotation.to_na(),
            translation: self.location.to_na_translation(),
        }
    }

    fn to_c_transform(&self) -> Self {
        *self
    }

    fn from_c_transform(transform: Self) -> Self {
        transform
    }
}

/// A nalgebra-based bounding box geometry object that provides
/// convenient mathematical operations on CARLA's native bounding box.
#[derive(Debug, Clone)]
pub struct NalgebraBoundingBox<T> {
    /// The pose of the center point.
    pub transform: Isometry3<T>,
    /// The lengths of box sides.
    pub extent: Vector3<T>,
}

impl NalgebraBoundingBox<f32> {
    /// Convert to a CARLA's native C transform.
    /// Note: BoundingBox is currently a placeholder using Transform.
    /// This will be updated when proper bounding box C struct is available.
    pub fn to_native(&self) -> BoundingBox {
        let Self { transform, .. } = self;
        BoundingBox {
            location: Location::from_na_translation(&transform.translation),
            rotation: Rotation::from_na(&transform.rotation),
        }
    }

    /// Create from a CARLA's native C transform.
    /// Note: BoundingBox is currently a placeholder using Transform.
    /// This will be updated when proper bounding box C struct is available.
    pub fn from_native(bbox: &BoundingBox) -> Self {
        let BoundingBox { location, rotation } = bbox;
        Self {
            transform: Isometry3 {
                rotation: rotation.to_na(),
                translation: location.to_na_translation(),
            },
            extent: Vector3::new(1.0, 1.0, 1.0), // Default extent
        }
    }
}

/// Extension methods for the native C BoundingBox type (placeholder implementation)
pub trait BoundingBoxExt {
    /// Check if a point is contained within the bounding box
    fn contains_point(&self, point: &Location, bbox_transform: &Transform) -> bool;
}

impl BoundingBoxExt for BoundingBox {
    fn contains_point(&self, _point: &Location, _bbox_transform: &Transform) -> bool {
        // TODO: Implement when proper bounding box C functions are available
        false
    }
}

assert_impl_all!(Vector2D: Send, Sync);
assert_impl_all!(Vector3D: Send, Sync);
assert_impl_all!(Location: Send, Sync);
assert_impl_all!(Rotation: Send, Sync);
assert_impl_all!(Transform: Send, Sync);
assert_impl_all!(BoundingBox: Send, Sync);
assert_impl_all!(NalgebraBoundingBox<f32>: Send, Sync);
