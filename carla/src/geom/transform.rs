//! Transform, Location, and Rotation geometry types.

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

// Re-export C types with Rust-friendly names
pub use carla_sys::{
    carla_rotation_t as Rotation, carla_transform_t as Transform, carla_vector3d_t as Location,
};

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
