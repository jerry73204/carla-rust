//! Geometry types and utilities.

use carla_sys::carla_rust::geom::FfiBoundingBox as NativeBoundingBox;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector2, Vector3};
use static_assertions::assert_impl_all;

pub use carla_sys::{
    carla::geom::{GeoLocation, Rotation, Vector2D, Vector3D},
    carla_rust::geom::{FfiLocation as Location, FfiTransform as Transform},
};

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
        }
    }

    fn to_na(&self) -> Vector2<f32> {
        let Self { x, y } = *self;
        Vector2::new(x, y)
    }
}

/// Extension trait for [Vector3D].
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
}

/// A bounding box geometry object converted from CARLA native
/// bounding box type.
#[derive(Debug, Clone)]
pub struct BoundingBox<T> {
    /// The pose of the center point.
    pub transform: Isometry3<T>,
    /// The lengths of box sides.
    pub extent: Vector3<T>,
}

impl BoundingBox<f32> {
    /// Convert to a CARLA's native C++ bounding box.
    pub fn to_native(&self) -> NativeBoundingBox {
        let Self { transform, extent } = self;
        NativeBoundingBox {
            location: Location::from_na_translation(&transform.translation),
            rotation: Rotation::from_na(&transform.rotation),
            extent: Vector3D::from_na(extent),
        }
    }

    /// Create from a CARLA's native C++ bounding box.
    pub fn from_native(bbox: &NativeBoundingBox) -> Self {
        let NativeBoundingBox {
            location,
            extent,
            rotation,
        } = bbox;
        Self {
            transform: Isometry3 {
                rotation: rotation.to_na(),
                translation: location.to_na_translation(),
            },
            extent: extent.to_na(),
        }
    }

    pub fn contains(
        &self,
        in_world_point: &Translation3<f32>,
        in_bbox_to_world_transform: &Isometry3<f32>,
    ) -> bool {
        self.to_native().Contains(
            &Location::from_na_translation(in_world_point),
            &Transform::from_na(in_bbox_to_world_transform),
        )
    }

    pub fn local_vertices(&self) -> Vec<Translation3<f32>> {
        self.to_native()
            .GetLocalVertices()
            .iter()
            .map(|loc| loc.to_na_translation())
            .collect()
    }

    pub fn world_vertices(&self, in_bbox_to_world_tr: &Isometry3<f32>) -> Vec<Translation3<f32>> {
        self.to_native()
            .GetWorldVertices(&Transform::from_na(in_bbox_to_world_tr))
            .iter()
            .map(|loc| loc.to_na_translation())
            .collect()
    }
}

assert_impl_all!(Vector2D: Send, Sync);
assert_impl_all!(Vector3D: Send, Sync);
assert_impl_all!(Location: Send, Sync);
assert_impl_all!(Rotation: Send, Sync);
assert_impl_all!(Transform: Send, Sync);
assert_impl_all!(NativeBoundingBox: Send, Sync);
