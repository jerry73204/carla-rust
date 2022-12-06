use carla_sys::carla_rust::geom::FfiBoundingBox as NativeBoundingBox;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector2, Vector3};
use static_assertions::assert_impl_all;

pub use carla_sys::{
    carla::geom::{GeoLocation, Rotation, Vector2D, Vector3D},
    carla_rust::geom::{FfiLocation as Location, FfiTransform as Transform},
};

pub trait Vector2DExt {
    fn from_na(from: &Vector2<f32>) -> Self;
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

pub trait Vector3DExt {
    fn from_na(from: &Vector3<f32>) -> Self;
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

pub trait LocationExt {
    fn from_na(from: &Translation3<f32>) -> Self;
    fn to_na(&self) -> Translation3<f32>;
}

impl LocationExt for Location {
    fn from_na(from: &Translation3<f32>) -> Self {
        Self {
            x: from.x,
            y: from.y,
            z: from.z,
        }
    }

    fn to_na(&self) -> Translation3<f32> {
        let Self { x, y, z } = *self;
        Translation3::new(x, y, z)
    }
}

pub trait RotationExt {
    fn from_na(from: &UnitQuaternion<f32>) -> Self;
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

pub trait TransformExt {
    fn from_na(pose: &Isometry3<f32>) -> Self;
    fn to_na(&self) -> Isometry3<f32>;
}

impl TransformExt for Transform {
    fn from_na(pose: &Isometry3<f32>) -> Self {
        let Isometry3 {
            rotation,
            translation,
        } = pose;
        let location = Location::from_na(translation);
        let rotation = Rotation::from_na(rotation);
        Self { location, rotation }
    }

    fn to_na(&self) -> Isometry3<f32> {
        Isometry3 {
            rotation: self.rotation.to_na(),
            translation: self.location.to_na(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct BoundingBox<T> {
    pub transform: Isometry3<T>,
    pub extent: Vector3<T>,
}

impl BoundingBox<f32> {
    pub fn to_native(&self) -> NativeBoundingBox {
        let Self { transform, extent } = self;
        NativeBoundingBox {
            location: Location::from_na(&transform.translation),
            rotation: Rotation::from_na(&transform.rotation),
            extent: Vector3D::from_na(extent),
        }
    }

    pub fn from_native(bbox: &NativeBoundingBox) -> Self {
        let NativeBoundingBox {
            location,
            extent,
            rotation,
        } = bbox;
        Self {
            transform: Isometry3 {
                rotation: rotation.to_na(),
                translation: location.to_na(),
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
            &Location::from_na(in_world_point),
            &Transform::from_na(in_bbox_to_world_transform),
        )
    }

    pub fn local_vertices(&self) -> Vec<Translation3<f32>> {
        self.to_native()
            .GetLocalVertices()
            .iter()
            .map(|loc| loc.to_na())
            .collect()
    }

    pub fn world_vertices(&self, in_bbox_to_world_tr: &Isometry3<f32>) -> Vec<Translation3<f32>> {
        self.to_native()
            .GetWorldVertices(&Transform::from_na(in_bbox_to_world_tr))
            .iter()
            .map(|loc| loc.to_na())
            .collect()
    }
}

assert_impl_all!(Vector2D: Send, Sync);
assert_impl_all!(Vector3D: Send, Sync);
assert_impl_all!(Location: Send, Sync);
assert_impl_all!(Rotation: Send, Sync);
assert_impl_all!(Transform: Send, Sync);
assert_impl_all!(NativeBoundingBox: Send, Sync);
