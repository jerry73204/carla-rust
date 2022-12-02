use autocxx::prelude::*;
use carla_sys::carla_rust::geom::{FfiLocation, FfiRotation, FfiTransform};
use cxx::UniquePtr;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector2, Vector3};

pub use carla_sys::carla::geom::{Vector2D, Vector3D};

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

#[repr(transparent)]
pub struct Location {
    pub(crate) inner: UniquePtr<FfiLocation>,
}

impl Location {
    pub fn from_xyz(xyz: [f32; 3]) -> Self {
        let [x, y, z] = xyz;
        let ptr = FfiLocation::new1(x, y, z).within_unique_ptr();
        Self::from_cxx(ptr).unwrap()
    }

    pub fn from_cxx(ptr: UniquePtr<FfiLocation>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }

    pub fn from_na(from: &Translation3<f32>) -> Self {
        Self::from_xyz([from.x, from.y, from.z])
    }

    pub fn x(&self) -> f32 {
        self.inner.x()
    }

    pub fn y(&self) -> f32 {
        self.inner.y()
    }

    pub fn z(&self) -> f32 {
        self.inner.z()
    }

    pub fn xyz(&self) -> [f32; 3] {
        [self.x(), self.y(), self.z()]
    }

    pub fn to_na(&self) -> Translation3<f32> {
        let [x, y, z] = self.xyz();
        Translation3::new(x, y, z)
    }
}

#[repr(transparent)]
pub struct Rotation {
    inner: UniquePtr<FfiRotation>,
}

impl Rotation {
    pub fn from_euler_angles(roll: f32, pitch: f32, yaw: f32) -> Self {
        let ptr = FfiRotation::new1(pitch, yaw, roll).within_unique_ptr();
        Self::from_cxx(ptr).unwrap()
    }

    pub fn from_cxx(ptr: UniquePtr<FfiRotation>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }

    pub fn from_na(from: &UnitQuaternion<f32>) -> Self {
        let (r, p, y) = from.euler_angles();
        Self::from_euler_angles(r, p, y)
    }

    pub fn roll(&self) -> f32 {
        self.inner.roll()
    }

    pub fn pitch(&self) -> f32 {
        self.inner.pitch()
    }

    pub fn yaw(&self) -> f32 {
        self.inner.yaw()
    }

    pub fn to_na(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_euler_angles(self.roll(), self.pitch(), self.yaw())
    }
}

#[repr(transparent)]
pub struct Transform {
    pub(crate) inner: UniquePtr<FfiTransform>,
}

impl Transform {
    pub fn new(location: &Location, rotation: &Rotation) -> Self {
        let ptr = FfiTransform::new1(&location.inner, &rotation.inner).within_unique_ptr();
        Self::from_cxx(ptr).unwrap()
    }

    pub fn from_cxx(ptr: UniquePtr<FfiTransform>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }

    pub fn from_na(pose: &Isometry3<f32>) -> Self {
        let Isometry3 {
            rotation,
            translation,
        } = pose;
        let location = Location::from_na(translation);
        let rotation = Rotation::from_na(rotation);
        Self::new(&location, &rotation)
    }

    pub fn location(&self) -> Location {
        Location::from_cxx(self.inner.location().within_unique_ptr()).unwrap()
    }

    pub fn rotation(&self) -> Rotation {
        Rotation::from_cxx(self.inner.rotation().within_unique_ptr()).unwrap()
    }

    pub fn to_na(&self) -> Isometry3<f32> {
        Isometry3 {
            rotation: self.rotation().to_na(),
            translation: self.location().to_na(),
        }
    }
}

#[derive(Clone, Copy)]
#[repr(transparent)]
pub(crate) struct TransformRef<'a> {
    pub(crate) inner: &'a FfiTransform,
}

impl<'a> TransformRef<'a> {
    pub fn from_cxx(ptr: &'a FfiTransform) -> Self {
        Self { inner: ptr }
    }

    pub fn location(&self) -> Location {
        Location::from_cxx(self.inner.location().within_unique_ptr()).unwrap()
    }

    pub fn rotation(&self) -> Rotation {
        Rotation::from_cxx(self.inner.rotation().within_unique_ptr()).unwrap()
    }

    pub fn to_na(&self) -> Isometry3<f32> {
        Isometry3 {
            rotation: self.rotation().to_na(),
            translation: self.location().to_na(),
        }
    }
}
