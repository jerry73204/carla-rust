use autocxx::prelude::*;
use carla_sys::carla_rust::geom::{
    FfiLocation, FfiRotation, FfiTransform, FfiVector2D, FfiVector3D,
};
use cxx::UniquePtr;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector2, Vector3};

pub struct Vector2D {
    inner: UniquePtr<FfiVector2D>,
}

impl Vector2D {
    pub fn from_xy(xy: [f32; 2]) -> Self {
        let [x, y] = xy;
        let ptr = FfiVector2D::new1(x, y).within_unique_ptr();
        Self::from_cxx(ptr)
    }

    pub fn from_cxx(from: UniquePtr<FfiVector2D>) -> Self {
        Self { inner: from }
    }

    pub fn from_na(from: &Vector2<f32>) -> Self {
        Self::from_xy([from.x, from.y])
    }

    pub fn x(&self) -> f32 {
        self.inner.x()
    }

    pub fn y(&self) -> f32 {
        self.inner.y()
    }

    pub fn xy(&self) -> [f32; 2] {
        [self.x(), self.y()]
    }

    pub fn to_na(&self) -> Vector2<f32> {
        Vector2::new(self.x(), self.y())
    }
}

pub struct Vector3D {
    inner: UniquePtr<FfiVector3D>,
}

impl Vector3D {
    pub fn from_xyz(xyz: [f32; 3]) -> Self {
        let [x, y, z] = xyz;
        let ptr = FfiVector3D::new1(x, y, z).within_unique_ptr();
        Self::from_cxx(ptr)
    }

    pub fn from_cxx(from: UniquePtr<FfiVector3D>) -> Self {
        Self { inner: from }
    }

    pub fn from_na(from: &Vector3<f32>) -> Self {
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

    pub fn to_na(&self) -> Vector3<f32> {
        Vector3::new(self.x(), self.y(), self.z())
    }
}

pub struct Location {
    inner: UniquePtr<FfiLocation>,
}

impl Location {
    pub fn from_xyz(xyz: [f32; 3]) -> Self {
        let [x, y, z] = xyz;
        let ptr = FfiLocation::new1(x, y, z).within_unique_ptr();
        Self::from_cxx(ptr)
    }

    pub fn from_cxx(from: UniquePtr<FfiLocation>) -> Self {
        Self { inner: from }
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

pub struct Rotation {
    inner: UniquePtr<FfiRotation>,
}

impl Rotation {
    pub fn from_euler_angles(roll: f32, pitch: f32, yaw: f32) -> Self {
        let ptr = FfiRotation::new1(pitch, yaw, roll).within_unique_ptr();
        Self::from_cxx(ptr)
    }

    pub fn from_cxx(from: UniquePtr<FfiRotation>) -> Self {
        Self { inner: from }
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

pub struct Transform {
    inner: UniquePtr<FfiTransform>,
}

impl Transform {
    pub fn new(location: &Location, rotation: &Rotation) -> Self {
        let ptr = FfiTransform::new1(&location.inner, &rotation.inner).within_unique_ptr();
        Self::from_cxx(ptr)
    }

    pub fn from_cxx(from: UniquePtr<FfiTransform>) -> Self {
        Self { inner: from }
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
        Location::from_cxx(self.inner.location().within_unique_ptr())
    }

    pub fn rotation(&self) -> Rotation {
        Rotation::from_cxx(self.inner.rotation().within_unique_ptr())
    }

    pub fn to_na(&self) -> Isometry3<f32> {
        Isometry3 {
            rotation: self.rotation().to_na(),
            translation: self.location().to_na(),
        }
    }
}
