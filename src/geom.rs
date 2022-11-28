use crate::ffi;
use cxx::UniquePtr;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector2, Vector3};

impl ffi::Vector2D {
    pub fn new(x: f32, y: f32) -> UniquePtr<Self> {
        ffi::vec2d_new(x, y)
    }

    pub fn from_na(from: &Vector2<f32>) -> UniquePtr<Self> {
        Self::new(from.x, from.y)
    }

    pub fn x(&self) -> f32 {
        ffi::vec2d_x(self)
    }

    pub fn y(&self) -> f32 {
        ffi::vec2d_y(self)
    }

    pub fn xy(&self) -> [f32; 2] {
        [self.x(), self.y()]
    }

    pub fn to_na(&self) -> Vector2<f32> {
        Vector2::new(self.x(), self.y())
    }
}

impl ffi::Vector3D {
    pub fn new(x: f32, y: f32, z: f32) -> UniquePtr<Self> {
        ffi::vec3d_new(x, y, z)
    }

    pub fn from_na(from: &Vector3<f32>) -> UniquePtr<Self> {
        Self::new(from.x, from.y, from.z)
    }

    pub fn x(&self) -> f32 {
        ffi::vec3d_x(self)
    }

    pub fn y(&self) -> f32 {
        ffi::vec3d_y(self)
    }
    pub fn z(&self) -> f32 {
        ffi::vec3d_z(self)
    }

    pub fn xyz(&self) -> [f32; 3] {
        [self.x(), self.y(), self.z()]
    }

    pub fn to_na(&self) -> Vector3<f32> {
        Vector3::new(self.x(), self.y(), self.z())
    }
}

impl ffi::Location {
    pub fn new(x: f32, y: f32, z: f32) -> UniquePtr<Self> {
        ffi::location_from_xyz(x, y, z)
    }

    pub fn from_na(from: &Translation3<f32>) -> UniquePtr<Self> {
        Self::new(from.x, from.y, from.z)
    }

    pub fn as_vec3d(&self) -> &ffi::Vector3D {
        ffi::location_as_vec3d(self)
    }

    pub fn to_na(&self) -> Translation3<f32> {
        let [x, y, z] = self.as_vec3d().xyz();
        Translation3::new(x, y, z)
    }
}

impl ffi::Rotation {
    pub fn from_euler_angles(roll: f32, pitch: f32, yaw: f32) -> UniquePtr<Self> {
        ffi::rotation_from_pitch_yaw_roll(pitch, yaw, roll)
    }

    pub fn from_na(from: &UnitQuaternion<f32>) -> UniquePtr<Self> {
        let (r, p, y) = from.euler_angles();
        Self::from_euler_angles(r, p, y)
    }

    pub fn roll(&self) -> f32 {
        ffi::rotation_roll(self)
    }

    pub fn pitch(&self) -> f32 {
        ffi::rotation_pitch(self)
    }

    pub fn yaw(&self) -> f32 {
        ffi::rotation_yaw(self)
    }

    pub fn to_na(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_euler_angles(self.roll(), self.pitch(), self.yaw())
    }
}

impl ffi::Transform {
    pub fn new(location: &ffi::Location, rotation: &ffi::Rotation) -> UniquePtr<Self> {
        ffi::transform_new(location, rotation)
    }

    pub fn from_na(pose: &Isometry3<f32>) -> UniquePtr<Self> {
        let Isometry3 {
            rotation,
            translation,
        } = pose;
        Self::new(
            &ffi::Location::from_na(translation),
            &ffi::Rotation::from_na(rotation),
        )
    }

    pub fn location(&self) -> &ffi::Location {
        ffi::transform_get_location(self)
    }

    pub fn rotation(&self) -> &ffi::Rotation {
        ffi::transform_get_rotation(self)
    }

    pub fn to_na(&self) -> Isometry3<f32> {
        Isometry3 {
            rotation: self.rotation().to_na(),
            translation: self.location().to_na(),
        }
    }
}
