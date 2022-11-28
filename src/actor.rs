use crate::ffi;
use cxx::UniquePtr;
use nalgebra::{Isometry3, Translation3, Vector3};

pub struct Actor {
    pub(crate) inner: UniquePtr<ffi::SharedActor>,
}

impl Actor {
    pub fn location(&self) -> Translation3<f32> {
        ffi::actor_get_location(&self.inner).to_na()
    }

    pub fn transform(&self) -> Isometry3<f32> {
        ffi::actor_get_transform(&self.inner).to_na()
    }

    pub fn velocity(&self) -> Vector3<f32> {
        ffi::actor_get_velocity(&self.inner).to_na()
    }

    pub fn acceleration(&self) -> Vector3<f32> {
        ffi::actor_get_acceleration(&self.inner).to_na()
    }
    pub fn angular_velocity(&self) -> Vector3<f32> {
        ffi::actor_get_angular_velocity(&self.inner).to_na()
    }
}
