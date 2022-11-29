use crate::ffi;
use cxx::UniquePtr;
use nalgebra::{Isometry3, Translation3, Vector3};

pub trait ActorTrait {
    fn as_actor(&self) -> &UniquePtr<ffi::SharedActor>;

    fn location(&self) -> Translation3<f32> {
        ffi::actor_get_location(&self.as_actor()).to_na()
    }

    fn transform(&self) -> Isometry3<f32> {
        ffi::actor_get_transform(&self.as_actor()).to_na()
    }

    fn velocity(&self) -> Vector3<f32> {
        ffi::actor_get_velocity(&self.as_actor()).to_na()
    }

    fn acceleration(&self) -> Vector3<f32> {
        ffi::actor_get_acceleration(&self.as_actor()).to_na()
    }

    fn angular_velocity(&self) -> Vector3<f32> {
        ffi::actor_get_angular_velocity(&self.as_actor()).to_na()
    }
}

pub struct Actor {
    pub(crate) inner: UniquePtr<ffi::SharedActor>,
}

impl ActorTrait for Actor {
    fn as_actor(&self) -> &UniquePtr<ffi::SharedActor> {
        &self.inner
    }
}
