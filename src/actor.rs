use crate::{ffi::carla_rust::client::FfiActor, Location, Transform, Vector3D};
use autocxx::prelude::*;
use cxx::UniquePtr;
use nalgebra::{Isometry3, Translation3, Vector3};

pub trait ActorTrait {
    fn as_cxx_actor(&self) -> &FfiActor;

    fn location(&self) -> Translation3<f32> {
        let ptr = self.as_cxx_actor().GetLocation().within_unique_ptr();
        Location::from_cxx(ptr).to_na()
    }

    fn transform(&self) -> Isometry3<f32> {
        let ptr = self.as_cxx_actor().GetTransform().within_unique_ptr();
        Transform::from_cxx(ptr).to_na()
    }

    fn velocity(&self) -> Vector3<f32> {
        let ptr = self.as_cxx_actor().GetVelocity().within_unique_ptr();
        Vector3D::from_cxx(ptr).to_na()
    }

    fn acceleration(&self) -> Vector3<f32> {
        let ptr = self.as_cxx_actor().GetAcceleration().within_unique_ptr();
        Vector3D::from_cxx(ptr).to_na()
    }

    fn angular_velocity(&self) -> Vector3<f32> {
        let ptr = self.as_cxx_actor().GetAngularVelocity().within_unique_ptr();
        Vector3D::from_cxx(ptr).to_na()
    }
}

pub struct Actor {
    pub(crate) inner: UniquePtr<FfiActor>,
}

impl ActorTrait for Actor {
    fn as_cxx_actor(&self) -> &FfiActor {
        &self.inner
    }
}
