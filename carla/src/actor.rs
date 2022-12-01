use crate::Location;
use crate::Transform;
use crate::Vector3D;
use crate::Vehicle;
use autocxx::prelude::*;
use carla_sys::carla_rust::client::FfiActor;
use cxx::UniquePtr;
use nalgebra::{Isometry3, Translation3, Vector3};
use std::ops::Deref;

pub enum CowFfiActor<'a> {
    Owned(UniquePtr<FfiActor>),
    Borrowed(&'a FfiActor),
}

impl<'a> Deref for CowFfiActor<'a> {
    type Target = FfiActor;

    fn deref(&self) -> &Self::Target {
        match self {
            CowFfiActor::Owned(me) => me,
            CowFfiActor::Borrowed(me) => me,
        }
    }
}

pub trait ActorBase {
    fn cxx_actor(&self) -> CowFfiActor<'_>;

    fn location(&self) -> Translation3<f32> {
        let ptr = self.cxx_actor().GetLocation().within_unique_ptr();
        Location::from_cxx(ptr).to_na()
    }

    fn transform(&self) -> Isometry3<f32> {
        let ptr = self.cxx_actor().GetTransform().within_unique_ptr();
        Transform::from_cxx(ptr).to_na()
    }

    fn velocity(&self) -> Vector3<f32> {
        let ptr = self.cxx_actor().GetVelocity().within_unique_ptr();
        Vector3D::from_cxx(ptr).to_na()
    }

    fn acceleration(&self) -> Vector3<f32> {
        let ptr = self.cxx_actor().GetAcceleration().within_unique_ptr();
        Vector3D::from_cxx(ptr).to_na()
    }

    fn angular_velocity(&self) -> Vector3<f32> {
        let ptr = self.cxx_actor().GetAngularVelocity().within_unique_ptr();
        Vector3D::from_cxx(ptr).to_na()
    }
}

pub struct Actor {
    pub(crate) inner: UniquePtr<FfiActor>,
}

impl Actor {
    pub fn try_into_vehicle(self) -> Result<Vehicle, Self> {
        let ptr = self.inner.to_vehicle();
        if ptr.is_null() {
            Err(self)
        } else {
            Ok(Vehicle::from_cxx(ptr))
        }
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiActor>) -> Self {
        Self { inner: ptr }
    }
}

impl ActorBase for Actor {
    fn cxx_actor(&self) -> CowFfiActor<'_> {
        CowFfiActor::Borrowed(&self.inner)
    }
}
