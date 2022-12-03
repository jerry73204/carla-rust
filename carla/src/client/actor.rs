use super::{Sensor, Vehicle};
use crate::geom::{LocationExt, TransformExt, Vector3DExt};
use carla_sys::carla_rust::client::FfiActor;
use cxx::SharedPtr;
use nalgebra::{Isometry3, Translation3, Vector3};

pub trait ActorBase {
    fn cxx_actor(&self) -> SharedPtr<FfiActor>;

    fn location(&self) -> Translation3<f32> {
        self.cxx_actor().GetLocation().to_na()
    }

    fn transform(&self) -> Isometry3<f32> {
        self.cxx_actor().GetTransform().to_na()
    }

    fn velocity(&self) -> Vector3<f32> {
        self.cxx_actor().GetVelocity().to_na()
    }

    fn acceleration(&self) -> Vector3<f32> {
        self.cxx_actor().GetAcceleration().to_na()
    }

    fn angular_velocity(&self) -> Vector3<f32> {
        self.cxx_actor().GetAngularVelocity().to_na()
    }
}

#[derive(Clone)]
#[repr(transparent)]
pub struct Actor {
    pub(crate) inner: SharedPtr<FfiActor>,
}

impl Actor {
    pub fn try_into_vehicle(self) -> Result<Vehicle, Self> {
        let ptr = self.inner.to_vehicle();
        Vehicle::from_cxx(ptr).ok_or(self)
    }

    pub fn try_into_sensor(self) -> Result<Sensor, Self> {
        let ptr = self.inner.to_sensor();
        Sensor::from_cxx(ptr).ok_or(self)
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiActor>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl ActorBase for Actor {
    fn cxx_actor(&self) -> SharedPtr<FfiActor> {
        self.inner.clone()
    }
}
