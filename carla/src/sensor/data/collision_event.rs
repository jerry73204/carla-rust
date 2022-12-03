use carla_sys::{carla::geom::Vector3D, carla_rust::sensor::data::FfiCollisionEvent};
use cxx::SharedPtr;

use crate::client::Actor;

pub struct CollisionEvent {
    inner: SharedPtr<FfiCollisionEvent>,
}

impl CollisionEvent {
    pub fn actor(&self) -> Actor {
        Actor::from_cxx(self.inner.GetActor()).unwrap()
    }

    pub fn other_actor(&self) -> Actor {
        Actor::from_cxx(self.inner.GetOtherActor()).unwrap()
    }

    pub fn normal_impulse(&self) -> &Vector3D {
        self.inner.GetNormalImpulse()
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiCollisionEvent>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}
