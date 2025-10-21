// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use crate::{client::Actor, sensor::SensorData};
use carla_sys::{carla::geom::Vector3D, carla_rust::sensor::data::FfiCollisionEvent};
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct CollisionEvent {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiCollisionEvent>,
}

impl CollisionEvent {
    pub fn actor(&self) -> Actor {
        unsafe { Actor::from_cxx(self.inner.GetActor()).unwrap_unchecked() }
    }

    pub fn other_actor(&self) -> Option<Actor> {
        Actor::from_cxx(self.inner.GetOtherActor())
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

impl TryFrom<SensorData> for CollisionEvent {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_collision_event();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(CollisionEvent: Send, Sync);
