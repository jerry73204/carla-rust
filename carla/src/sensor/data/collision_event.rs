use crate::{client::Actor, sensor::SensorData};
use carla_sys::{carla::geom::Vector3D, carla_rust::sensor::data::FfiCollisionEvent};
use cxx::SharedPtr;
use derivative::Derivative;

#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct CollisionEvent {
    #[derivative(Debug = "ignore")]
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

impl TryFrom<SensorData> for CollisionEvent {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_collision_event();
        Self::from_cxx(ptr).ok_or(value)
    }
}
