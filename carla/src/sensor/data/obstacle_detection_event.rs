use crate::client::Actor;
use carla_sys::carla_rust::sensor::data::FfiObstacleDetectionEvent;
use cxx::SharedPtr;

pub struct ObstacleDetectionEvent {
    inner: SharedPtr<FfiObstacleDetectionEvent>,
}

impl ObstacleDetectionEvent {
    pub fn actor(&self) -> Actor {
        Actor::from_cxx(self.inner.GetActor()).unwrap()
    }

    pub fn other_actor(&self) -> Actor {
        Actor::from_cxx(self.inner.GetOtherActor()).unwrap()
    }

    pub fn distance(&self) -> f32 {
        self.inner.GetDistance()
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiObstacleDetectionEvent>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}
