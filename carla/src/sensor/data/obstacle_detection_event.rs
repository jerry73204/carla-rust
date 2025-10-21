// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use crate::{client::Actor, sensor::SensorData};
use carla_sys::carla_rust::sensor::data::FfiObstacleDetectionEvent;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct ObstacleDetectionEvent {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiObstacleDetectionEvent>,
}

impl ObstacleDetectionEvent {
    pub fn actor(&self) -> Actor {
        unsafe { Actor::from_cxx(self.inner.GetActor()).unwrap_unchecked() }
    }

    pub fn other_actor(&self) -> Actor {
        unsafe { Actor::from_cxx(self.inner.GetOtherActor()).unwrap_unchecked() }
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

impl TryFrom<SensorData> for ObstacleDetectionEvent {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_obstacle_detection_event();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(ObstacleDetectionEvent: Send, Sync);
