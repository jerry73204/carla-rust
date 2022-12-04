use crate::{client::Actor, road::element::LaneMarking, sensor::SensorData};
use autocxx::prelude::*;
use carla_sys::carla_rust::sensor::data::FfiLaneInvasionEvent;
use cxx::SharedPtr;

#[derive(Clone)]
#[repr(transparent)]
pub struct LaneInvasionEvent {
    inner: SharedPtr<FfiLaneInvasionEvent>,
}

impl LaneInvasionEvent {
    pub fn actor(&self) -> Actor {
        Actor::from_cxx(self.inner.GetActor()).unwrap()
    }

    pub fn crossed_lane_markings(&self) -> Vec<LaneMarking> {
        self.inner
            .GetCrossedLaneMarkings()
            .iter()
            .map(|mark| (*mark).clone().within_unique_ptr())
            .map(|ptr| LaneMarking::from_cxx(ptr).unwrap())
            .collect()
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiLaneInvasionEvent>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl TryFrom<SensorData> for LaneInvasionEvent {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_lane_invasion_event();
        Self::from_cxx(ptr).ok_or(value)
    }
}
