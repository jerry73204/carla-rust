use crate::{client::Actor, road::element::LaneMarking};
use autocxx::prelude::*;
use carla_sys::carla_rust::sensor::data::FfiLaneInvasionEvent;
use cxx::SharedPtr;

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
