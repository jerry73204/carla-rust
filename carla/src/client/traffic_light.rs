// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::{Actor, ActorBase, BoundingBoxList, TrafficLightList, WaypointList};
use crate::{geom::BoundingBox, road::SignId, rpc::TrafficLightState};
use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::client::{FfiActor, FfiTrafficLight};
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Represents a traffic light in the simulation, corresponding to
/// `carla.TrafficLight` in Python API.
#[derive(Clone, Derivative)]
#[derivative(Debug)]
pub struct TrafficLight {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiTrafficLight>,
}

impl TrafficLight {
    pub fn sign_id(&self) -> SignId {
        self.inner.GetSignId().to_string()
    }

    pub fn trigger_volume(&self) -> BoundingBox<f32> {
        BoundingBox::from_native(self.inner.GetTriggerVolume())
    }

    pub fn state(&self) -> TrafficLightState {
        self.inner.GetState()
    }

    pub fn set_state(&self, state: TrafficLightState) {
        self.inner.SetState(state);
    }

    pub fn green_time(&self) -> f32 {
        self.inner.GetGreenTime()
    }

    pub fn yellow_time(&self) -> f32 {
        self.inner.GetYellowTime()
    }

    pub fn red_time(&self) -> f32 {
        self.inner.GetRedTime()
    }

    pub fn set_green_time(&self, time: f32) {
        self.inner.SetGreenTime(time)
    }

    pub fn set_yellow_time(&self, time: f32) {
        self.inner.SetYellowTime(time)
    }

    pub fn set_red_time(&self, time: f32) {
        self.inner.SetRedTime(time)
    }

    pub fn elapsed_time(&self) -> f32 {
        self.inner.GetElapsedTime()
    }

    pub fn freeze(&self, freeze: bool) {
        self.inner.Freeze(freeze)
    }

    pub fn is_frozen(&self) -> bool {
        self.inner.IsFrozen()
    }

    pub fn pole_index(&self) -> u32 {
        self.inner.GetPoleIndex()
    }

    pub fn group_traffic_lights(&self) -> TrafficLightList {
        let ptr = self.inner.GetGroupTrafficLights().within_unique_ptr();
        unsafe { TrafficLightList::from_cxx(ptr).unwrap_unchecked() }
    }

    pub fn affected_lane_waypoints(&self) -> WaypointList {
        let ptr = self.inner.GetAffectedLaneWaypoints().within_unique_ptr();
        unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() }
    }

    pub fn light_boxes(&self) -> BoundingBoxList {
        let ptr = self.inner.GetLightBoxes().within_unique_ptr();
        unsafe { BoundingBoxList::from_cxx(ptr).unwrap_unchecked() }
    }

    pub fn opendrive_id(&self) -> SignId {
        self.inner.GetOpenDRIVEID().to_string()
    }

    pub fn stop_waypoints(&self) -> WaypointList {
        let ptr = self.inner.GetStopWaypoints().within_unique_ptr();
        unsafe { WaypointList::from_cxx(ptr).unwrap_unchecked() }
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiTrafficLight>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl ActorBase for TrafficLight {
    fn cxx_actor(&self) -> SharedPtr<FfiActor> {
        self.inner.to_actor()
    }
}

impl TryFrom<Actor> for TrafficLight {
    type Error = Actor;

    fn try_from(value: Actor) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_traffic_light();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(TrafficLight: Send, Sync);
