// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use crate::{
    geom::TransformExt,
    road::{RoadId, SignalOrientation},
};
use carla_sys::carla_rust::client::FfiLandmark;
use cxx::SharedPtr;
use derivative::Derivative;
use nalgebra::Isometry3;
use static_assertions::assert_impl_all;

use super::Waypoint;

/// Represents a landmark in the simulation.
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Landmark {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: SharedPtr<FfiLandmark>,
}

impl Landmark {
    pub fn waypoint(&self) -> Waypoint {
        let ptr = self.inner.GetWaypoint();
        unsafe { Waypoint::from_cxx(ptr).unwrap_unchecked() }
    }

    pub fn transform(&self) -> Isometry3<f32> {
        self.inner.GetTransform().to_na()
    }

    pub fn road_id(&self) -> RoadId {
        self.inner.GetRoadId()
    }

    pub fn distance(&self) -> f64 {
        self.inner.GetDistance()
    }

    pub fn s(&self) -> f64 {
        self.inner.GetS()
    }

    pub fn t(&self) -> f64 {
        self.inner.GetT()
    }

    pub fn id(&self) -> String {
        self.inner.GetId().to_string()
    }

    pub fn name(&self) -> String {
        self.inner.GetName().to_string()
    }

    pub fn is_dynamic(&self) -> bool {
        self.inner.IsDynamic()
    }

    pub fn orientation(&self) -> SignalOrientation {
        self.inner.GetOrientation()
    }

    pub fn z_offet(&self) -> f64 {
        self.inner.GetZOffset()
    }

    pub fn country(&self) -> String {
        self.inner.GetCountry().to_string()
    }

    pub fn type_(&self) -> String {
        self.inner.GetType().to_string()
    }

    pub fn sub_type(&self) -> String {
        self.inner.GetSubType().to_string()
    }

    pub fn value(&self) -> f64 {
        self.inner.GetValue()
    }

    pub fn unit(&self) -> String {
        self.inner.GetUnit().to_string()
    }

    pub fn height(&self) -> f64 {
        self.inner.GetHeight()
    }

    pub fn width(&self) -> f64 {
        self.inner.GetWidth()
    }

    pub fn text(&self) -> String {
        self.inner.GetText().to_string()
    }

    pub fn h_offset(&self) -> f64 {
        self.inner.GethOffset()
    }

    pub fn pitch(&self) -> f64 {
        self.inner.GetPitch()
    }

    pub fn roll(&self) -> f64 {
        self.inner.GetRoll()
    }
}

impl Landmark {
    pub(crate) fn from_cxx(ptr: SharedPtr<FfiLandmark>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

assert_impl_all!(Landmark: Send, Sync);
