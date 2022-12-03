use crate::{
    geom::TransformExt,
    road::{JuncId, LaneId, LaneType, RoadId, SectionId},
};
use carla_sys::carla_rust::client::FfiWaypoint;
use cxx::SharedPtr;
use nalgebra::Isometry3;

#[repr(transparent)]
pub struct Waypoint {
    inner: SharedPtr<FfiWaypoint>,
}

impl Waypoint {
    pub fn id(&self) -> u64 {
        self.inner.GetId()
    }

    pub fn road_id(&self) -> RoadId {
        self.inner.GetRoadId()
    }

    pub fn section_id(&self) -> SectionId {
        self.inner.GetSectionId()
    }

    pub fn land_id(&self) -> LaneId {
        self.inner.GetLaneId()
    }

    pub fn junction_id(&self) -> JuncId {
        self.inner.GetJunctionId()
    }

    pub fn distance(&self) -> f64 {
        self.inner.GetDistance()
    }

    pub fn transofrm(&self) -> Isometry3<f32> {
        self.inner.GetTransform().to_na()
    }

    pub fn is_junction(&self) -> bool {
        self.inner.IsJunction()
    }

    pub fn lane_width(&self) -> f64 {
        self.inner.GetLaneWidth()
    }

    pub fn type_(&self) -> LaneType {
        self.inner.GetType()
    }

    pub fn left(&self) -> Waypoint {
        let ptr = self.inner.GetLeft();
        Self::from_cxx(ptr).unwrap()
    }

    pub fn right(&self) -> Waypoint {
        let ptr = self.inner.GetRight();
        Self::from_cxx(ptr).unwrap()
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiWaypoint>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}
