use crate::{
    geom::BoundingBox,
    road::{JuncId, LaneType},
};
use carla_sys::carla_rust::client::{FfiJunction, FfiWaypointPair};
use cxx::{CxxVector, SharedPtr, UniquePtr};
use derivative::Derivative;

use super::Waypoint;

/// Represents a junction in the simulation.
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Junction {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: SharedPtr<FfiJunction>,
}

impl Junction {
    pub fn id(&self) -> JuncId {
        self.inner.GetId()
    }

    pub fn waypoints(&self, type_: LaneType) -> WaypointPairList {
        let vec = self.inner.GetWaypoints(type_);
        WaypointPairList::from_cxx(vec).unwrap()
    }

    pub fn bounding_box(&self) -> BoundingBox<f32> {
        let bbox = self.inner.GetBoundingBox();
        BoundingBox::from_native(&bbox)
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiJunction>) -> Option<Self> {
        if ptr.is_null() {
            return None;
        }
        Some(Self { inner: ptr })
    }
}

#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct WaypointPairList {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<CxxVector<FfiWaypointPair>>,
}

impl WaypointPairList {
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn get(&self, index: usize) -> Option<(Waypoint, Waypoint)> {
        let pair = self.inner.get(index)?;
        Some(convert_pair(pair))
    }

    pub fn iter(&self) -> impl Iterator<Item = (Waypoint, Waypoint)> + '_ {
        self.inner.iter().map(convert_pair)
    }

    fn from_cxx(ptr: UniquePtr<CxxVector<FfiWaypointPair>>) -> Option<Self> {
        if ptr.is_null() {
            return None;
        }
        Some(Self { inner: ptr })
    }
}

fn convert_pair(from: &FfiWaypointPair) -> (Waypoint, Waypoint) {
    let first = Waypoint::from_cxx(from.first()).unwrap();
    let second = Waypoint::from_cxx(from.second()).unwrap();
    (first, second)
}
