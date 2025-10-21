// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use crate::{
    geom::BoundingBox,
    road::{JuncId, LaneType},
};
use carla_sys::carla_rust::client::{FfiJunction, FfiWaypointPair};
use cxx::{CxxVector, SharedPtr, UniquePtr};
use derivative::Derivative;
use static_assertions::assert_impl_all;

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
        unsafe { WaypointPairList::from_cxx(vec).unwrap_unchecked() }
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
    let first = unsafe { Waypoint::from_cxx(from.first()).unwrap_unchecked() };
    let second = unsafe { Waypoint::from_cxx(from.second()).unwrap_unchecked() };
    (first, second)
}

assert_impl_all!(Junction: Send, Sync);
// Note: WaypointPairList cannot be Send/Sync because it contains CxxVector<FfiWaypointPair>
// which has PhantomData<*const u8> making it !Send. This is intentional as CxxVector
// is designed for short-lived borrows from C++.
