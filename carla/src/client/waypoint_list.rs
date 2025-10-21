use super::Waypoint;
use carla_sys::carla_rust::client::FfiWaypointList;
use cxx::UniquePtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// A list of waypoints.
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct WaypointList {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiWaypointList>,
}

impl WaypointList {
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn get(&self, index: usize) -> Option<Waypoint> {
        if index >= self.len() {
            return None;
        }
        let ptr = self.inner.get(index);
        Some(Waypoint::from_cxx(ptr).unwrap())
    }

    pub fn iter(&self) -> impl Iterator<Item = Waypoint> + '_ {
        (0..self.len()).filter_map(move |index| self.get(index))
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiWaypointList>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

assert_impl_all!(WaypointList: Send, Sync);
