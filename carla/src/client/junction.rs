use crate::{
    geom::BoundingBox,
    road::{JuncId, LaneType},
};
use carla_sys::carla_rust::client::{FfiJunction, FfiWaypointPair};
use cxx::{CxxVector, SharedPtr, UniquePtr};
use derivative::Derivative;
use static_assertions::assert_impl_all;

use super::Waypoint;

/// Represents a junction (intersection) in the simulation.
///
/// Junctions are areas where multiple roads meet. They provide information about
/// the possible paths through the intersection and can be used for navigation
/// and traffic management.
///
/// Corresponds to [`carla.Junction`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Junction) in the Python API.
///
/// # Examples
///
/// ```no_run
/// use carla::{client::Client, road::LaneType};
///
/// let client = Client::default();
/// let world = client.world();
/// let map = world.map();
///
/// # let spawn_points = map.recommended_spawn_points();
/// # let waypoint = map.waypoint_at(&spawn_points.get(0).unwrap().location);
/// // Check if a waypoint is in a junction
/// if let Some(wp) = waypoint {
///     if wp.is_junction() {
///         if let Some(junction) = wp.junction() {
///             println!("Junction ID: {}", junction.id());
///
///             // Get waypoint pairs for driving lanes
///             let pairs = junction.waypoints(LaneType::Driving);
///             println!("Junction has {} waypoint pairs", pairs.len());
///         }
///     }
/// }
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Junction {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: SharedPtr<FfiJunction>,
}

impl Junction {
    /// Returns the junction ID.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Junction.id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Junction.id)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Junction.id](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Junction.id)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Junction.id](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Junction.id)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn id(&self) -> JuncId {
        self.inner.GetId()
    }

    /// Returns pairs of waypoints defining possible paths through the junction.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Junction.get_waypoints](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Junction.get_waypoints)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Junction.get_waypoints](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Junction.get_waypoints)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Junction.get_waypoints](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Junction.get_waypoints)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    ///
    /// # Arguments
    /// * `type_` - Lane type filter (e.g., `LaneType::Driving`)
    pub fn waypoints(&self, type_: LaneType) -> WaypointPairList {
        let vec = self.inner.GetWaypoints(type_);
        unsafe { WaypointPairList::from_cxx(vec).unwrap_unchecked() }
    }

    /// Returns the bounding box enclosing the junction.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Junction.bounding_box](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Junction.bounding_box)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Junction.bounding_box](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Junction.bounding_box)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Junction.bounding_box](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Junction.bounding_box)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn bounding_box(&self) -> BoundingBox {
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
