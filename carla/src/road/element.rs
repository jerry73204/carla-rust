use carla_sys::carla_rust::road::element::FfiLaneMarking;
use cxx::UniquePtr;

pub use carla_sys::carla::road::element::{
    LaneMarking_Color, LaneMarking_LaneChange, LaneMarking_Type,
};
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Road lane marking information.
///
/// Provides information about the visual and regulatory properties of lane markings,
/// including their type (solid, broken, etc.), color, width, and permitted lane changes.
///
/// Lane markings are typically accessed through [`Waypoint::left_lane_marking()`](crate::client::Waypoint::left_lane_marking)
/// and [`Waypoint::right_lane_marking()`](crate::client::Waypoint::right_lane_marking).
#[cfg_attr(
    carla_version_0916,
    doc = " See [carla.LaneMarking](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LaneMarking) in the Python API."
)]
#[cfg_attr(
    carla_version_0915,
    doc = " See [carla.LaneMarking](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LaneMarking) in the Python API."
)]
#[cfg_attr(
    carla_version_0914,
    doc = " See [carla.LaneMarking](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LaneMarking) in the Python API."
)]
///
/// # Examples
///
/// ```no_run
/// use carla::client::Client;
///
/// let client = Client::default();
/// let world = client.world();
/// let map = world.map();
///
/// // Get waypoint and check lane markings
/// let location = carla::geom::Location::new(0.0, 0.0, 0.0);
/// if let Some(waypoint) = map.waypoint_at(&location) {
///     let right_marking = waypoint.right_lane_marking();
///     println!("Right marking width: {} m", right_marking.width());
/// }
/// ```
#[derive(Derivative)]
#[derivative(Debug)]
pub struct LaneMarking {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiLaneMarking>,
}

impl LaneMarking {
    /// Returns the type of the lane marking.
    ///
    /// The type indicates the visual appearance (solid, broken, solid/broken combination, etc.)
    /// which helps determine when lane changes are permitted.
    ///
    /// See [`LaneMarking_Type`] for available types.
    pub fn type_(&self) -> LaneMarking_Type {
        self.inner.type_()
    }

    /// Returns the color of the lane marking.
    ///
    /// Common colors include white (for lane separation) and yellow (for center lines
    /// or no-passing zones in some regions).
    ///
    /// See [`LaneMarking_Color`] for available colors.
    pub fn color(&self) -> LaneMarking_Color {
        self.inner.color()
    }

    /// Returns the lane change permissions for this marking.
    ///
    /// Indicates whether lane changes are permitted across this marking, and in which direction.
    ///
    /// See [`LaneMarking_LaneChange`] for available options.
    pub fn lane_change(&self) -> LaneMarking_LaneChange {
        self.inner.lane_change()
    }

    /// Returns the width of the lane marking in meters.
    ///
    /// The width of the painted line. Standard lane markings are typically
    /// 0.1-0.15 meters wide.
    pub fn width(&self) -> f64 {
        self.inner.width()
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiLaneMarking>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

assert_impl_all!(LaneMarking: Send);
