// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use crate::{
    geom::Transform,
    road::{RoadId, SignalOrientation},
};
use carla_sys::carla_rust::client::FfiLandmark;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

use super::Waypoint;

/// Represents a landmark in the simulation.
///
/// Landmarks are elements in the road network such as traffic signs, speed limits,
/// and other regulatory or informational signs defined in OpenDRIVE. They provide
/// detailed information about road rules and can be queried from waypoints or the map.
///
/// Corresponds to [`carla.Landmark`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark) in the Python API.
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
/// # let spawn_points = map.recommended_spawn_points();
/// # let waypoint = map.waypoint_at(&spawn_points.get(0).unwrap().location).unwrap();
/// // Get landmarks within 50 meters of a waypoint
/// let landmarks = waypoint.all_landmarks_in_distance(50.0, false);
///
/// for landmark in landmarks.iter() {
///     println!("Landmark: {} (type: {})", landmark.name(), landmark.type_());
///     println!("  Value: {}{}", landmark.value(), landmark.unit());
///     println!("  Position: {:?}", landmark.transform().location);
/// }
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Landmark {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: SharedPtr<FfiLandmark>,
}

impl Landmark {
    /// Returns the waypoint closest to the landmark.
    ///
    /// See [carla.Landmark.waypoint](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.waypoint)
    /// in the Python API.
    pub fn waypoint(&self) -> Option<Waypoint> {
        let ptr = self.inner.GetWaypoint();
        Waypoint::from_cxx(ptr)
    }

    /// Returns the transform (position and rotation) of the landmark.
    ///
    /// See [carla.Landmark.transform](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.transform)
    /// in the Python API.
    pub fn transform(&self) -> Transform {
        Transform::from_ffi(self.inner.GetTransform().clone())
    }

    /// Returns the OpenDRIVE road ID.
    ///
    /// See [carla.Landmark.road_id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.road_id)
    /// in the Python API.
    pub fn road_id(&self) -> RoadId {
        self.inner.GetRoadId()
    }

    /// Returns the distance from the beginning of the road.
    ///
    /// See [carla.Landmark.distance](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.distance)
    /// in the Python API.
    pub fn distance(&self) -> f64 {
        self.inner.GetDistance()
    }

    /// Returns the s coordinate in the OpenDRIVE specification.
    ///
    /// See [carla.Landmark.s](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.s)
    /// in the Python API.
    ///
    /// The "s" coordinate represents the distance along the road.
    pub fn s(&self) -> f64 {
        self.inner.GetS()
    }

    /// Returns the t coordinate in the OpenDRIVE specification.
    ///
    /// See [carla.Landmark.t](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.t)
    /// in the Python API.
    ///
    /// The "t" coordinate represents the lateral offset from the road center.
    pub fn t(&self) -> f64 {
        self.inner.GetT()
    }

    /// Returns the unique identifier of the landmark.
    ///
    /// See [carla.Landmark.id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.id)
    /// in the Python API.
    pub fn id(&self) -> String {
        self.inner.GetId().to_string()
    }

    /// Returns the name of the landmark.
    ///
    /// See [carla.Landmark.name](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.name)
    /// in the Python API.
    pub fn name(&self) -> String {
        self.inner.GetName().to_string()
    }

    /// Returns whether the landmark is dynamic.
    ///
    /// See [carla.Landmark.is_dynamic](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.is_dynamic)
    /// in the Python API.
    ///
    /// Dynamic landmarks can change state (e.g., variable speed limit signs).
    pub fn is_dynamic(&self) -> bool {
        self.inner.IsDynamic()
    }

    /// Returns the orientation of the signal.
    ///
    /// See [carla.Landmark.orientation](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.orientation)
    /// in the Python API.
    pub fn orientation(&self) -> SignalOrientation {
        self.inner.GetOrientation()
    }

    /// Returns the Z offset of the landmark.
    ///
    /// See [carla.Landmark.z_offset](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.z_offset)
    /// in the Python API.
    pub fn z_offset(&self) -> f64 {
        self.inner.GetZOffset()
    }

    /// Returns the country code of the landmark.
    ///
    /// See [carla.Landmark.country](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.country)
    /// in the Python API.
    pub fn country(&self) -> String {
        self.inner.GetCountry().to_string()
    }

    /// Returns the type of the landmark.
    ///
    /// See [carla.Landmark.type](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.type)
    /// in the Python API.
    ///
    /// Common types include speed limits ("1000001"), stop signs, etc.
    pub fn type_(&self) -> String {
        self.inner.GetType().to_string()
    }

    /// Returns the subtype of the landmark.
    ///
    /// See [carla.Landmark.sub_type](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.sub_type)
    /// in the Python API.
    pub fn sub_type(&self) -> String {
        self.inner.GetSubType().to_string()
    }

    /// Returns the value of the landmark.
    ///
    /// See [carla.Landmark.value](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.value)
    /// in the Python API.
    ///
    /// For speed limit signs, this is the speed limit value.
    pub fn value(&self) -> f64 {
        self.inner.GetValue()
    }

    /// Returns the unit of the landmark's value.
    ///
    /// See [carla.Landmark.unit](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.unit)
    /// in the Python API.
    ///
    /// For speed limits, this is typically "km/h" or "mph".
    pub fn unit(&self) -> String {
        self.inner.GetUnit().to_string()
    }

    /// Returns the height of the landmark.
    ///
    /// See [carla.Landmark.height](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.height)
    /// in the Python API.
    pub fn height(&self) -> f64 {
        self.inner.GetHeight()
    }

    /// Returns the width of the landmark.
    ///
    /// See [carla.Landmark.width](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.width)
    /// in the Python API.
    pub fn width(&self) -> f64 {
        self.inner.GetWidth()
    }

    /// Returns the text content of the landmark.
    ///
    /// See [carla.Landmark.text](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.text)
    /// in the Python API.
    pub fn text(&self) -> String {
        self.inner.GetText().to_string()
    }

    /// Returns the horizontal offset.
    ///
    /// See [carla.Landmark.h_offset](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.h_offset)
    /// in the Python API.
    pub fn h_offset(&self) -> f64 {
        self.inner.GethOffset()
    }

    /// Returns the pitch angle.
    ///
    /// See [carla.Landmark.pitch](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.pitch)
    /// in the Python API.
    pub fn pitch(&self) -> f64 {
        self.inner.GetPitch()
    }

    /// Returns the roll angle.
    ///
    /// See [carla.Landmark.roll](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark.roll)
    /// in the Python API.
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
