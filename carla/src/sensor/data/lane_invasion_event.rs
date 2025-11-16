//! Lane invasion event sensor data.
//!
//! This module provides the [`LaneInvasionEvent`] type for detecting when an actor
//! crosses lane markings.

use crate::{client::Actor, road::element::LaneMarking, sensor::SensorData};
use autocxx::prelude::*;
use carla_sys::carla_rust::sensor::data::FfiLaneInvasionEvent;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Lane invasion event data from a lane invasion sensor.
///
/// Generated when an actor with a lane invasion sensor crosses one or more lane markings.
/// Useful for implementing lane departure warnings and autonomous driving assistance.
///
/// Corresponds to [`carla.LaneInvasionEvent`] in the Python API.
#[cfg_attr(carla_version_0916, doc = "")]
#[cfg_attr(
    carla_version_0916,
    doc = " [`carla.LaneInvasionEvent`]: https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LaneInvasionEvent"
)]
#[cfg_attr(carla_version_0915, doc = "")]
#[cfg_attr(
    carla_version_0915,
    doc = " [`carla.LaneInvasionEvent`]: https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LaneInvasionEvent"
)]
#[cfg_attr(carla_version_0914, doc = "")]
#[cfg_attr(
    carla_version_0914,
    doc = " [`carla.LaneInvasionEvent`]: https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LaneInvasionEvent"
)]
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     client::Client,
///     sensor::{data::LaneInvasionEvent, SensorDataBase},
/// };
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// // Spawn vehicle with lane invasion sensor
/// let bp_lib = world.blueprint_library();
/// let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
/// let lane_invasion_bp = bp_lib.find("sensor.other.lane_invasion").unwrap();
///
/// let spawn_points = world.map().recommended_spawn_points();
/// let vehicle = world
///     .spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap())
///     .unwrap();
///
/// let lane_sensor = world
///     .spawn_actor_opt(
///         &lane_invasion_bp,
///         &nalgebra::Isometry3::identity(),
///         Some(&vehicle),
///         carla::rpc::AttachmentType::Rigid,
///     )
///     .unwrap();
///
/// let sensor: carla::client::Sensor = lane_sensor.try_into().unwrap();
/// sensor.listen(move |data| {
///     if let Ok(lane_invasion) = LaneInvasionEvent::try_from(data) {
///         println!("Lane departure detected!");
///         let markings = lane_invasion.crossed_lane_markings();
///         println!("  Crossed {} lane marking(s)", markings.len());
///         for marking in markings {
///             println!(
///                 "    Type: {:?}, Color: {:?}",
///                 marking.lane_type(),
///                 marking.color()
///             );
///         }
///     }
/// });
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct LaneInvasionEvent {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiLaneInvasionEvent>,
}

impl LaneInvasionEvent {
    /// Returns the actor that owns the lane invasion sensor.
    ///
    /// This is the actor that crossed the lane markings and has the sensor attached.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LaneInvasionEvent.actor](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LaneInvasionEvent.actor)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LaneInvasionEvent.actor](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LaneInvasionEvent.actor)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LaneInvasionEvent.actor](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LaneInvasionEvent.actor)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::sensor::data::LaneInvasionEvent;
    /// # let lane_invasion: LaneInvasionEvent = todo!();
    /// let actor = lane_invasion.actor();
    /// println!("Lane invasion actor ID: {}", actor.id());
    /// ```
    pub fn actor(&self) -> Actor {
        unsafe { Actor::from_cxx(self.inner.GetActor()).unwrap_unchecked() }
    }

    /// Returns the list of lane markings that were crossed.
    ///
    /// Each [`LaneMarking`] contains information about the type (solid, broken, etc.),
    /// color (white, yellow, etc.), and width of the marking.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LaneInvasionEvent.crossed_lane_markings](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LaneInvasionEvent.crossed_lane_markings)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LaneInvasionEvent.crossed_lane_markings](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LaneInvasionEvent.crossed_lane_markings)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LaneInvasionEvent.crossed_lane_markings](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LaneInvasionEvent.crossed_lane_markings)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::sensor::data::LaneInvasionEvent;
    /// # let lane_invasion: LaneInvasionEvent = todo!();
    /// let markings = lane_invasion.crossed_lane_markings();
    /// println!("Crossed {} lane marking(s)", markings.len());
    /// for marking in markings {
    ///     println!("  Type: {:?}", marking.lane_type());
    ///     println!("  Color: {:?}", marking.color());
    ///     println!("  Width: {}", marking.width());
    /// }
    /// ```
    pub fn crossed_lane_markings(&self) -> Vec<LaneMarking> {
        self.inner
            .GetCrossedLaneMarkings()
            .iter()
            .map(|mark| (*mark).clone().within_unique_ptr())
            .map(|ptr| unsafe { LaneMarking::from_cxx(ptr).unwrap_unchecked() })
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

impl TryFrom<SensorData> for LaneInvasionEvent {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_lane_invasion_event();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(LaneInvasionEvent: Send, Sync);
