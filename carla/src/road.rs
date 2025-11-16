pub use carla_sys::carla::road::{
    JuncId, LaneId, Lane_LaneType as LaneType, RoadId, SectionId, SignalOrientation,
};

/// Traffic sign identifier.
///
/// String-based identifier for traffic signs in the road network. Traffic signs
/// can be queried through the map and waypoint APIs.
#[cfg_attr(
    carla_version_0916,
    doc = " See [OpenDRIVE specification](https://carla.readthedocs.io/en/0.9.16/) for details."
)]
#[cfg_attr(
    carla_version_0915,
    doc = " See [OpenDRIVE specification](https://carla.readthedocs.io/en/0.9.15/) for details."
)]
#[cfg_attr(
    carla_version_0914,
    doc = " See [OpenDRIVE specification](https://carla.readthedocs.io/en/0.9.14/) for details."
)]
pub type SignId = String;

/// Traffic signal controller identifier.
///
/// String-based identifier for traffic light controllers in the road network.
/// Controllers manage groups of traffic lights that operate in coordination.
#[cfg_attr(
    carla_version_0916,
    doc = " See [OpenDRIVE specification](https://carla.readthedocs.io/en/0.9.16/) for details."
)]
#[cfg_attr(
    carla_version_0915,
    doc = " See [OpenDRIVE specification](https://carla.readthedocs.io/en/0.9.15/) for details."
)]
#[cfg_attr(
    carla_version_0914,
    doc = " See [OpenDRIVE specification](https://carla.readthedocs.io/en/0.9.14/) for details."
)]
pub type ContId = String;

pub mod element;
