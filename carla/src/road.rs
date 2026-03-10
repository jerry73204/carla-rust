pub use carla_sys::carla::road::{Lane_LaneType as LaneType, SignalOrientation};

/// Road identifier type. Matches C++ `uint32_t` typedef.
pub type RoadId = u32;
/// Junction identifier type. Matches C++ `int32_t` typedef.
pub type JuncId = i32;
/// Road section identifier type. Matches C++ `uint32_t` typedef.
pub type SectionId = u32;
/// Lane identifier type. Matches C++ `int32_t` typedef.
pub type LaneId = i32;

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
