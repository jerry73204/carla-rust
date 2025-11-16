pub use carla_sys::carla::road::{
    JuncId, LaneId, Lane_LaneType as LaneType, RoadId, SectionId, SignalOrientation,
};

/// Traffic sign ID (string identifier).
pub type SignId = String;
/// Controller ID (string identifier).
pub type ContId = String;

pub mod element;
