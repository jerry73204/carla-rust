pub use carla_sys::carla::road::{
    JuncId, LaneId, Lane_LaneType as LaneType, RoadId, SectionId, SignalOrientation,
};
pub type SignId = String;
pub type ContId = String;

pub mod element;
pub mod opendrive;
