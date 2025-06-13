//! Road network and OpenDRIVE functionality.
//!
//! This module provides types and utilities for working with CARLA's road network,
//! including OpenDRIVE parsing, lane markings, and road topology.
//!
//! The module is organized into logical submodules:
//! - `element` - Road elements like lane markings and topology
//! - `opendrive` - OpenDRIVE map generation and parsing

// TODO: Update road module to use new C API functions when available
// pub use carla_sys::carla::road::{
//     JuncId, LaneId, Lane_LaneType as LaneType, RoadId, SectionId, SignalOrientation,
// };

// Road network submodules
pub mod element;
// TODO: Re-enable opendrive module when C API types are available
// pub mod opendrive;

// Re-export commonly used types from submodules
pub use element::LaneMarking;
// TODO: Re-enable when opendrive module is ready
// pub use opendrive::OpenDriveGenerator;

// Basic road types for now - will be updated when C API is integrated
pub type JuncId = u32;
pub type LaneId = i32;
pub type RoadId = u32;
pub type SectionId = u32;
pub type SignId = String;
pub type ContId = String;

#[derive(Debug, Clone, PartialEq)]
pub enum LaneType {
    None,
    Driving,
    Stop,
    Shoulder,
    Biking,
    Sidewalk,
    // TODO: Add more lane types when C API is integrated
}

#[derive(Debug, Clone, PartialEq)]
pub enum SignalOrientation {
    Positive,
    Negative,
    Both,
}
