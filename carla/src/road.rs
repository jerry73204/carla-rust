//! Road network and navigation types.
//!
//! This module provides types for working with CARLA's road network, based on
//! the OpenDRIVE standard:
//!
//! - Waypoint - Points on the road for navigation
//! - Landmark - Road signs and traffic lights
//! - Road identifiers: [`RoadId`], [`LaneId`], [`JuncId`], [`SectionId`]
//! - [`LaneType`] - Lane classification (driving, parking, sidewalk, etc.)
//!
//! # Navigation
//!
//! Waypoints are the primary tool for road-following navigation. Use
//! [`client::Map::generate_waypoints()`](crate::client::Map::generate_waypoints)
//! to navigate the road network.

pub use carla_sys::carla::road::{
    JuncId, LaneId, Lane_LaneType as LaneType, RoadId, SectionId, SignalOrientation,
};

/// Traffic sign ID (string identifier).
pub type SignId = String;
/// Controller ID (string identifier).
pub type ContId = String;

pub mod element;
