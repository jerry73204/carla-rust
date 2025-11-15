//! Road network and navigation types based on OpenDRIVE.
//!
//! This module provides types for working with CARLA's road network topology,
//! which follows the [OpenDRIVE](https://www.asam.net/standards/detail/opendrive/)
//! standard. It corresponds to the `carla.libcarla` road-related classes in the
//! Python API.
//!
//! # Key Types
//!
//! - [`client::Waypoint`](crate::client::Waypoint) - Points on the road for navigation
//! - [`client::Landmark`](crate::client::Landmark) - Road signs and traffic signals
//! - Road identifiers: [`RoadId`], [`LaneId`], [`JuncId`], [`SectionId`]
//! - [`LaneType`] - Lane classification (driving, parking, sidewalk, shoulder, etc.)
//! - [`SignalOrientation`] - Traffic signal orientation
//! - [`element::LaneMarking`] - Lane marking properties
//!
//! # Navigation
//!
//! Waypoints are the primary tool for road-following navigation. Use
//! [`client::Map`](crate::client::Map) methods to query and navigate the road network:
//! - [`client::Map::generate_waypoints()`](crate::client::Map::generate_waypoints) - Get all waypoints
//! - [`client::Map::waypoint_at()`](crate::client::Map::waypoint_at) - Get waypoint at location
//! - [`client::Waypoint::next()`](crate::client::Waypoint::next) - Get next waypoints
//!
//! # OpenDRIVE Concepts
//!
//! The road network is organized hierarchically:
//! - **Road** ([`RoadId`]) - A stretch of road with multiple lanes
//! - **Section** ([`SectionId`]) - A segment of a road with consistent lane configuration
//! - **Lane** ([`LaneId`]) - Individual lanes within a section
//! - **Junction** ([`JuncId`]) - Intersection connecting multiple roads
//!
//! # Python API Reference
//!
//! See the [Python API documentation](https://carla.readthedocs.io/en/0.9.16/python_api/#carla-waypoint)
//! for the Python equivalent types, particularly:
//! - [carla.Waypoint](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Waypoint)
//! - [carla.Landmark](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Landmark)
//! - [carla.LaneMarking](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LaneMarking)

pub use carla_sys::carla::road::{
    JuncId, LaneId, Lane_LaneType as LaneType, RoadId, SectionId, SignalOrientation,
};

/// Traffic sign ID (string identifier).
pub type SignId = String;
/// Controller ID (string identifier).
pub type ContId = String;

pub mod element;
