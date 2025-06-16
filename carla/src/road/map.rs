//! Map representation and utilities.

use super::{Junction, Waypoint};
use crate::{
    error::CarlaResult,
    geom::{Location, Transform},
};

/// Represents the road map.
#[derive(Debug)]
pub struct Map {
    /// Internal handle to carla-cxx Map
    inner: carla_cxx::MapWrapper,
}

impl Map {
    /// Create a new Map from a carla-cxx MapWrapper.
    pub fn new(inner: carla_cxx::MapWrapper) -> Self {
        Self { inner }
    }

    /// Get the name of this map.
    pub fn name(&self) -> String {
        self.inner.get_name()
    }

    /// Get a waypoint at the given location.
    pub fn waypoint(&self, location: Location) -> CarlaResult<Option<Waypoint>> {
        // TODO: Implement using carla-cxx FFI interface
        let _location = location;
        todo!("Map::waypoint not yet implemented with carla-cxx FFI")
    }

    /// Get all spawn points for vehicles.
    pub fn spawn_points(&self) -> CarlaResult<Vec<Transform>> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Map::spawn_points not yet implemented with carla-cxx FFI")
    }

    /// Generate waypoints along the road network.
    pub fn generate_waypoints(&self, distance: f64) -> CarlaResult<Vec<Waypoint>> {
        // TODO: Implement using carla-cxx FFI interface
        let _distance = distance;
        todo!("Map::generate_waypoints not yet implemented with carla-cxx FFI")
    }

    /// Get the OpenDRIVE file contents.
    pub fn to_opendrive(&self) -> String {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Map::to_opendrive not yet implemented with carla-cxx FFI")
    }

    /// Save the map as OpenDRIVE file.
    pub fn save_to_disk(&self, path: &str) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _path = path;
        todo!("Map::save_to_disk not yet implemented with carla-cxx FFI")
    }

    /// Get topology of the road network.
    pub fn topology(&self) -> CarlaResult<Vec<(Waypoint, Waypoint)>> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Map::topology not yet implemented with carla-cxx FFI")
    }

    /// Get crosswalks in the map.
    pub fn crosswalks(&self) -> CarlaResult<Vec<Location>> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Map::crosswalks not yet implemented with carla-cxx FFI")
    }

    /// Get all junctions in the map.
    pub fn junctions(&self) -> CarlaResult<Vec<Junction>> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Map::junctions not yet implemented with carla-cxx FFI")
    }

    /// Transform a location from geospatial (lat/lon) to map coordinates.
    pub fn transform_to_geolocation(&self, location: &Location) -> CarlaResult<GeoLocation> {
        // TODO: Implement using carla-cxx FFI interface
        let _location = location;
        todo!("Map::transform_to_geolocation not yet implemented with carla-cxx FFI")
    }

    /// Transform a geospatial location (lat/lon) to map coordinates.
    pub fn transform_from_geolocation(&self, geo_location: &GeoLocation) -> CarlaResult<Location> {
        // TODO: Implement using carla-cxx FFI interface
        let _geo_location = geo_location;
        todo!("Map::transform_from_geolocation not yet implemented with carla-cxx FFI")
    }
}

/// Geospatial location (latitude/longitude).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoLocation {
    /// Latitude in degrees
    pub latitude: f64,
    /// Longitude in degrees
    pub longitude: f64,
    /// Altitude in meters
    pub altitude: f64,
}

impl GeoLocation {
    /// Create a new geospatial location.
    pub fn new(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self {
            latitude,
            longitude,
            altitude,
        }
    }
}
