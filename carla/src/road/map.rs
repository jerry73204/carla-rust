//! Map representation and utilities.

use super::{Junction, Waypoint};
use crate::{
    error::CarlaResult,
    geom::{Location, Transform},
};
use carla_cxx::map::{
    LocationVector, LocationVectorIterator, TopologyVector, TransformVector,
    TransformVectorIterator, WaypointVector,
};

/// Represents the road map.
#[derive(Debug)]
pub struct Map {
    /// Internal handle to carla-cxx Map
    pub(crate) inner: carla_cxx::MapWrapper,
}

impl Map {
    /// Create a new Map from a carla-cxx MapWrapper.
    pub(crate) fn new(inner: carla_cxx::MapWrapper) -> Self {
        Self { inner }
    }

    /// Get the name of this map.
    pub fn name(&self) -> String {
        self.inner.get_name()
    }

    /// Get a waypoint at the given location.
    pub fn waypoint(&self, location: Location) -> Option<Waypoint> {
        use carla_cxx::ffi::SimpleLocation;
        let simple_location = SimpleLocation {
            x: location.x,
            y: location.y,
            z: location.z,
        };
        let waypoint_wrapper = self.inner.get_waypoint(
            &simple_location,
            true,
            Some(carla_cxx::map::LaneType::Driving),
        );
        waypoint_wrapper.map(|w| Waypoint::from_cxx_wrapper(w))
    }

    /// Get all spawn points for vehicles.
    pub fn spawn_points(&self) -> TransformList {
        let transform_vec = self.inner.get_recommended_spawn_points_vector();
        TransformList::new(transform_vec)
    }

    /// Generate waypoints along the road network.
    pub fn generate_waypoints(&self, distance: f64) -> WaypointList {
        let waypoint_vec = self.inner.generate_waypoints_vector(distance);
        WaypointList::new(waypoint_vec)
    }

    /// Get the OpenDRIVE file contents.
    pub fn to_opendrive(&self) -> String {
        self.inner.get_open_drive()
    }

    /// Get topology of the road network.
    pub fn topology(&self) -> Topology {
        let topology_vec = self.inner.get_topology_vector();
        Topology::new(topology_vec)
    }

    /// Get crosswalks in the map.
    pub fn crosswalks(&self) -> LocationList {
        let location_vec = self.inner.get_all_crosswalk_zones_vector();
        LocationList::new(location_vec)
    }

    /// Get all junctions in the map.
    pub fn junctions(&self) -> Vec<Junction> {
        // Note: carla-cxx doesn't expose a direct get_all_junctions method
        // This would require additional FFI implementation
        todo!("Map::junctions requires additional FFI support for listing all junctions")
    }

    /// Get a junction from a waypoint.
    pub fn junction(&self, waypoint: &Waypoint) -> Option<Junction> {
        self.inner
            .get_junction(waypoint.wrapper())
            .map(|j| Junction::from_cxx_wrapper(j))
    }

    /// Transform a location from geospatial (lat/lon) to map coordinates.
    pub fn transform_to_geolocation(&self, location: &Location) -> GeoLocation {
        let geo_ref = self.inner.get_geo_reference();
        GeoLocation {
            latitude: geo_ref.latitude,
            longitude: geo_ref.longitude,
            altitude: location.z,
        }
    }

    /// Transform a geospatial location (lat/lon) to map coordinates.
    pub fn transform_from_geolocation(&self, geo_location: &GeoLocation) -> CarlaResult<Location> {
        // TODO: Implement using carla-cxx FFI interface
        let _geo_location = geo_location;
        todo!("Map::transform_from_geolocation not yet implemented with carla-cxx FFI")
    }

    /// Get next waypoint from coordinates (for waypoints without wrapper)
    pub fn next_waypoint_from_coordinates(
        &self,
        road_id: u32,
        lane_id: i32,
        s: f64,
        distance: f64,
    ) -> Option<Waypoint> {
        let waypoint_wrapper = self
            .inner
            .get_next_waypoint_xodr(road_id, lane_id, s, distance);
        waypoint_wrapper.map(|w| Waypoint::from_cxx_wrapper(w))
    }

    /// Get previous waypoint from coordinates (for waypoints without wrapper)
    pub fn previous_waypoint_from_coordinates(
        &self,
        road_id: u32,
        lane_id: i32,
        s: f64,
        distance: f64,
    ) -> Option<Waypoint> {
        let waypoint_wrapper = self
            .inner
            .get_previous_waypoint_xodr(road_id, lane_id, s, distance);
        waypoint_wrapper.map(|w| Waypoint::from_cxx_wrapper(w))
    }

    /// Get right lane waypoint from coordinates (for waypoints without wrapper)
    pub fn right_lane_waypoint_from_coordinates(
        &self,
        road_id: u32,
        lane_id: i32,
        s: f64,
    ) -> Option<Waypoint> {
        let waypoint_wrapper = self.inner.get_right_lane_waypoint_xodr(road_id, lane_id, s);
        waypoint_wrapper.map(|w| Waypoint::from_cxx_wrapper(w))
    }

    /// Get left lane waypoint from coordinates (for waypoints without wrapper)
    pub fn left_lane_waypoint_from_coordinates(
        &self,
        road_id: u32,
        lane_id: i32,
        s: f64,
    ) -> Option<Waypoint> {
        let waypoint_wrapper = self.inner.get_left_lane_waypoint_xodr(road_id, lane_id, s);
        waypoint_wrapper.map(|w| Waypoint::from_cxx_wrapper(w))
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

/// Represents the topology of the road network.
///
/// The topology is a collection of waypoint pairs representing
/// connections between different road segments.
pub struct Topology {
    inner: TopologyVector,
}

impl Topology {
    /// Create a new Topology from a carla-cxx TopologyVector.
    pub(crate) fn new(inner: TopologyVector) -> Self {
        Self { inner }
    }

    /// Get the number of topology pairs.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Check if the topology is empty.
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Get a topology pair at the specified index.
    pub fn get(&self, index: usize) -> Option<(Waypoint, Waypoint)> {
        self.inner.get(index).map(|(start, end)| {
            (
                Waypoint::from_cxx_wrapper(start),
                Waypoint::from_cxx_wrapper(end),
            )
        })
    }

    /// Iterate over all topology pairs.
    pub fn iter(&self) -> TopologyIterator {
        TopologyIterator {
            topology: self,
            index: 0,
        }
    }

    /// Convert to a Vec of waypoint pairs.
    pub fn to_vec(self) -> Vec<(Waypoint, Waypoint)> {
        self.inner
            .into_iter()
            .map(|(start, end)| {
                (
                    Waypoint::from_cxx_wrapper(start),
                    Waypoint::from_cxx_wrapper(end),
                )
            })
            .collect()
    }
}

impl IntoIterator for Topology {
    type Item = (Waypoint, Waypoint);
    type IntoIter = TopologyIntoIterator;

    fn into_iter(self) -> Self::IntoIter {
        TopologyIntoIterator {
            inner: self.inner.into_iter(),
        }
    }
}

/// Iterator over topology pairs (borrowed).
pub struct TopologyIterator<'a> {
    topology: &'a Topology,
    index: usize,
}

impl<'a> Iterator for TopologyIterator<'a> {
    type Item = (Waypoint, Waypoint);

    fn next(&mut self) -> Option<Self::Item> {
        let item = self.topology.get(self.index);
        if item.is_some() {
            self.index += 1;
        }
        item
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.topology.len() - self.index;
        (remaining, Some(remaining))
    }
}

/// Iterator over topology pairs (owned).
pub struct TopologyIntoIterator {
    inner: carla_cxx::map::TopologyVectorIterator,
}

impl Iterator for TopologyIntoIterator {
    type Item = (Waypoint, Waypoint);

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next().map(|(start, end)| {
            (
                Waypoint::from_cxx_wrapper(start),
                Waypoint::from_cxx_wrapper(end),
            )
        })
    }
}

/// A list of waypoints.
///
/// This is a wrapper around the C++ WaypointVector that provides
/// efficient access to waypoints without conversion.
pub struct WaypointList {
    inner: WaypointVector,
}

impl WaypointList {
    /// Create a new WaypointList from a WaypointVector.
    pub(crate) fn new(inner: WaypointVector) -> Self {
        Self { inner }
    }

    /// Get the number of waypoints.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Check if the list is empty.
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Get a waypoint at the specified index.
    pub fn get(&self, index: usize) -> Option<Waypoint> {
        self.inner.get(index).map(|w| Waypoint::from_cxx_wrapper(w))
    }

    /// Iterate over the waypoints.
    pub fn iter(&self) -> WaypointListIterator {
        WaypointListIterator {
            list: self,
            index: 0,
        }
    }

    /// Convert to a Vec of Waypoints.
    pub fn to_vec(self) -> Vec<Waypoint> {
        self.inner
            .into_iter()
            .map(|w| Waypoint::from_cxx_wrapper(w))
            .collect()
    }
}

impl IntoIterator for WaypointList {
    type Item = Waypoint;
    type IntoIter = WaypointListIntoIterator;

    fn into_iter(self) -> Self::IntoIter {
        WaypointListIntoIterator {
            inner: self.inner.into_iter(),
        }
    }
}

/// Iterator over waypoints (borrowed).
pub struct WaypointListIterator<'a> {
    list: &'a WaypointList,
    index: usize,
}

impl<'a> Iterator for WaypointListIterator<'a> {
    type Item = Waypoint;

    fn next(&mut self) -> Option<Self::Item> {
        let item = self.list.get(self.index);
        if item.is_some() {
            self.index += 1;
        }
        item
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.list.len() - self.index;
        (remaining, Some(remaining))
    }
}

/// Iterator over waypoints (owned).
pub struct WaypointListIntoIterator {
    inner: carla_cxx::map::WaypointVectorIterator,
}

impl Iterator for WaypointListIntoIterator {
    type Item = Waypoint;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next().map(|w| Waypoint::from_cxx_wrapper(w))
    }
}

/// A list of transforms.
///
/// This is a wrapper around the C++ TransformVector that provides
/// efficient access to transforms without conversion.
pub struct TransformList {
    inner: TransformVector,
}

impl TransformList {
    /// Create a new TransformList from a TransformVector.
    pub(crate) fn new(inner: TransformVector) -> Self {
        Self { inner }
    }

    /// Get the number of transforms.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Check if the list is empty.
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Get a transform at the specified index.
    pub fn get(&self, index: usize) -> Option<Transform> {
        self.inner.get(index).map(|t| Transform::from(t))
    }

    /// Iterate over the transforms.
    pub fn iter(&self) -> TransformListIterator {
        TransformListIterator {
            list: self,
            index: 0,
        }
    }

    /// Convert to a Vec of Transforms.
    pub fn to_vec(self) -> Vec<Transform> {
        self.inner.into_iter().map(|t| Transform::from(t)).collect()
    }
}

impl IntoIterator for TransformList {
    type Item = Transform;
    type IntoIter = TransformListIntoIterator;

    fn into_iter(self) -> Self::IntoIter {
        TransformListIntoIterator {
            inner: self.inner.into_iter(),
        }
    }
}

/// Iterator over transforms (borrowed).
pub struct TransformListIterator<'a> {
    list: &'a TransformList,
    index: usize,
}

impl<'a> Iterator for TransformListIterator<'a> {
    type Item = Transform;

    fn next(&mut self) -> Option<Self::Item> {
        let item = self.list.get(self.index);
        if item.is_some() {
            self.index += 1;
        }
        item
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.list.len() - self.index;
        (remaining, Some(remaining))
    }
}

/// Iterator over transforms (owned).
pub struct TransformListIntoIterator {
    inner: TransformVectorIterator,
}

impl Iterator for TransformListIntoIterator {
    type Item = Transform;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next().map(|t| Transform::from(t))
    }
}

/// A list of locations.
///
/// This is a wrapper around the C++ LocationVector that provides
/// efficient access to locations without conversion.
pub struct LocationList {
    inner: LocationVector,
}

impl LocationList {
    /// Create a new LocationList from a LocationVector.
    pub(crate) fn new(inner: LocationVector) -> Self {
        Self { inner }
    }

    /// Get the number of locations.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Check if the list is empty.
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Get a location at the specified index.
    pub fn get(&self, index: usize) -> Option<Location> {
        self.inner
            .get(index)
            .map(|loc| Location::new(loc.x, loc.y, loc.z))
    }

    /// Iterate over the locations.
    pub fn iter(&self) -> LocationListIterator {
        LocationListIterator {
            list: self,
            index: 0,
        }
    }

    /// Convert to a Vec of Locations.
    pub fn to_vec(self) -> Vec<Location> {
        self.inner
            .into_iter()
            .map(|loc| Location::new(loc.x, loc.y, loc.z))
            .collect()
    }
}

impl IntoIterator for LocationList {
    type Item = Location;
    type IntoIter = LocationListIntoIterator;

    fn into_iter(self) -> Self::IntoIter {
        LocationListIntoIterator {
            inner: self.inner.into_iter(),
        }
    }
}

/// Iterator over locations (borrowed).
pub struct LocationListIterator<'a> {
    list: &'a LocationList,
    index: usize,
}

impl<'a> Iterator for LocationListIterator<'a> {
    type Item = Location;

    fn next(&mut self) -> Option<Self::Item> {
        let item = self.list.get(self.index);
        if item.is_some() {
            self.index += 1;
        }
        item
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.list.len() - self.index;
        (remaining, Some(remaining))
    }
}

/// Iterator over locations (owned).
pub struct LocationListIntoIterator {
    inner: LocationVectorIterator,
}

impl Iterator for LocationListIntoIterator {
    type Item = Location;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner
            .next()
            .map(|loc| Location::new(loc.x, loc.y, loc.z))
    }
}
