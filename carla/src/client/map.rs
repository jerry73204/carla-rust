// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use core::slice;

use super::{Junction, Landmark, LandmarkList, Waypoint, WaypointList};
use crate::{
    geom::{Location, Transform},
    road::{LaneId, LaneType, RoadId},
};
use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::client::{FfiMap, FfiTransformList};
use cxx::{SharedPtr, UniquePtr};
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Represents the map of the simulation.
///
/// Corresponds to [`carla.Map`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map) in the Python API.
///
/// The [`Map`] provides access to the road network based on the OpenDRIVE standard.
/// It allows you to query waypoints, spawn points, landmarks, and road topology.
///
/// # Key Features
///
/// - **Navigation**: Generate waypoints for path planning
/// - **Spawn points**: Get recommended vehicle spawn locations
/// - **Road network**: Query road topology, lanes, and junctions
/// - **Landmarks**: Access traffic signs, lights, and road markings
/// - **OpenDRIVE**: Export raw OpenDRIVE XML description
///
/// # Examples
///
/// ```no_run
/// use carla::client::Client;
/// use nalgebra::Translation3;
///
/// let client = Client::default();
/// let world = client.world();
/// let map = world.map();
///
/// // Get map name
/// println!("Map: {}", map.name());
///
/// // Get recommended spawn points
/// let spawn_points = map.recommended_spawn_points();
/// if let Some(spawn_point) = spawn_points.get(0) {
///     println!("First spawn point: {:?}", spawn_point);
/// }
///
/// // Generate waypoints for navigation (every 2 meters)
/// let waypoints = map.generate_waypoints(2.0);
/// println!("Generated {} waypoints", waypoints.len());
///
/// // Find nearest waypoint to a location
/// let location = Translation3::new(100.0, 200.0, 1.0);
/// if let Some(waypoint) = map.waypoint(&location) {
///     println!("Nearest waypoint on lane {}", waypoint.lane_id());
/// }
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Map {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiMap>,
}

impl Map {
    /// Returns the name of the map (e.g., "Town01", "Town02").
    ///
    /// See [carla.Map.name](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map.name)
    /// in the Python API.
    pub fn name(&self) -> String {
        self.inner.GetName().to_string()
    }

    /// Returns the raw OpenDRIVE XML description of the map.
    ///
    /// OpenDRIVE is the standard format for road network descriptions.
    ///
    /// See [carla.Map.to_opendrive](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map.to_opendrive)
    /// in the Python API.
    pub fn to_open_drive(&self) -> String {
        self.inner.GetOpenDrive().to_string()
    }

    /// Returns recommended spawn points for vehicles.
    ///
    /// These are predefined locations suitable for spawning vehicles without collisions.
    ///
    /// See [carla.Map.get_spawn_points](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map.get_spawn_points)
    /// in the Python API.
    pub fn recommended_spawn_points(&self) -> RecommendedSpawnPoints {
        let ptr = self.inner.GetRecommendedSpawnPoints().within_unique_ptr();
        unsafe { RecommendedSpawnPoints::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Finds the nearest waypoint on a driving lane.
    ///
    /// See [carla.Map.get_waypoint](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map.get_waypoint)
    /// in the Python API.
    ///
    /// # Arguments
    ///
    /// * `location` - The 3D position to search from
    ///
    /// # Returns
    ///
    /// The nearest waypoint on a driving lane, or `None` if no waypoint is found.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # use carla::geom::Location;
    /// let client = Client::default();
    /// let world = client.world();
    /// let map = world.map();
    ///
    /// let location = Location::new(100.0, 50.0, 0.5);
    /// if let Some(waypoint) = map.waypoint_at(&location) {
    ///     println!("Found waypoint at road {}", waypoint.road_id());
    /// }
    /// ```
    pub fn waypoint_at(&self, location: &Location) -> Option<Waypoint> {
        let ptr = self
            .inner
            .GetWaypoint(location.as_ffi(), true, LaneType::Driving as i32);
        Waypoint::from_cxx(ptr)
    }

    /// Finds the nearest waypoint with custom options.
    ///
    /// See [carla.Map.get_waypoint](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map.get_waypoint)
    /// in the Python API.
    ///
    /// # Arguments
    ///
    /// * `location` - 3D position to query from
    /// * `project_to_road` - If true, projects the location onto the road surface
    /// * `lane_type` - Filter by lane type (driving, parking, sidewalk, etc.)
    ///
    /// # Returns
    ///
    /// The nearest waypoint matching the criteria, or `None` if not found.
    pub fn waypoint_opt(
        &self,
        location: &Location,
        project_to_road: bool,
        lane_type: LaneType,
    ) -> Option<Waypoint> {
        let ptr = self
            .inner
            .GetWaypoint(location.as_ffi(), project_to_road, lane_type as i32);
        Waypoint::from_cxx(ptr)
    }

    /// Gets a waypoint by OpenDRIVE coordinates.
    ///
    /// See [carla.Map.get_waypoint_xodr](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map.get_waypoint_xodr)
    /// in the Python API.
    ///
    /// # Arguments
    ///
    /// * `road_id` - Road identifier from OpenDRIVE
    /// * `lane_id` - Lane identifier (negative for right lanes, positive for left)
    /// * `distance` - Distance along the road (meters from road start)
    pub fn waypoint_xodr(
        &self,
        road_id: RoadId,
        lane_id: LaneId,
        distance: f32,
    ) -> Option<Waypoint> {
        let ptr = self.inner.GetWaypointXODR(road_id, lane_id, distance);
        Waypoint::from_cxx(ptr)
    }

    /// Generates a grid of waypoints covering the entire road network.
    ///
    /// See [carla.Map.generate_waypoints](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map.generate_waypoints)
    /// in the Python API.
    ///
    /// # Arguments
    ///
    /// * `distance` - Approximate distance between waypoints (meters)
    ///
    /// # Returns
    ///
    /// A list of waypoints covering all drivable lanes.
    pub fn generate_waypoints(&self, distance: f64) -> WaypointList {
        let waypoints = self.inner.GenerateWaypoints(distance);
        unsafe { WaypointList::from_cxx(waypoints).unwrap_unchecked() }
    }

    /// Gets the junction associated with a waypoint.
    ///
    /// See [carla.Map.get_junction](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map.get_junction)
    /// in the Python API.
    ///
    /// # Arguments
    ///
    /// * `waypoint` - Waypoint to query (should be inside a junction)
    pub fn junction(&self, waypoint: &Waypoint) -> Junction {
        let junction = self.inner.GetJunction(&waypoint.inner);
        unsafe { Junction::from_cxx(junction).unwrap_unchecked() }
    }

    /// Returns all landmarks in the map (traffic signs, lights, etc.).
    ///
    /// See [carla.Map.get_all_landmarks](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map.get_all_landmarks)
    /// in the Python API.
    pub fn all_landmarks(&self) -> LandmarkList {
        let ptr = self.inner.GetAllLandmarks().within_unique_ptr();
        unsafe { LandmarkList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Finds landmarks by their OpenDRIVE ID.
    ///
    /// See [carla.Map.get_landmark_from_id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map.get_landmark_from_id)
    /// in the Python API.
    ///
    /// # Arguments
    ///
    /// * `id` - Landmark identifier from OpenDRIVE
    pub fn landmarks_from_id(&self, id: &str) -> LandmarkList {
        let ptr = self.inner.GetLandmarksFromId(id).within_unique_ptr();
        unsafe { LandmarkList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Finds all landmarks of a specific type.
    ///
    /// See [carla.Map.get_all_landmarks_of_type](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map.get_all_landmarks_of_type)
    /// in the Python API.
    ///
    /// # Arguments
    ///
    /// * `type_` - Landmark type (e.g., "1000001" for speed limit signs)
    pub fn all_landmarks_of_type(&self, type_: &str) -> LandmarkList {
        let ptr = self.inner.GetAllLandmarksOfType(type_).within_unique_ptr();
        unsafe { LandmarkList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Gets all landmarks in the same group as the given landmark.
    ///
    /// See [carla.Map.get_landmark_group](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map.get_landmark_group)
    /// in the Python API.
    ///
    /// # Arguments
    ///
    /// * `landmark` - Reference landmark
    pub fn landmark_group(&self, landmark: &Landmark) -> LandmarkList {
        let ptr = self
            .inner
            .GetLandmarkGroup(&landmark.inner)
            .within_unique_ptr();
        unsafe { LandmarkList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns the road network topology as pairs of connected waypoints.
    ///
    /// The topology describes the connectivity of the road network, where each
    /// pair represents a connection from one waypoint to another. This is useful
    /// for path planning and understanding the road network structure.
    ///
    /// See [carla.Map.get_topology](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Map.get_topology)
    /// in the Python API.
    ///
    /// # Returns
    ///
    /// A vector of waypoint pairs, where each pair `(start, end)` represents
    /// a directed connection in the road network.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// let world = client.world();
    /// let map = world.map();
    ///
    /// let topology = map.topology();
    /// println!("Road network has {} connections", topology.len());
    ///
    /// // Examine first connection
    /// if let Some((start, end)) = topology.first() {
    ///     println!(
    ///         "Connection: lane {} -> lane {}",
    ///         start.lane_id(),
    ///         end.lane_id()
    ///     );
    /// }
    /// ```
    pub fn topology(&self) -> Vec<(Waypoint, Waypoint)> {
        let topology_pairs = self.inner.GetTopology();
        let mut result = Vec::with_capacity(topology_pairs.len());

        for pair in topology_pairs.iter() {
            let first = pair.first();
            let second = pair.second();

            // Skip null waypoints in topology
            if let (Some(start), Some(end)) =
                (Waypoint::from_cxx(first), Waypoint::from_cxx(second))
            {
                result.push((start, end));
            }
        }

        result
    }
}

impl Map {
    pub fn from_cxx(ptr: SharedPtr<FfiMap>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

/// List of recommended spawn points for vehicles.
///
/// These are predefined transforms (position + rotation) that are suitable
/// for spawning vehicles without immediate collisions.
///
/// # Examples
///
/// ```no_run
/// use carla::client::Client;
///
/// let client = Client::default();
/// let world = client.world();
/// let map = world.map();
///
/// let spawn_points = map.recommended_spawn_points();
/// println!("Available spawn points: {}", spawn_points.len());
///
/// // Use the first spawn point
/// if let Some(transform) = spawn_points.get(0) {
///     println!("Spawn at: {:?}", transform.translation);
/// }
///
/// // Iterate over all spawn points
/// for (i, transform) in spawn_points.iter().enumerate().take(5) {
///     println!("Spawn point {}: {:?}", i, transform);
/// }
/// ```
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct RecommendedSpawnPoints {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiTransformList>,
}

impl RecommendedSpawnPoints {
    /// Returns the number of spawn points.
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    /// Returns true if there are no spawn points.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the spawn points as a slice.
    pub fn as_slice(&self) -> &[Transform] {
        // Safety: Transform is #[repr(transparent)] over FfiTransform,
        // so pointer cast is safe
        unsafe { slice::from_raw_parts(self.inner.data() as *const Transform, self.len()) }
    }

    /// Gets the spawn point at the given index.
    ///
    /// Returns `None` if the index is out of bounds.
    pub fn get(&self, index: usize) -> Option<&Transform> {
        self.as_slice().get(index)
    }

    /// Returns an iterator over all spawn points.
    pub fn iter(&self) -> impl Iterator<Item = &Transform> + Send + '_ {
        self.as_slice().iter()
    }

    fn from_cxx(ptr: UniquePtr<FfiTransformList>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

assert_impl_all!(Map: Send, Sync);

assert_impl_all!(RecommendedSpawnPoints: Send, Sync);
