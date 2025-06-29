//! # CARLA Road Network and Navigation
//!
//! This module provides comprehensive access to CARLA's road network information,
//! enabling advanced navigation, route planning, and traffic analysis. It mirrors
//! CARLA's `carla::road` namespace and integrates with OpenDRIVE road definitions.
//!
//! ## Core Components
//!
//! ### Map System
//! - [`Map`]: Complete road network representation
//! - [`Waypoint`]: Specific points along lanes with navigation context
//! - [`WaypointList`]: Collections of waypoints for efficient batch operations
//!
//! ### Road Infrastructure
//! - [`Junction`]: Intersections and complex road connections
//! - [`Lane`]: Individual driving lanes with properties and markings
//! - [`Landmark`]: Traffic signs, signals, and road infrastructure
//!
//! ### Navigation Tools
//! - [`Navigator`]: Pathfinding and route planning utilities
//! - [`Route`]: Planned paths between waypoints
//! - [`PathfindingOptions`]: Configuration for route planning algorithms
//!
//! ## Usage Examples
//!
//! ### Basic Map Operations
//! ```rust,no_run
//! use carla::{client::Client, geom::Location};
//!
//! # fn map_example() -> carla::error::CarlaResult<()> {
//! let client = Client::new("localhost", 2000, None)?;
//! let world = client.world()?;
//! let map = world.map()?;
//!
//! // Get waypoint at a location
//! let location = Location::new(10.0, 20.0, 0.0);
//! let waypoint = map.get_waypoint(location, true, None)?;
//!
//! if let Some(wp) = waypoint {
//!     println!("Lane ID: {}, Road ID: {}", wp.lane_id, wp.road_id);
//!     println!("Lane width: {:.2}m", wp.lane_width);
//!     println!("Is junction: {}", wp.is_junction());
//! }
//!
//! // Get spawn points
//! let spawn_points = map.spawn_points()?;
//! println!("Available spawn points: {}", spawn_points.len());
//! # Ok(())
//! # }
//! ```
//!
//! ### Waypoint Navigation
//! ```rust,no_run
//! # use carla::road::Waypoint;
//! # fn waypoint_example(waypoint: &Waypoint) -> carla::error::CarlaResult<()> {
//! // Get next waypoints along the lane
//! let next_waypoints = waypoint.next(5.0)?; // 5 meters ahead
//! for next_wp in next_waypoints.iter() {
//!     println!("Next waypoint at: {:?}", next_wp.transform.location);
//! }
//!
//! // Lane changing
//! if let Some(left_lane) = waypoint.left_lane()? {
//!     println!("Can change to left lane");
//! }
//!
//! if let Some(right_lane) = waypoint.right_lane()? {
//!     println!("Can change to right lane");
//! }
//!
//! // Distance to another waypoint
//! # let other_waypoint = waypoint; // placeholder
//! let distance = waypoint.distance(other_waypoint);
//! println!("Distance: {:.2}m", distance);
//! # Ok(())
//! # }
//! ```
//!
//! ### Route Planning
//! ```rust,no_run
//! use carla::{
//!     geom::Location,
//!     road::{Navigator, PathfindingOptions},
//! };
//!
//! # fn pathfinding_example() -> carla::error::CarlaResult<()> {
//! # let map = todo!(); // Would get from world
//!
//! let navigator = Navigator::new(map);
//!
//! // Find route between two locations
//! let start = Location::new(0.0, 0.0, 0.0);
//! let end = Location::new(100.0, 50.0, 0.0);
//!
//! let options = PathfindingOptions {
//!     max_distance: 1000.0,
//!     prefer_highways: true,
//!     avoid_toll_roads: false,
//! };
//!
//! if let Some(route) = navigator.find_route(start, end, Some(options))? {
//!     println!("Route found with {} waypoints", route.waypoints().len());
//!     println!("Total distance: {:.2}m", route.total_distance());
//!
//!     // Follow the route
//!     for (i, waypoint) in route.waypoints().iter().enumerate() {
//!         println!("Step {}: {:?}", i, waypoint.transform.location);
//!     }
//! }
//! # Ok(())
//! # }
//! ```
//!
//! ### Landmark Detection
//! ```rust,no_run
//! # use carla::road::Waypoint;
//! # fn landmark_example(waypoint: &Waypoint) -> carla::error::CarlaResult<()> {
//! // Find all landmarks within 50 meters
//! let landmarks = waypoint.landmarks(50.0)?;
//!
//! for landmark in landmarks {
//!     println!(
//!         "Landmark: {} at {:?}",
//!         landmark.landmark_type, landmark.transform.location
//!     );
//! }
//!
//! // Find specific types of landmarks
//! let traffic_lights = waypoint.landmarks_of_type("traffic_light", 30.0)?;
//! let stop_signs = waypoint.landmarks_of_type("stop", 20.0)?;
//!
//! println!(
//!     "Found {} traffic lights, {} stop signs",
//!     traffic_lights.len(),
//!     stop_signs.len()
//! );
//! # Ok(())
//! # }
//! ```
//!
//! ### Junction Handling
//! ```rust,no_run
//! # use carla::road::Waypoint;
//! # fn junction_example(waypoint: &Waypoint) -> carla::error::CarlaResult<()> {
//! if waypoint.is_junction() {
//!     if let Some(junction) = waypoint.junction()? {
//!         println!("In junction with ID: {}", junction.id());
//!
//!         // Get waypoints for each possible path through junction
//!         let junction_waypoints = junction.waypoints()?;
//!         println!("Junction has {} path options", junction_waypoints.len());
//!     }
//! }
//! # Ok(())
//! # }
//! ```
//!
//! ## Lane Types and Properties
//!
//! CARLA supports various lane types for realistic road modeling:
//!
//! ```rust
//! use carla::road::{LaneChange, LaneMarkingColor, LaneMarkingType, LaneType};
//!
//! fn analyze_lane_properties(waypoint: &carla::road::Waypoint) {
//!     match waypoint.lane_type {
//!         LaneType::Driving => println!("Normal driving lane"),
//!         LaneType::Shoulder => println!("Road shoulder"),
//!         LaneType::Sidewalk => println!("Pedestrian sidewalk"),
//!         LaneType::Biking => println!("Bicycle lane"),
//!         LaneType::OnRamp => println!("Highway on-ramp"),
//!         LaneType::OffRamp => println!("Highway off-ramp"),
//!         _ => println!("Other lane type: {:?}", waypoint.lane_type),
//!     }
//!
//!     match waypoint.lane_change {
//!         LaneChange::Both => println!("Can change lanes in both directions"),
//!         LaneChange::Left => println!("Can change to left lane only"),
//!         LaneChange::Right => println!("Can change to right lane only"),
//!         LaneChange::None => println!("No lane changes allowed"),
//!     }
//!
//!     println!(
//!         "Lane marking: {:?} {:?}",
//!         waypoint.lane_marking_type, waypoint.lane_marking_color
//!     );
//! }
//! ```
//!
//! ## OpenDRIVE Integration
//!
//! CARLA's road network is based on the OpenDRIVE standard:
//!
//! - **Road Elements**: Roads, lanes, junctions defined by OpenDRIVE specification
//! - **Coordinate System**: Uses OpenDRIVE's s-t coordinate system internally
//! - **Geometry**: Supports OpenDRIVE geometry primitives (lines, arcs, spirals)
//! - **Semantics**: Lane types, markings, and properties follow OpenDRIVE standards
//!
//! ## Performance Considerations
//!
//! - **Waypoint Caching**: Waypoints are cached by the map for performance
//! - **Batch Operations**: Use `WaypointList` for processing multiple waypoints
//! - **Distance Queries**: Spatial queries are optimized using internal data structures
//! - **Route Caching**: Consider caching frequently used routes
//!
//! ## Navigation Algorithms
//!
//! The pathfinding system supports multiple algorithms:
//!
//! - **Breadth-First Search**: For shortest path in terms of waypoint count
//! - **Dijkstra's Algorithm**: For shortest path by distance
//! - **A* Search**: For heuristic-guided pathfinding (future)
//! - **Lane-Aware Planning**: Considers lane change costs and restrictions
//!
//! ## Common Use Cases
//!
//! - **Autonomous Vehicle Navigation**: Route planning and lane following
//! - **Traffic Analysis**: Road network topology analysis
//! - **Map Validation**: Checking road network connectivity
//! - **Scenario Generation**: Creating realistic driving scenarios
//! - **Infrastructure Planning**: Analyzing road network properties

mod element;
mod junction;
mod landmark;
mod lane;
mod map;
mod pathfinding;
mod road_types;
mod waypoint;

pub use element::*;
pub use junction::*;
pub use landmark::*;
pub use lane::*;
pub use map::*;
pub use pathfinding::*;
pub use road_types::*;
pub use waypoint::*;
