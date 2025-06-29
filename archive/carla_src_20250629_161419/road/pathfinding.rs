//! Pathfinding and navigation utilities for the road network.

use super::{Map, Waypoint};
use crate::{error::CarlaResult, geom::Location};
use std::collections::{HashMap, VecDeque};

/// A route between two points on the road network.
#[derive(Debug, Clone)]
pub struct Route {
    /// The waypoints that make up this route
    pub waypoints: Vec<Waypoint>,
    /// Total distance of the route in meters
    pub distance: f64,
}

impl Route {
    /// Create a new route from waypoints.
    pub fn new(waypoints: Vec<Waypoint>) -> Self {
        let distance = if waypoints.len() < 2 {
            0.0
        } else {
            waypoints
                .windows(2)
                .map(|pair| pair[0].distance(&pair[1]))
                .sum()
        };

        Self {
            waypoints,
            distance,
        }
    }

    /// Get the length of the route.
    pub fn len(&self) -> usize {
        self.waypoints.len()
    }

    /// Check if the route is empty.
    pub fn is_empty(&self) -> bool {
        self.waypoints.is_empty()
    }

    /// Get the starting waypoint.
    pub fn start(&self) -> Option<&Waypoint> {
        self.waypoints.first()
    }

    /// Get the ending waypoint.
    pub fn end(&self) -> Option<&Waypoint> {
        self.waypoints.last()
    }
}

/// Navigation utilities for the road network.
pub struct Navigator {
    map: Map,
}

impl Navigator {
    /// Create a new navigator for the given map.
    pub fn new(map: Map) -> Self {
        Self { map }
    }

    /// Find a route between two locations.
    ///
    /// This uses a simple breadth-first search to find a path through the waypoint network.
    pub fn find_route(&self, start: Location, end: Location) -> CarlaResult<Option<Route>> {
        // Get waypoints for start and end locations
        let start_waypoint = self.map.get_waypoint(start, true, None);
        let end_waypoint = self.map.get_waypoint(end, true, None);

        match (start_waypoint, end_waypoint) {
            (Some(start_wp), Some(end_wp)) => self.find_route_between_waypoints(&start_wp, &end_wp),
            _ => Ok(None),
        }
    }

    /// Find a route between two waypoints using breadth-first search.
    pub fn find_route_between_waypoints(
        &self,
        start: &Waypoint,
        end: &Waypoint,
    ) -> CarlaResult<Option<Route>> {
        // Simple BFS pathfinding
        let mut queue = VecDeque::new();
        let mut visited = HashMap::new();
        let mut parent = HashMap::new();

        let start_key = waypoint_key(start);
        let end_key = waypoint_key(end);

        queue.push_back(start_key);
        visited.insert(start_key, start.clone());

        // BFS to find path
        while let Some(current_key) = queue.pop_front() {
            if current_key == end_key {
                // Found the destination, reconstruct path
                let mut path = Vec::new();
                let mut current = current_key;

                while let Some(waypoint) = visited.get(&current) {
                    path.push(waypoint.clone());
                    if let Some(parent_key) = parent.get(&current) {
                        current = *parent_key;
                    } else {
                        break;
                    }
                }

                path.reverse();
                return Ok(Some(Route::new(path)));
            }

            // Get current waypoint and explore neighbors
            if let Some(current_waypoint) = visited.get(&current_key).cloned() {
                // Try to get next waypoints (assuming 5 meter steps)
                if let Ok(next_waypoints) = current_waypoint.next(5.0) {
                    for next_waypoint in next_waypoints.iter() {
                        let next_key = waypoint_key(&next_waypoint);

                        if let std::collections::hash_map::Entry::Vacant(e) =
                            visited.entry(next_key)
                        {
                            queue.push_back(next_key);
                            e.insert(next_waypoint.clone());
                            parent.insert(next_key, current_key);
                        }
                    }
                }

                // Also try lane changes
                if let Ok(Some(left_waypoint)) = current_waypoint.left_lane() {
                    let left_key = waypoint_key(&left_waypoint);
                    if let std::collections::hash_map::Entry::Vacant(e) = visited.entry(left_key) {
                        queue.push_back(left_key);
                        e.insert(left_waypoint.clone());
                        parent.insert(left_key, current_key);
                    }
                }

                if let Ok(Some(right_waypoint)) = current_waypoint.right_lane() {
                    let right_key = waypoint_key(&right_waypoint);
                    if let std::collections::hash_map::Entry::Vacant(e) = visited.entry(right_key) {
                        queue.push_back(right_key);
                        e.insert(right_waypoint.clone());
                        parent.insert(right_key, current_key);
                    }
                }
            }

            // Limit search to prevent infinite loops
            if visited.len() > 1000 {
                break;
            }
        }

        Ok(None)
    }

    /// Get nearby waypoints within a radius.
    pub fn nearby_waypoints(&self, location: Location, radius: f64) -> CarlaResult<Vec<Waypoint>> {
        // Generate waypoints in the area
        let waypoints = self.map.generate_waypoints(2.0); // 2 meter spacing

        let mut nearby = Vec::new();
        for waypoint in waypoints.iter() {
            let distance = location.distance(&waypoint.transform.location);
            if distance <= radius {
                nearby.push(waypoint.clone());
            }
        }

        Ok(nearby)
    }

    /// Get the closest waypoint to a location.
    pub fn closest_waypoint(&self, location: Location) -> Option<Waypoint> {
        self.map.get_waypoint(location, true, None)
    }

    /// Check if lane change is allowed from the current waypoint.
    pub fn can_change_lane(&self, waypoint: &Waypoint, direction: LaneChangeDirection) -> bool {
        use super::LaneChange;

        match direction {
            LaneChangeDirection::Left => {
                matches!(waypoint.lane_change, LaneChange::Left | LaneChange::Both)
            }
            LaneChangeDirection::Right => {
                matches!(waypoint.lane_change, LaneChange::Right | LaneChange::Both)
            }
        }
    }
}

/// Direction for lane changes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneChangeDirection {
    /// Change to left lane
    Left,
    /// Change to right lane
    Right,
}

/// Create a unique key for a waypoint for pathfinding.
fn waypoint_key(waypoint: &Waypoint) -> u64 {
    // Create a hash-like key from waypoint coordinates and IDs
    let x = (waypoint.transform.location.x * 10.0) as i64;
    let y = (waypoint.transform.location.y * 10.0) as i64;
    let road_id = waypoint.road_id as u64;
    let lane_id = waypoint.lane_id as u64;

    // Combine into a single key
    ((x as u64) << 32) ^ ((y as u64) << 16) ^ (road_id << 8) ^ lane_id
}

#[cfg(test)]
mod tests {
    use super::*;
    // Location is used in route creation tests

    #[test]
    fn test_route_creation() {
        // This test would require actual waypoints, so it's a placeholder
        let route = Route::new(Vec::new());
        assert_eq!(route.len(), 0);
        assert!(route.is_empty());
        assert_eq!(route.distance, 0.0);
    }

    #[test]
    fn test_waypoint_key() {
        // Create a mock waypoint for testing the key function
        // Note: This is a simplified test since creating real waypoints requires CARLA
        // In practice, the key function should create unique keys for different waypoints
    }
}
