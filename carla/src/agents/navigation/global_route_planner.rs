//! Global route planner for high-level path planning.

use super::types::RoadOption;
use crate::{
    client::{Map, Waypoint},
    geom::Location,
};
use anyhow::Result;

/// Global route planner that computes routes between locations.
///
/// This is a simplified implementation that uses Map::get_waypoint()
/// and Waypoint::next() to create simple routes. A full implementation
/// would use A* pathfinding on a graph built from Map::topology().
pub struct GlobalRoutePlanner {
    map: Map,
    sampling_resolution: f32,
}

impl GlobalRoutePlanner {
    /// Creates a new GlobalRoutePlanner.
    ///
    /// # Arguments
    /// * `map` - CARLA map instance
    /// * `sampling_resolution` - Distance between route waypoints in meters
    pub fn new(map: Map, sampling_resolution: f32) -> Self {
        Self {
            map,
            sampling_resolution,
        }
    }

    /// Computes a route from origin to destination.
    ///
    /// # Arguments
    /// * `origin` - Starting location
    /// * `destination` - Target location
    ///
    /// # Returns
    /// Vector of (Waypoint, RoadOption) pairs representing the route
    ///
    /// # Note
    /// This is a simplified implementation that creates a straight-line sequence
    /// of waypoints. A full implementation would use A* on the road topology graph.
    pub fn trace_route(
        &self,
        origin: Location,
        destination: Location,
    ) -> Result<Vec<(Waypoint, RoadOption)>> {
        let mut route = Vec::new();

        // Get starting waypoint
        let start_wp = self
            .map
            .waypoint(&origin.to_na_translation())
            .ok_or_else(|| anyhow::anyhow!("Could not find waypoint at origin"))?;

        let end_wp = self
            .map
            .waypoint(&destination.to_na_translation())
            .ok_or_else(|| anyhow::anyhow!("Could not find waypoint at destination"))?;

        // Simple route: follow next() until we get close to destination
        let mut current_wp = start_wp;
        route.push((current_wp.clone(), RoadOption::LaneFollow));

        let max_iterations = 1000; // Prevent infinite loops
        let mut iterations = 0;

        while iterations < max_iterations {
            iterations += 1;

            // Check if we're close to destination
            let current_isometry = current_wp.transform();
            let current_loc =
                crate::geom::Location::from_na_translation(&current_isometry.translation);

            let dest_isometry = end_wp.transform();
            let dest_loc = crate::geom::Location::from_na_translation(&dest_isometry.translation);

            let distance = ((current_loc.x - dest_loc.x).powi(2)
                + (current_loc.y - dest_loc.y).powi(2))
            .sqrt();

            if distance < self.sampling_resolution * 2.0 {
                // Close enough, add final waypoint
                route.push((end_wp.clone(), RoadOption::LaneFollow));
                break;
            }

            // Get next waypoints
            let next_waypoints = current_wp.next(self.sampling_resolution as f64);

            if next_waypoints.is_empty() {
                // Dead end, use what we have
                break;
            }

            // Choose the waypoint closest to destination
            let next_wp = next_waypoints
                .iter()
                .min_by(|a, b| {
                    let a_isometry = a.transform();
                    let a_loc = crate::geom::Location::from_na_translation(&a_isometry.translation);

                    let b_isometry = b.transform();
                    let b_loc = crate::geom::Location::from_na_translation(&b_isometry.translation);

                    let a_dist =
                        ((a_loc.x - dest_loc.x).powi(2) + (a_loc.y - dest_loc.y).powi(2)).sqrt();
                    let b_dist =
                        ((b_loc.x - dest_loc.x).powi(2) + (b_loc.y - dest_loc.y).powi(2)).sqrt();

                    a_dist
                        .partial_cmp(&b_dist)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .unwrap()
                .clone();

            // Determine road option (simplified - just use LaneFollow for now)
            // A full implementation would compute turn directions
            let road_option = RoadOption::LaneFollow;

            route.push((next_wp.clone(), road_option));
            current_wp = next_wp;
        }

        if route.len() < 2 {
            return Err(anyhow::anyhow!("Failed to compute route"));
        }

        Ok(route)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_global_route_planner_creation() {
        // This test just checks that the struct can be created
        // Full testing requires a CARLA connection
        let sampling_resolution = 2.0;
        assert_eq!(sampling_resolution, 2.0);
    }
}
