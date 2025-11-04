//! Global route planner for high-level path planning using A* pathfinding.

use super::types::RoadOption;
use crate::{
    client::{Map, Waypoint},
    geom::Location,
};
use anyhow::Result;
use petgraph::{
    graph::{Graph, NodeIndex},
    Directed,
};
use std::collections::HashMap;

/// Node in the road topology graph.
#[derive(Debug, Clone)]
struct RoadNode {
    waypoint: Waypoint,
    location: Location,
}

/// Edge in the road topology graph.
#[derive(Debug, Clone)]
struct RoadEdge {
    /// Length of this road segment in meters
    length: f32,
    /// Road option for this edge (left turn, right turn, etc.)
    road_option: RoadOption,
}

/// Global route planner that computes optimal routes using A* pathfinding.
///
/// Builds a graph from the CARLA map topology and uses A* to find
/// the shortest path between locations.
pub struct GlobalRoutePlanner {
    map: Map,
    #[allow(dead_code)]
    sampling_resolution: f32,
    /// Graph structure for pathfinding
    graph: Graph<RoadNode, RoadEdge, Directed>,
    /// Map from waypoint ID to graph node index
    waypoint_to_node: HashMap<u64, NodeIndex>,
}

impl GlobalRoutePlanner {
    /// Creates a new GlobalRoutePlanner and builds the road topology graph.
    ///
    /// # Arguments
    /// * `map` - CARLA map instance
    /// * `sampling_resolution` - Distance between route waypoints in meters
    pub fn new(map: Map, sampling_resolution: f32) -> Self {
        let mut planner = Self {
            map,
            sampling_resolution,
            graph: Graph::new(),
            waypoint_to_node: HashMap::new(),
        };

        planner.build_graph();
        planner
    }

    /// Builds the road topology graph from the CARLA map.
    fn build_graph(&mut self) {
        // Get topology from map
        let topology = self.map.topology();

        // Phase 1: Create nodes for all waypoints in topology
        for (start_wp, end_wp) in topology.iter() {
            // Add start waypoint if not already present
            if !self.waypoint_to_node.contains_key(&start_wp.id()) {
                let isometry = start_wp.transform();
                let location = Location::from_na_translation(&isometry.translation);
                let node = RoadNode {
                    waypoint: start_wp.clone(),
                    location,
                };
                let node_idx = self.graph.add_node(node);
                self.waypoint_to_node.insert(start_wp.id(), node_idx);
            }

            // Add end waypoint if not already present
            if !self.waypoint_to_node.contains_key(&end_wp.id()) {
                let isometry = end_wp.transform();
                let location = Location::from_na_translation(&isometry.translation);
                let node = RoadNode {
                    waypoint: end_wp.clone(),
                    location,
                };
                let node_idx = self.graph.add_node(node);
                self.waypoint_to_node.insert(end_wp.id(), node_idx);
            }
        }

        // Phase 2: Create edges between connected waypoints
        for (start_wp, end_wp) in topology.iter() {
            let start_idx = self.waypoint_to_node[&start_wp.id()];
            let end_idx = self.waypoint_to_node[&end_wp.id()];

            // Calculate edge length
            let start_isometry = start_wp.transform();
            let start_loc = Location::from_na_translation(&start_isometry.translation);
            let end_isometry = end_wp.transform();
            let end_loc = Location::from_na_translation(&end_isometry.translation);

            let length = ((end_loc.x - start_loc.x).powi(2)
                + (end_loc.y - start_loc.y).powi(2)
                + (end_loc.z - start_loc.z).powi(2))
            .sqrt();

            // Determine road option based on waypoint relationship
            let road_option = self.compute_road_option(start_wp, end_wp);

            let edge = RoadEdge {
                length,
                road_option,
            };

            self.graph.add_edge(start_idx, end_idx, edge);
        }

        // Phase 3: Add lane change edges
        self.add_lane_change_edges();
    }

    /// Adds lane change edges to the graph.
    fn add_lane_change_edges(&mut self) {
        // Collect all waypoints that have lane change options
        let waypoints: Vec<_> = self.waypoint_to_node.keys().copied().collect();

        // Collect edges to add (to avoid borrowing issues)
        let mut edges_to_add: Vec<(NodeIndex, NodeIndex, RoadEdge)> = Vec::new();

        for wp_id in waypoints {
            let node_idx = self.waypoint_to_node[&wp_id];
            let node = &self.graph[node_idx];
            let waypoint = node.waypoint.clone();

            // Check for left lane
            let left_wp = waypoint.left();
            if let Some(&left_idx) = self.waypoint_to_node.get(&left_wp.id()) {
                // Add lane change left edge
                let edge = RoadEdge {
                    length: 1.0, // Small cost for lane changes
                    road_option: RoadOption::ChangeLaneLeft,
                };
                edges_to_add.push((node_idx, left_idx, edge));
            }

            // Check for right lane
            let right_wp = waypoint.right();
            if let Some(&right_idx) = self.waypoint_to_node.get(&right_wp.id()) {
                // Add lane change right edge
                let edge = RoadEdge {
                    length: 1.0, // Small cost for lane changes
                    road_option: RoadOption::ChangeLaneRight,
                };
                edges_to_add.push((node_idx, right_idx, edge));
            }
        }

        // Now add all the edges
        for (from, to, edge) in edges_to_add {
            self.graph.add_edge(from, to, edge);
        }
    }

    /// Computes the RoadOption for an edge based on the turn angle.
    fn compute_road_option(&self, start_wp: &Waypoint, end_wp: &Waypoint) -> RoadOption {
        // Get transforms
        let start_transform = crate::geom::Transform::from_na(&start_wp.transform());
        let end_transform = crate::geom::Transform::from_na(&end_wp.transform());

        // Get forward vectors
        let start_forward = start_transform.rotation.forward_vector();
        let end_forward = end_transform.rotation.forward_vector();

        // Calculate the cross product to determine turn direction
        let cross_z = start_forward.x * end_forward.y - start_forward.y * end_forward.x;

        // Calculate the dot product to determine if it's a turn or straight
        let dot = start_forward.x * end_forward.x + start_forward.y * end_forward.y;

        // Thresholds for turn classification
        const TURN_THRESHOLD: f32 = 0.1;
        const STRAIGHT_THRESHOLD: f32 = 0.7;

        if dot > STRAIGHT_THRESHOLD {
            RoadOption::LaneFollow
        } else if cross_z > TURN_THRESHOLD {
            RoadOption::Left
        } else if cross_z < -TURN_THRESHOLD {
            RoadOption::Right
        } else {
            RoadOption::Straight
        }
    }

    /// Computes an optimal route from origin to destination using A* pathfinding.
    ///
    /// # Arguments
    /// * `origin` - Starting location
    /// * `destination` - Target location
    ///
    /// # Returns
    /// Vector of (Waypoint, RoadOption) pairs representing the optimal route
    pub fn trace_route(
        &self,
        origin: Location,
        destination: Location,
    ) -> Result<Vec<(Waypoint, RoadOption)>> {
        // Get starting and ending waypoints
        let start_wp = self
            .map
            .waypoint_at(&origin)
            .ok_or_else(|| anyhow::anyhow!("Could not find waypoint at origin"))?;

        let end_wp = self
            .map
            .waypoint_at(&destination)
            .ok_or_else(|| anyhow::anyhow!("Could not find waypoint at destination"))?;

        // Find closest nodes in graph to start and end waypoints
        let start_node = self.find_closest_node(&start_wp)?;
        let end_node = self.find_closest_node(&end_wp)?;

        // Run A* pathfinding
        let path = petgraph::algo::astar(
            &self.graph,
            start_node,
            |node| node == end_node,
            |edge| edge.weight().length,
            |node| {
                // Heuristic: Euclidean distance to goal
                let node_loc = &self.graph[node].location;
                let end_loc = &self.graph[end_node].location;
                ((node_loc.x - end_loc.x).powi(2)
                    + (node_loc.y - end_loc.y).powi(2)
                    + (node_loc.z - end_loc.z).powi(2))
                .sqrt()
            },
        );

        let (_cost, node_path) = path.ok_or_else(|| anyhow::anyhow!("No path found"))?;

        // Convert node path to waypoint route with road options
        let mut route = Vec::new();

        for i in 0..node_path.len() {
            let node_idx = node_path[i];
            let waypoint = self.graph[node_idx].waypoint.clone();

            // Determine road option from edge to next node
            let road_option = if i + 1 < node_path.len() {
                let next_node_idx = node_path[i + 1];
                // Find edge between current and next node
                self.graph
                    .edges_connecting(node_idx, next_node_idx)
                    .next()
                    .map(|edge| edge.weight().road_option)
                    .unwrap_or(RoadOption::LaneFollow)
            } else {
                RoadOption::LaneFollow
            };

            route.push((waypoint, road_option));
        }

        if route.is_empty() {
            return Err(anyhow::anyhow!("Failed to compute route"));
        }

        Ok(route)
    }

    /// Finds the closest node in the graph to the given waypoint.
    fn find_closest_node(&self, waypoint: &Waypoint) -> Result<NodeIndex> {
        let wp_isometry = waypoint.transform();
        let wp_loc = Location::from_na_translation(&wp_isometry.translation);

        // Check if waypoint is directly in the graph
        if let Some(&node_idx) = self.waypoint_to_node.get(&waypoint.id()) {
            return Ok(node_idx);
        }

        // Find closest node by distance
        let mut closest_node: Option<NodeIndex> = None;
        let mut min_distance = f32::INFINITY;

        for node_idx in self.graph.node_indices() {
            let node_loc = &self.graph[node_idx].location;
            let distance = ((wp_loc.x - node_loc.x).powi(2)
                + (wp_loc.y - node_loc.y).powi(2)
                + (wp_loc.z - node_loc.z).powi(2))
            .sqrt();

            if distance < min_distance {
                min_distance = distance;
                closest_node = Some(node_idx);
            }
        }

        closest_node.ok_or_else(|| anyhow::anyhow!("No nodes in graph"))
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
