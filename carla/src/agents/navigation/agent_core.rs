//! Shared agent core functionality for hazard detection and planning.

use super::{
    global_route_planner::GlobalRoutePlanner,
    local_planner::{LocalPlanner, LocalPlannerConfig},
    types::RoadOption,
};
use crate::{
    client::{ActorBase, ActorList, Map, TrafficLight, Vehicle, Waypoint, World},
    geom::{Location, Vector3D},
};
use anyhow::Result;

/// Result of obstacle detection.
#[derive(Debug, Clone)]
pub struct ObstacleDetectionResult {
    pub obstacle_was_found: bool,
    pub obstacle_id: Option<u32>,
    pub distance: f32,
}

/// Result of traffic light detection.
#[derive(Debug, Clone)]
pub struct TrafficLightDetectionResult {
    pub traffic_light_was_found: bool,
    pub traffic_light: Option<TrafficLight>,
}

/// Lane change direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneChangeDirection {
    /// Change to left lane
    Left,
    /// Change to right lane
    Right,
}

/// Configuration for AgentCore.
#[derive(Debug, Clone)]
pub struct AgentCoreConfig {
    /// Ignore traffic lights
    pub ignore_traffic_lights: bool,
    /// Ignore stop signs
    pub ignore_stop_signs: bool,
    /// Ignore vehicles
    pub ignore_vehicles: bool,
    /// Base threshold for traffic light detection
    pub base_tlight_threshold: f32,
    /// Base threshold for vehicle detection
    pub base_vehicle_threshold: f32,
    /// Speed ratio for dynamic thresholds
    pub speed_ratio: f32,
    /// Maximum brake force
    pub max_brake: f32,
    /// Lateral offset
    pub offset: f32,
    /// Use bounding box detection
    pub use_bbs_detection: bool,
    /// Sampling resolution for route planner
    pub sampling_resolution: f32,
}

impl Default for AgentCoreConfig {
    fn default() -> Self {
        Self {
            ignore_traffic_lights: false,
            ignore_stop_signs: false,
            ignore_vehicles: false,
            base_tlight_threshold: 5.0,
            base_vehicle_threshold: 5.0,
            speed_ratio: 1.0,
            max_brake: 0.5,
            offset: 0.0,
            use_bbs_detection: false,
            sampling_resolution: 2.0,
        }
    }
}

/// Shared core state and detection logic for all agent types.
pub struct AgentCore {
    pub vehicle: Vehicle,
    pub world: World,
    pub map: Map,
    pub local_planner: LocalPlanner,
    pub global_planner: GlobalRoutePlanner,

    // Configuration
    pub ignore_traffic_lights: bool,
    pub ignore_stop_signs: bool,
    pub ignore_vehicles: bool,
    pub base_tlight_threshold: f32,
    pub base_vehicle_threshold: f32,
    pub speed_ratio: f32,
    pub max_brake: f32,

    // Cached state
    lights_list: Option<ActorList>,
    last_traffic_light: Option<TrafficLight>,
}

impl AgentCore {
    /// Creates a new AgentCore.
    pub fn new(
        vehicle: Vehicle,
        map: Option<Map>,
        grp_inst: Option<GlobalRoutePlanner>,
        config: AgentCoreConfig,
        local_config: LocalPlannerConfig,
    ) -> Result<Self> {
        let world = vehicle.world();

        let map = if let Some(m) = map { m } else { world.map() };

        let global_planner = if let Some(grp) = grp_inst {
            grp
        } else {
            GlobalRoutePlanner::new(map.clone(), config.sampling_resolution)
        };

        let local_planner = LocalPlanner::new(vehicle.clone(), local_config);

        Ok(Self {
            vehicle,
            world,
            map,
            local_planner,
            global_planner,
            ignore_traffic_lights: config.ignore_traffic_lights,
            ignore_stop_signs: config.ignore_stop_signs,
            ignore_vehicles: config.ignore_vehicles,
            base_tlight_threshold: config.base_tlight_threshold,
            base_vehicle_threshold: config.base_vehicle_threshold,
            speed_ratio: config.speed_ratio,
            max_brake: config.max_brake,
            lights_list: None,
            last_traffic_light: None,
        })
    }

    /// Sets whether to ignore traffic lights.
    pub fn ignore_traffic_lights(&mut self, active: bool) {
        self.ignore_traffic_lights = active;
    }

    /// Sets whether to ignore stop signs.
    pub fn ignore_stop_signs(&mut self, active: bool) {
        self.ignore_stop_signs = active;
    }

    /// Sets whether to ignore vehicles.
    pub fn ignore_vehicles(&mut self, active: bool) {
        self.ignore_vehicles = active;
    }

    /// Checks if a traffic light is affecting the vehicle.
    ///
    /// Uses distance-based detection as a fallback. For more accurate detection,
    /// use `affected_by_traffic_light_with_trigger_volumes()`.
    pub fn affected_by_traffic_light(
        &mut self,
        max_distance: f32,
    ) -> Result<TrafficLightDetectionResult> {
        if self.ignore_traffic_lights {
            return Ok(TrafficLightDetectionResult {
                traffic_light_was_found: false,
                traffic_light: None,
            });
        }

        // Get traffic lights if not cached
        if self.lights_list.is_none() {
            self.lights_list = Some(self.world.actors().filter("*traffic_light*"));
        }

        // Simplified implementation: check if any traffic light is red and nearby
        let vehicle_location = self.vehicle.transform().location;

        if let Some(ref lights) = self.lights_list {
            for actor in lights.iter() {
                // Try to cast to TrafficLight
                if let Ok(traffic_light) = TrafficLight::try_from(actor.clone()) {
                    // Check if red
                    use crate::rpc::TrafficLightState;
                    if traffic_light.state() != TrafficLightState::Red {
                        continue;
                    }

                    // Check distance
                    let light_location = traffic_light.transform().location;
                    let distance = ((vehicle_location.x - light_location.x).powi(2)
                        + (vehicle_location.y - light_location.y).powi(2))
                    .sqrt();

                    if distance < max_distance {
                        self.last_traffic_light = Some(traffic_light.clone());
                        return Ok(TrafficLightDetectionResult {
                            traffic_light_was_found: true,
                            traffic_light: Some(traffic_light),
                        });
                    }
                }
            }
        }

        Ok(TrafficLightDetectionResult {
            traffic_light_was_found: false,
            traffic_light: None,
        })
    }

    /// Checks if a traffic light is affecting the vehicle using trigger volume waypoints.
    ///
    /// This is more accurate than the simple distance check, as it only detects
    /// traffic lights that actually affect the vehicle's current lane.
    pub fn affected_by_traffic_light_with_trigger_volumes(
        &mut self,
        max_distance: f32,
    ) -> Result<TrafficLightDetectionResult> {
        if self.ignore_traffic_lights {
            return Ok(TrafficLightDetectionResult {
                traffic_light_was_found: false,
                traffic_light: None,
            });
        }

        // Get traffic lights if not cached
        if self.lights_list.is_none() {
            self.lights_list = Some(self.world.actors().filter("*traffic_light*"));
        }

        // Get vehicle waypoint
        let vehicle_location = self.vehicle.transform().location;
        let vehicle_waypoint = self
            .map
            .waypoint_at(&vehicle_location)
            .ok_or_else(|| anyhow::anyhow!("Failed to get waypoint for vehicle location"))?;

        if let Some(ref lights) = self.lights_list {
            for actor in lights.iter() {
                // Try to cast to TrafficLight
                if let Ok(traffic_light) = TrafficLight::try_from(actor.clone()) {
                    // Check if red or yellow
                    use crate::rpc::TrafficLightState;
                    let state = traffic_light.state();
                    if state != TrafficLightState::Red && state != TrafficLightState::Yellow {
                        continue;
                    }

                    // Get affected lane waypoints (trigger volume)
                    let affected_waypoints = traffic_light.affected_lane_waypoints();

                    // Check if vehicle's waypoint is in the affected waypoints
                    for trigger_wp in affected_waypoints.iter() {
                        // Check if trigger waypoint is close to vehicle waypoint
                        let trigger_loc = trigger_wp.transform().location;
                        let vehicle_loc = vehicle_location;

                        let distance = ((trigger_loc.x - vehicle_loc.x).powi(2)
                            + (trigger_loc.y - vehicle_loc.y).powi(2))
                        .sqrt();

                        // Also check if same road_id and lane_id for more accuracy
                        let same_road = trigger_wp.road_id() == vehicle_waypoint.road_id();
                        let same_lane = trigger_wp.lane_id() == vehicle_waypoint.lane_id();

                        if distance < max_distance && same_road && same_lane {
                            self.last_traffic_light = Some(traffic_light.clone());
                            return Ok(TrafficLightDetectionResult {
                                traffic_light_was_found: true,
                                traffic_light: Some(traffic_light),
                            });
                        }
                    }
                }
            }
        }

        Ok(TrafficLightDetectionResult {
            traffic_light_was_found: false,
            traffic_light: None,
        })
    }

    /// Checks if a vehicle obstacle is in front.
    ///
    /// Uses distance and dot product check. For more accurate detection,
    /// use `vehicle_obstacle_detected_with_bounding_boxes()`.
    pub fn vehicle_obstacle_detected(&self, max_distance: f32) -> Result<ObstacleDetectionResult> {
        if self.ignore_vehicles {
            return Ok(ObstacleDetectionResult {
                obstacle_was_found: false,
                obstacle_id: None,
                distance: -1.0,
            });
        }

        let vehicle_list = self.world.actors().filter("*vehicle*");

        let vehicle_transform = self.vehicle.transform();
        let vehicle_location = vehicle_transform.location;
        let vehicle_forward = vehicle_transform.rotation.forward_vector();

        for actor in vehicle_list.iter() {
            // Skip self
            if actor.id() == self.vehicle.id() {
                continue;
            }

            let other_location = actor.transform().location;

            // Check distance
            let dx = other_location.x - vehicle_location.x;
            let dy = other_location.y - vehicle_location.y;
            let distance = (dx * dx + dy * dy).sqrt();

            if distance > max_distance {
                continue;
            }

            // Check if in front (dot product > 0)
            let dot = dx * vehicle_forward.x + dy * vehicle_forward.y;
            if dot > 0.0 {
                return Ok(ObstacleDetectionResult {
                    obstacle_was_found: true,
                    obstacle_id: Some(actor.id()),
                    distance,
                });
            }
        }

        Ok(ObstacleDetectionResult {
            obstacle_was_found: false,
            obstacle_id: None,
            distance: -1.0,
        })
    }

    /// Checks if a vehicle obstacle is in the planned route using bounding box intersection.
    ///
    /// This is more accurate than simple distance checks, as it checks if the vehicle's
    /// bounding box actually intersects with the planned route polygon.
    ///
    /// # Arguments
    /// * `max_distance` - Maximum look-ahead distance for checking obstacles
    /// * `route_waypoints` - Planned route waypoints to check against
    ///
    /// # Returns
    /// ObstacleDetectionResult with obstacle information
    pub fn vehicle_obstacle_detected_with_bounding_boxes(
        &self,
        max_distance: f32,
        route_waypoints: &[(Waypoint, RoadOption)],
    ) -> Result<ObstacleDetectionResult> {
        use geo::{Coord, LineString, Polygon as GeoPolygon};

        if self.ignore_vehicles {
            return Ok(ObstacleDetectionResult {
                obstacle_was_found: false,
                obstacle_id: None,
                distance: -1.0,
            });
        }

        if route_waypoints.is_empty() {
            return Ok(ObstacleDetectionResult {
                obstacle_was_found: false,
                obstacle_id: None,
                distance: -1.0,
            });
        }

        // Build route polygon from waypoints
        let mut route_coords: Vec<Coord<f64>> = Vec::new();
        const ROUTE_WIDTH: f32 = 2.0; // Half width of route corridor in meters

        for (waypoint, _) in route_waypoints.iter().take(20) {
            // Look ahead at most 20 waypoints
            let wp_transform = waypoint.transform();
            let wp_loc = wp_transform.location;

            // Get forward and right vectors
            let forward = wp_transform.rotation.forward_vector();
            let right = crate::geom::Vector3D {
                x: -forward.y,
                y: forward.x,
                z: 0.0,
            };

            // Add left and right edge points
            let left_point = Coord {
                x: (wp_loc.x - right.x * ROUTE_WIDTH) as f64,
                y: (wp_loc.y - right.y * ROUTE_WIDTH) as f64,
            };
            route_coords.push(left_point);
        }

        // Add right side in reverse to close the polygon
        for (waypoint, _) in route_waypoints.iter().take(20).rev() {
            let wp_transform = waypoint.transform();
            let wp_loc = wp_transform.location;

            let forward = wp_transform.rotation.forward_vector();
            let right = crate::geom::Vector3D {
                x: -forward.y,
                y: forward.x,
                z: 0.0,
            };

            let right_point = Coord {
                x: (wp_loc.x + right.x * ROUTE_WIDTH) as f64,
                y: (wp_loc.y + right.y * ROUTE_WIDTH) as f64,
            };
            route_coords.push(right_point);
        }

        // Close the polygon
        if !route_coords.is_empty() {
            let first = route_coords[0];
            route_coords.push(first);
        }

        let route_polygon = GeoPolygon::new(LineString::from(route_coords), vec![]);

        // Check each vehicle for intersection
        let vehicle_list = self.world.actors().filter("*vehicle*");
        let vehicle_location = self.vehicle.transform().location;

        for actor in vehicle_list.iter() {
            // Skip self
            if actor.id() == self.vehicle.id() {
                continue;
            }

            // Distance check first for efficiency
            let other_transform = actor.transform();
            let other_location = other_transform.location;

            let dx = other_location.x - vehicle_location.x;
            let dy = other_location.y - vehicle_location.y;
            let distance = (dx * dx + dy * dy).sqrt();

            if distance > max_distance {
                continue;
            }

            // Try to cast to Vehicle to get bounding box
            if let Ok(other_vehicle) = crate::client::Vehicle::try_from(actor.clone()) {
                let bbox = other_vehicle.bounding_box();

                // Build bounding box polygon in world coordinates
                let bbox_transform = other_transform;

                // Get bounding box corners (in local coordinates)
                let extent = &bbox.extent;
                let corners = vec![
                    crate::geom::Location::new(extent.x, extent.y, 0.0),
                    crate::geom::Location::new(-extent.x, extent.y, 0.0),
                    crate::geom::Location::new(-extent.x, -extent.y, 0.0),
                    crate::geom::Location::new(extent.x, -extent.y, 0.0),
                ];

                // Transform corners to world coordinates
                let mut world_coords: Vec<Coord<f64>> = Vec::new();
                for corner in corners {
                    // Apply rotation then translation: world = rotation * local + translation
                    let rotated = bbox_transform.rotation.rotate_vector(&Vector3D {
                        x: corner.x,
                        y: corner.y,
                        z: corner.z,
                    });
                    let world_corner = Location {
                        x: rotated.x + bbox_transform.location.x,
                        y: rotated.y + bbox_transform.location.y,
                        z: rotated.z + bbox_transform.location.z,
                    };
                    world_coords.push(Coord {
                        x: world_corner.x as f64,
                        y: world_corner.y as f64,
                    });
                }

                // Close the polygon
                world_coords.push(world_coords[0]);

                let bbox_polygon = GeoPolygon::new(LineString::from(world_coords), vec![]);

                // Check intersection
                use geo::algorithm::intersects::Intersects;
                if route_polygon.intersects(&bbox_polygon) {
                    return Ok(ObstacleDetectionResult {
                        obstacle_was_found: true,
                        obstacle_id: Some(actor.id()),
                        distance,
                    });
                }
            }
        }

        Ok(ObstacleDetectionResult {
            obstacle_was_found: false,
            obstacle_id: None,
            distance: -1.0,
        })
    }

    /// Generates a lane change path.
    ///
    /// # Arguments
    /// * `waypoint` - Starting waypoint
    /// * `direction` - Lane change direction (left or right)
    /// * `distance_same_lane` - Distance to travel in same lane before starting change (meters)
    /// * `distance_other_lane` - Distance to travel in new lane after completing change (meters)
    /// * `lane_change_distance` - Distance over which to perform the lane change (meters)
    /// * `check` - Whether to check if lane change is possible
    /// * `step_distance` - Distance between waypoints in the path (meters)
    ///
    /// # Returns
    /// Vector of waypoints with road options, or empty if lane change not possible
    pub fn generate_lane_change_path(
        waypoint: &Waypoint,
        direction: LaneChangeDirection,
        distance_same_lane: f32,
        distance_other_lane: f32,
        lane_change_distance: f32,
        check: bool,
        step_distance: f32,
    ) -> Vec<(Waypoint, RoadOption)> {
        let mut path = Vec::new();

        // Phase 1: Stay in same lane for distance_same_lane
        let mut current_wp = waypoint.clone();
        let same_lane_steps = (distance_same_lane / step_distance).ceil() as usize;

        for _ in 0..same_lane_steps {
            path.push((current_wp.clone(), RoadOption::LaneFollow));
            let next_wps = current_wp.next(step_distance as f64);
            if next_wps.is_empty() {
                return Vec::new(); // Can't continue
            }
            current_wp = next_wps.get(0).unwrap().clone();
        }

        // Check if lane change is possible
        let target_lane_wp = match direction {
            LaneChangeDirection::Left => current_wp.left(),
            LaneChangeDirection::Right => current_wp.right(),
        };

        if check {
            // Check if target lane exists by trying to get next waypoint
            let target_next = target_lane_wp.next(1.0);
            if target_next.is_empty() {
                return Vec::new(); // Lane change not possible
            }
        }

        // Phase 2: Lane change transition
        let lane_change_steps = (lane_change_distance / step_distance).ceil() as usize;
        let road_option = match direction {
            LaneChangeDirection::Left => RoadOption::ChangeLaneLeft,
            LaneChangeDirection::Right => RoadOption::ChangeLaneRight,
        };

        // During lane change, gradually transition to the target lane
        for i in 0..lane_change_steps {
            path.push((current_wp.clone(), road_option));

            // Try to move towards target lane
            let next_wps = current_wp.next(step_distance as f64);
            if next_wps.is_empty() {
                return path; // End of road
            }
            current_wp = next_wps.get(0).unwrap().clone();

            // Gradually move to target lane during the transition
            if i == lane_change_steps / 2 {
                // At midpoint, try to switch to target lane
                let target_next = target_lane_wp.next((step_distance * (i as f32)) as f64);
                if !target_next.is_empty() {
                    current_wp = target_next.get(0).unwrap().clone();
                }
            }
        }

        // Phase 3: Stay in new lane for distance_other_lane
        let other_lane_steps = (distance_other_lane / step_distance).ceil() as usize;
        for _ in 0..other_lane_steps {
            path.push((current_wp.clone(), RoadOption::LaneFollow));
            let next_wps = current_wp.next(step_distance as f64);
            if next_wps.is_empty() {
                break; // End of road
            }
            current_wp = next_wps.get(0).unwrap().clone();
        }

        path
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_agent_core_config_default() {
        let config = AgentCoreConfig::default();
        assert!(!config.ignore_traffic_lights);
        assert_eq!(config.base_tlight_threshold, 5.0);
        assert_eq!(config.sampling_resolution, 2.0);
    }

    #[test]
    fn test_obstacle_detection_result() {
        let result = ObstacleDetectionResult {
            obstacle_was_found: true,
            obstacle_id: Some(42),
            distance: 10.5,
        };
        assert!(result.obstacle_was_found);
        assert_eq!(result.obstacle_id, Some(42));
    }

    #[test]
    fn test_traffic_light_detection_result() {
        let result = TrafficLightDetectionResult {
            traffic_light_was_found: false,
            traffic_light: None,
        };
        assert!(!result.traffic_light_was_found);
        assert!(result.traffic_light.is_none());
    }
}
