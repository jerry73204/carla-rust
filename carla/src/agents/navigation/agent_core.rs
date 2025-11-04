//! Shared agent core functionality for hazard detection and planning.

use super::{
    global_route_planner::GlobalRoutePlanner,
    local_planner::{LocalPlanner, LocalPlannerConfig},
};
use crate::client::{ActorBase, ActorList, Map, TrafficLight, Vehicle, World};
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
    ignore_traffic_lights: bool,
    ignore_stop_signs: bool,
    ignore_vehicles: bool,
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
        let vehicle_isometry = self.vehicle.transform();
        let vehicle_location =
            crate::geom::Location::from_na_translation(&vehicle_isometry.translation);

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
                    let light_isometry = traffic_light.transform();
                    let light_location =
                        crate::geom::Location::from_na_translation(&light_isometry.translation);
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

    /// Checks if a vehicle obstacle is in front.
    pub fn vehicle_obstacle_detected(&self, max_distance: f32) -> Result<ObstacleDetectionResult> {
        if self.ignore_vehicles {
            return Ok(ObstacleDetectionResult {
                obstacle_was_found: false,
                obstacle_id: None,
                distance: -1.0,
            });
        }

        let vehicle_list = self.world.actors().filter("*vehicle*");

        let vehicle_isometry = self.vehicle.transform();
        let vehicle_location =
            crate::geom::Location::from_na_translation(&vehicle_isometry.translation);

        // Convert isometry to our Transform type to get rotation
        let vehicle_transform = crate::geom::Transform::from_na(&vehicle_isometry);
        let vehicle_forward = vehicle_transform.rotation.forward_vector();

        for actor in vehicle_list.iter() {
            // Skip self
            if actor.id() == self.vehicle.id() {
                continue;
            }

            let other_isometry = actor.transform();
            let other_location =
                crate::geom::Location::from_na_translation(&other_isometry.translation);

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
