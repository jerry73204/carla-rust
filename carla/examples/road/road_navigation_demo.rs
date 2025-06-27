//! Road network navigation and pathfinding demonstration.
//!
//! This example shows how to use the enhanced map and waypoint functionality
//! for road network exploration, pathfinding, and landmark detection.

use carla::{
    client::Client,
    geom::Location,
    road::{LandmarkType, Navigator},
};
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("CARLA Road Navigation Demo");
    println!("=========================");
    
    // Connect to CARLA
    let client = Client::connect("localhost", 2000, Some(Duration::from_secs(10)))?;
    let world = client.world();
    
    println!("Connected to CARLA server");
    
    // Get the map
    let map = world.map()?;
    println!("Map name: {}", map.name());
    
    // Create a navigator
    let navigator = Navigator::new(map.clone());
    
    // Demo 1: Get spawn points and basic waypoint information
    println!("\n1. Spawn Points Analysis");
    println!("========================");
    
    let spawn_points = map.spawn_points();
    println!("Total spawn points: {}", spawn_points.len());
    
    if spawn_points.len() > 0 {
        let first_spawn = spawn_points.get(0).unwrap();
        println!("First spawn point: {:?}", first_spawn.location);
        
        // Get waypoint at spawn location
        if let Some(waypoint) = map.get_waypoint(first_spawn.location, true, None) {
            println!("Waypoint info:");
            println!("  Road ID: {}", waypoint.road_id);
            println!("  Lane ID: {}", waypoint.lane_id);
            println!("  Lane width: {:.2}m", waypoint.lane_width);
            println!("  Lane type: {:?}", waypoint.lane_type);
            println!("  Lane change: {:?}", waypoint.lane_change);
            println!("  In junction: {}", waypoint.is_junction());
        }
    }
    
    // Demo 2: Pathfinding between two points
    println!("\n2. Pathfinding Demo");
    println!("==================");
    
    if spawn_points.len() >= 2 {
        let start_location = spawn_points.get(0).unwrap().location;
        let end_location = spawn_points.get(1).unwrap().location;
        
        println!("Finding route from {:?} to {:?}", start_location, end_location);
        
        match navigator.find_route(start_location, end_location)? {
            Some(route) => {
                println!("Route found!");
                println!("  Waypoints: {}", route.len());
                println!("  Distance: {:.2}m", route.distance);
                
                if let (Some(start), Some(end)) = (route.start(), route.end()) {
                    println!("  Start: Road {}, Lane {}", start.road_id, start.lane_id);
                    println!("  End: Road {}, Lane {}", end.road_id, end.lane_id);
                }
            }
            None => {
                println!("No route found between these points");
            }
        }
    }
    
    // Demo 3: Landmark detection
    println!("\n3. Landmark Analysis");
    println!("===================");
    
    let all_landmarks = map.landmarks();
    println!("Total landmarks in map: {}", all_landmarks.len());
    
    // Count landmarks by type
    let mut traffic_lights = 0;
    let mut stop_signs = 0;
    let mut speed_limits = 0;
    let mut other_landmarks = 0;
    
    for landmark in &all_landmarks {
        let landmark_type = LandmarkType::from_str(&landmark.landmark_type);
        match landmark_type {
            LandmarkType::TrafficLight => traffic_lights += 1,
            LandmarkType::StopSign => stop_signs += 1,
            LandmarkType::SpeedLimit => speed_limits += 1,
            _ => other_landmarks += 1,
        }
    }
    
    println!("Landmark breakdown:");
    println!("  Traffic lights: {}", traffic_lights);
    println!("  Stop signs: {}", stop_signs);
    println!("  Speed limits: {}", speed_limits);
    println!("  Other landmarks: {}", other_landmarks);
    
    // Show some speed limit information
    if speed_limits > 0 {
        println!("\nSpeed limit signs found:");
        let mut speed_limit_count = 0;
        for landmark in &all_landmarks {
            if landmark.is_speed_limit() {
                if let Some(limit) = landmark.speed_limit() {
                    println!("  {} {}: {:.0} {}", landmark.name, landmark.id, limit, landmark.unit);
                    speed_limit_count += 1;
                    if speed_limit_count >= 5 {
                        break; // Show only first 5
                    }
                }
            }
        }
    }
    
    // Demo 4: Waypoint landmark detection
    println!("\n4. Waypoint Landmark Detection");
    println!("=============================");
    
    if spawn_points.len() > 0 {
        let test_location = spawn_points.get(0).unwrap().location;
        if let Some(waypoint) = map.get_waypoint(test_location, true, None) {
            println!("Testing landmarks near spawn point...");
            
            // Get landmarks within 50 meters
            match waypoint.landmarks(50.0) {
                Ok(nearby_landmarks) => {
                    println!("Found {} landmarks within 50m", nearby_landmarks.len());
                    
                    for (i, landmark) in nearby_landmarks.iter().enumerate() {
                        if i >= 3 { break; } // Show only first 3
                        println!("  {}: {} ({})", i + 1, landmark.name, landmark.landmark_type);
                        
                        let distance = waypoint.transform.location.distance(&landmark.transform.location);
                        println!("    Distance: {:.1}m", distance);
                    }
                    
                    // Look specifically for traffic lights
                    match waypoint.landmarks_of_type("traffic_light", 100.0) {
                        Ok(traffic_lights) => {
                            if !traffic_lights.is_empty() {
                                println!("Found {} traffic lights within 100m", traffic_lights.len());
                            }
                        }
                        Err(e) => println!("Error getting traffic lights: {}", e),
                    }
                }
                Err(e) => println!("Error getting landmarks: {}", e),
            }
        }
    }
    
    // Demo 5: Nearby waypoint search
    println!("\n5. Nearby Waypoint Search");
    println!("========================");
    
    if spawn_points.len() > 0 {
        let search_location = spawn_points.get(0).unwrap().location;
        println!("Searching for waypoints near {:?}", search_location);
        
        match navigator.nearby_waypoints(search_location, 25.0) {
            Ok(nearby_waypoints) => {
                println!("Found {} waypoints within 25m", nearby_waypoints.len());
                
                // Show some statistics
                let mut junction_count = 0;
                let mut lane_types = std::collections::HashMap::new();
                
                for waypoint in &nearby_waypoints {
                    if waypoint.is_junction() {
                        junction_count += 1;
                    }
                    
                    *lane_types.entry(format!("{:?}", waypoint.lane_type)).or_insert(0) += 1;
                }
                
                println!("  Junction waypoints: {}", junction_count);
                println!("  Lane types:");
                for (lane_type, count) in lane_types {
                    println!("    {}: {}", lane_type, count);
                }
            }
            Err(e) => println!("Error searching nearby waypoints: {}", e),
        }
    }
    
    // Demo 6: Map topology
    println!("\n6. Map Topology");
    println!("==============");
    
    let topology = map.topology();
    println!("Topology connections: {}", topology.len());
    
    // Generate waypoints for road network analysis
    let waypoints = map.generate_waypoints(10.0); // 10 meter spacing
    println!("Generated waypoints: {}", waypoints.len());
    
    println!("\nRoad network navigation demo completed!");
    
    Ok(())
}