//! Landmark Detection Demo
//!
//! This example demonstrates how to use the Waypoint landmark detection API
//! to find road signs, speed limits, and other landmarks in CARLA.
//!
//! # Usage
//! ```bash
//! cargo run --example landmark_demo --profile dev-release
//! ```

use anyhow::Result;
use carla::client::Client;

fn main() -> Result<()> {
    println!("Landmark Detection Demo - Connecting to CARLA...\n");

    // Connect to CARLA
    let client = Client::connect("localhost", 2000, 0);
    let world = client.world();
    let map = world.map();

    println!("Map: {}\n", map.name());

    // Get spawn points to find landmarks
    let spawn_points = map.recommended_spawn_points();

    if spawn_points.is_empty() {
        return Err(anyhow::anyhow!("No spawn points available"));
    }

    println!("=== Finding Landmarks Around Spawn Points ===\n");

    // Check landmarks around first few spawn points
    for (i, spawn_transform) in spawn_points.as_slice().iter().take(5).enumerate() {
        let waypoint = match map.waypoint_at(&spawn_transform.location) {
            Some(wp) => wp,
            None => {
                println!("Spawn point {}: No waypoint found", i);
                continue;
            }
        };

        println!(
            "Spawn Point {}: Road ID {}, Lane ID {}",
            i,
            waypoint.road_id(),
            waypoint.lane_id()
        );

        // Method 1: Get all landmarks within 50 meters (Rust-idiomatic method)
        let landmarks = waypoint.all_landmarks_in_distance(50.0, false);
        println!("  Found {} landmarks within 50m", landmarks.len());

        if !landmarks.is_empty() {
            for (j, landmark) in landmarks.iter().take(3).enumerate() {
                println!(
                    "    [{}] {} (type: {}, distance: {:.2}m, value: {})",
                    j,
                    landmark.name(),
                    landmark.type_(),
                    landmark.distance(),
                    landmark.value()
                );
            }
            if landmarks.len() > 3 {
                println!("    ... and {} more", landmarks.len() - 3);
            }
        }
        println!();
    }

    println!("=== Using Python-Compatible API ===\n");

    // Demonstrate Python-compatible method aliases
    let spawn_transform = &spawn_points.as_slice()[0];
    if let Some(waypoint) = map.waypoint_at(&spawn_transform.location) {
        // Method 2: Python-compatible alias (same functionality)
        let landmarks = waypoint.get_landmarks(100.0, false);
        println!(
            "Using get_landmarks(): Found {} landmarks within 100m",
            landmarks.len()
        );

        // Group landmarks by type
        let mut types_count: std::collections::HashMap<String, usize> =
            std::collections::HashMap::new();
        for landmark in landmarks.iter() {
            *types_count.entry(landmark.type_()).or_insert(0) += 1;
        }

        println!("\nLandmark types found:");
        for (type_, count) in types_count.iter() {
            println!("  Type {}: {} landmarks", type_, count);
        }
    }

    println!("\n=== Filtering Landmarks by Type ===\n");

    // Common landmark type codes:
    // "1000001" - Speed Limit signs
    // "206"     - Stop signs
    // "205"     - Yield signs
    // "207"     - Traffic lights

    let test_types = vec![
        ("1000001", "Speed Limit"),
        ("206", "Stop Sign"),
        ("205", "Yield Sign"),
        ("207", "Traffic Light"),
    ];

    for spawn_transform in spawn_points.as_slice().iter().take(10) {
        if let Some(waypoint) = map.waypoint_at(&spawn_transform.location) {
            for (type_code, type_name) in &test_types {
                // Method 3: Filter by type (Rust-idiomatic)
                let filtered = waypoint.landmarks_of_type_in_distance(100.0, type_code, false);

                if !filtered.is_empty() {
                    println!(
                        "Found {} {} landmark(s) at Road {}, Lane {}:",
                        filtered.len(),
                        type_name,
                        waypoint.road_id(),
                        waypoint.lane_id()
                    );

                    for landmark in filtered.iter().take(2) {
                        println!(
                            "  - {} at {:.2}m (value: {})",
                            landmark.name(),
                            landmark.distance(),
                            landmark.value()
                        );

                        // Show landmark transform
                        let transform = landmark.transform();
                        println!(
                            "    Position: ({:.2}, {:.2}, {:.2})",
                            transform.location.x, transform.location.y, transform.location.z
                        );
                    }
                }
            }
        }
    }

    println!("\n=== Using Python-Compatible Type Filtering ===\n");

    // Method 4: Python-compatible type filtering alias
    if let Some(waypoint) = map.waypoint_at(&spawn_points.as_slice()[0].location) {
        let speed_limits = waypoint.get_landmarks_of_type(150.0, "1000001", false);

        if !speed_limits.is_empty() {
            println!("Speed limits ahead:");
            for limit in speed_limits.iter() {
                println!(
                    "  {} km/h at {:.2}m ({})",
                    limit.value(),
                    limit.distance(),
                    limit.name()
                );
            }
        } else {
            println!("No speed limit signs found within 150m");
        }
    }

    println!("\n=== Landmark Properties Demo ===\n");

    // Find any landmark to demonstrate all available properties
    for spawn_transform in spawn_points.as_slice().iter() {
        if let Some(waypoint) = map.waypoint_at(&spawn_transform.location) {
            let landmarks = waypoint.get_landmarks(200.0, false);

            if let Some(landmark) = landmarks.get(0) {
                println!("Detailed landmark properties:");
                println!("  ID: {}", landmark.id());
                println!("  Name: {}", landmark.name());
                println!("  Type: {}", landmark.type_());
                println!("  Sub-type: {}", landmark.sub_type());
                println!("  Value: {}", landmark.value());
                println!("  Distance: {:.2}m", landmark.distance());
                println!("  Road ID: {}", landmark.road_id());
                println!("  Is Dynamic: {}", landmark.is_dynamic());
                // Note: orientation() returns C++ enum without Display/Debug trait
                println!("  Z-Offset: {}", landmark.z_offset());
                println!("  Country: {}", landmark.country());
                println!("  Height: {}", landmark.height());
                println!("  Width: {}", landmark.width());
                println!("  Text: {}", landmark.text());
                println!("  H-Offset: {}", landmark.h_offset());
                println!("  Pitch: {}", landmark.pitch());
                println!("  Roll: {}", landmark.roll());

                let transform = landmark.transform();
                println!(
                    "  Transform: pos=({:.2}, {:.2}, {:.2})",
                    transform.location.x, transform.location.y, transform.location.z
                );

                // Get the waypoint where the landmark is effective
                if let Some(lm_waypoint) = landmark.waypoint() {
                    println!(
                        "  Effective at: Road {}, Lane {}",
                        lm_waypoint.road_id(),
                        lm_waypoint.lane_id()
                    );
                } else {
                    println!("  Effective waypoint: None");
                }

                break;
            }
        }
    }

    println!("\n✅ SUCCESS: Landmark detection demo completed!");
    println!("\n📝 Summary:");
    println!("  - Demonstrated all_landmarks_in_distance() [Rust method]");
    println!("  - Demonstrated get_landmarks() [Python-compatible alias]");
    println!("  - Demonstrated landmarks_of_type_in_distance() [Rust method]");
    println!("  - Demonstrated get_landmarks_of_type() [Python-compatible alias]");
    println!("  - Showed all 18+ landmark properties");

    Ok(())
}
