//! Landmark and Signal System Demonstration
//!
//! This example demonstrates how to use CARLA's landmark and signal system
//! to detect and analyze traffic signs, speed limits, and other road infrastructure.

use anyhow::Result;
use carla_cxx::{landmark_utils, ClientWrapper, LandmarkInfo, LandmarkType, LaneType};

fn main() -> Result<()> {
    println!("🛑 CARLA Landmark and Signal System Demo");
    println!("========================================");

    // Connect to CARLA server
    let client = ClientWrapper::new("localhost", 2000)?;
    println!("✅ Connected to CARLA server");

    let world = client.get_world();
    println!("🌍 Retrieved world instance");

    let map = world.get_map();
    println!("🗺️  Retrieved map instance");

    // Get all landmarks in the map
    let all_landmarks = map.get_all_landmarks();
    println!(
        "🔍 Found {} total landmarks in the map",
        all_landmarks.len()
    );

    if all_landmarks.is_empty() {
        println!(
            "⚠️  No landmarks found in this map. Try loading a map with traffic infrastructure."
        );
        return Ok(());
    }

    // Analyze landmark distribution by type
    println!("\n📊 Landmark Analysis:");
    println!("===================");

    let groups = landmark_utils::group_by_type(&all_landmarks);
    for (landmark_type, landmarks) in &groups {
        println!("  {:?}: {} landmarks", landmark_type, landmarks.len());
    }

    // Get specific types of landmarks
    let speed_limits = map.get_all_speed_limits();
    let stop_signs = map.get_all_stop_signs();
    let traffic_lights = map.get_all_traffic_lights();

    println!("\n🚦 Infrastructure Summary:");
    println!("  🚀 Speed Limits: {}", speed_limits.len());
    println!("  🛑 Stop Signs: {}", stop_signs.len());
    println!("  🚥 Traffic Lights: {}", traffic_lights.len());

    // Analyze speed limits
    if !speed_limits.is_empty() {
        println!("\n🚀 Speed Limit Analysis:");
        println!("========================");

        let mut speed_values: std::collections::HashMap<i32, usize> =
            std::collections::HashMap::new();
        for landmark in &speed_limits {
            if let Some(speed) = landmark.get_speed_limit() {
                let speed_int = speed as i32;
                *speed_values.entry(speed_int).or_insert(0) += 1;
            }
        }

        for (speed, count) in speed_values {
            println!("  {}km/h: {} signs", speed, count);
        }

        // Show some examples
        println!("\n📝 Speed Limit Examples:");
        for (i, landmark) in speed_limits.iter().take(3).enumerate() {
            println!(
                "  {}. {} (ID: {})",
                i + 1,
                landmark.get_description(),
                landmark.id
            );
            if let Some(speed) = landmark.get_speed_limit() {
                println!(
                    "     Speed: {}km/h, Location: ({:.1}, {:.1})",
                    speed, landmark.h_offset, landmark.z_offset
                );
            }
        }
    }

    // Analyze stop signs
    if !stop_signs.is_empty() {
        println!("\n🛑 Stop Sign Analysis:");
        println!("=====================");

        for (i, landmark) in stop_signs.iter().take(3).enumerate() {
            println!(
                "  {}. {} (ID: {})",
                i + 1,
                landmark.get_description(),
                landmark.id
            );
            println!("     Orientation: {:?}", landmark.orientation);
            println!(
                "     Location: s={:.1}m, h_offset={:.1}m",
                landmark.s, landmark.h_offset
            );
        }
    }

    // Show dynamic landmarks (traffic lights)
    let dynamic_landmarks: Vec<&LandmarkInfo> =
        all_landmarks.iter().filter(|l| l.is_dynamic()).collect();

    if !dynamic_landmarks.is_empty() {
        println!("\n🔄 Dynamic Landmarks (Can Change State):");
        println!("========================================");
        for landmark in dynamic_landmarks.iter().take(5) {
            println!(
                "  • {} ({})",
                landmark.get_description(),
                landmark.type_code
            );
            println!(
                "    Orientation: {:?}, Country: {}",
                landmark.orientation, landmark.country
            );
        }
    }

    // Demonstrate waypoint-based landmark detection
    println!("\n🛣️  Waypoint-Based Landmark Detection:");
    println!("=====================================");

    // Get a spawn point to use as a starting location
    let spawn_points = map.get_recommended_spawn_points();
    if let Some(spawn_point) = spawn_points.first() {
        println!(
            "  Using spawn point: ({:.1}, {:.1}, {:.1})",
            spawn_point.location.x, spawn_point.location.y, spawn_point.location.z
        );

        // Get waypoint at this location
        if let Some(waypoint) =
            map.get_waypoint(&spawn_point.location, true, Some(LaneType::Driving))
        {
            println!("  Found waypoint on road ID: {}", waypoint.get_road_id());

            // Search for landmarks within 100m
            let nearby_landmarks = waypoint.get_all_landmarks_in_distance(100.0, false);
            println!("  Found {} landmarks within 100m", nearby_landmarks.len());

            if !nearby_landmarks.is_empty() {
                println!("  📍 Nearby landmarks:");
                for (i, landmark) in nearby_landmarks.iter().take(5).enumerate() {
                    println!(
                        "    {}. {} (distance: {:.1}m)",
                        i + 1,
                        landmark.get_description(),
                        landmark.distance
                    );
                }

                // Check for speed limit at this location
                if let Some(speed_limit) = waypoint.get_speed_limit(Some(50.0)) {
                    println!("  🚀 Speed limit here: {}km/h", speed_limit);
                } else {
                    println!("  ❓ No speed limit found within 50m");
                }

                // Get speed limit signs specifically
                let speed_limit_signs = waypoint.get_speed_limits_in_distance(100.0, false);
                if !speed_limit_signs.is_empty() {
                    println!("  🚀 Speed limit signs nearby: {}", speed_limit_signs.len());
                    for sign in speed_limit_signs.iter().take(3) {
                        if let Some(limit) = sign.get_speed_limit() {
                            println!("    • {}km/h at distance {:.1}m", limit, sign.distance);
                        }
                    }
                }
            }
        }
    }

    // Search for specific landmark IDs (if any have names/IDs)
    if let Some(first_landmark) = all_landmarks.first() {
        if !first_landmark.id.is_empty() {
            println!("\n🔍 Landmark ID Search Example:");
            println!("=============================");
            let matching_landmarks = map.get_landmarks_from_id(&first_landmark.id);
            println!(
                "  Searching for ID '{}': found {} matches",
                first_landmark.id,
                matching_landmarks.len()
            );
        }
    }

    // Show landmark filtering utilities
    println!("\n🔧 Landmark Utility Functions:");
    println!("=============================");

    let filtered_speed_limits = landmark_utils::filter_speed_limits(&all_landmarks);
    let filtered_stop_signs = landmark_utils::filter_stop_signs(&all_landmarks);
    let filtered_traffic_lights = landmark_utils::filter_traffic_lights(&all_landmarks);

    println!(
        "  • filter_speed_limits(): {} results",
        filtered_speed_limits.len()
    );
    println!(
        "  • filter_stop_signs(): {} results",
        filtered_stop_signs.len()
    );
    println!(
        "  • filter_traffic_lights(): {} results",
        filtered_traffic_lights.len()
    );

    // Find nearest landmarks of specific types
    if let Some(nearest_speed_limit) =
        landmark_utils::find_nearest_of_type(&all_landmarks, LandmarkType::SpeedLimit)
    {
        println!(
            "  • Nearest speed limit: {} at distance {:.1}m",
            nearest_speed_limit.get_description(),
            nearest_speed_limit.distance
        );
    }

    if let Some(nearest_stop) =
        landmark_utils::find_nearest_of_type(&all_landmarks, LandmarkType::Stop)
    {
        println!(
            "  • Nearest stop sign: {} at distance {:.1}m",
            nearest_stop.get_description(),
            nearest_stop.distance
        );
    }

    // Get applicable speed limit (closest one)
    if let Some(applicable_speed) = landmark_utils::get_applicable_speed_limit(&all_landmarks) {
        println!("  • Applicable speed limit: {}km/h", applicable_speed);
    }

    println!("\n📋 Landmark Information Structure:");
    println!("=================================");
    if let Some(example) = all_landmarks.first() {
        println!("  Example landmark details:");
        println!("    ID: '{}'", example.id);
        println!("    Name: '{}'", example.name);
        println!(
            "    Type Code: '{}' -> {:?}",
            example.type_code, example.landmark_type
        );
        println!("    Sub Type: '{}'", example.sub_type);
        println!("    Description: '{}'", example.get_description());
        println!("    Value: {} {}", example.value, example.unit);
        println!(
            "    Dimensions: {:.1}m × {:.1}m",
            example.width, example.height
        );
        println!(
            "    Position: s={:.1}m, h_offset={:.1}m, z_offset={:.1}m",
            example.s, example.h_offset, example.z_offset
        );
        println!("    Orientation: {:?}", example.orientation);
        println!("    Country: '{}'", example.country);
        println!("    Dynamic: {}", example.is_dynamic());
        println!("    Text: '{}'", example.text);

        // Show type-specific information
        if example.is_speed_limit() {
            println!("    ➜ This is a speed limit sign");
            if let Some(limit) = example.get_speed_limit() {
                println!("    ➜ Speed limit: {}km/h", limit);
            }
        } else if example.is_stop_sign() {
            println!("    ➜ This is a stop sign");
        } else if example.is_yield_sign() {
            println!("    ➜ This is a yield sign");
        } else if example.is_traffic_light() {
            println!("    ➜ This is a traffic light");
        }
    }

    println!("\n✨ Landmark detection completed!");
    println!("💡 The landmark system allows you to:");
    println!("   • Access all road infrastructure from the map");
    println!("   • Search for landmarks near specific waypoints");
    println!("   • Filter by type (speed limits, stop signs, traffic lights)");
    println!("   • Analyze road regulations and driving conditions");
    println!("   • Build autonomous driving algorithms that respect traffic rules");
    println!("   • Get detailed information about each landmark's properties");

    Ok(())
}
