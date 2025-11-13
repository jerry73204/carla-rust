//! Dynamic World Manipulation Integration Test
//!
//! This integration test demonstrates dynamic world manipulation:
//! - Load map with layers
//! - Query and manipulate environment objects
//! - Freeze and manipulate traffic lights
//! - Change weather conditions
//! - Test map operations
//!
//! This validates the integration of world manipulation features.
//!
//! Run with:
//! ```bash
//! cargo run --example integration_dynamic_world --profile dev-release
//! ```

use carla::{client::Client, geom::Location};
use std::time::Duration;

fn main() {
    println!("=== Dynamic World Manipulation Integration Test ===\n");

    // Connect to CARLA
    let client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();
    println!("Connected to CARLA server");

    // Test 1: Query world information
    println!("\n--- Test 1: Query world information ---");
    test_world_info(&world);

    // Test 2: Environment objects manipulation
    println!("\n--- Test 2: Environment objects ---");
    test_environment_objects(&world);

    // Test 3: Traffic light manipulation
    println!("\n--- Test 3: Traffic light manipulation ---");
    test_traffic_lights(&mut world);

    // Test 4: Weather manipulation
    println!("\n--- Test 4: Weather manipulation ---");
    test_weather_changes(&mut world);

    // Test 5: Map operations
    println!("\n--- Test 5: Map operations ---");
    test_map_operations(&world);

    println!("\n=== All Tests Passed ===");
    println!("✅ Integration test completed successfully!");
    std::process::exit(0);
}

fn test_world_info(world: &carla::client::World) {
    // Get world ID
    let world_id = world.id();
    println!("✓ World ID: {}", world_id);

    // Get map name
    let map = world.map();
    let map_name = map.name();
    println!("✓ Map name: {}", map_name);

    // Get spawn points
    let spawn_points = map.recommended_spawn_points();
    println!("✓ Available spawn points: {}", spawn_points.len());

    // Get topology
    let topology = map.topology();
    println!("✓ Road topology segments: {}", topology.len());

    assert!(!map_name.is_empty(), "Map name should not be empty");
    assert!(!spawn_points.is_empty(), "Should have spawn points");
}

fn test_environment_objects(world: &carla::client::World) {
    // Query all environment objects
    let objects = world.environment_objects(0xFF);
    println!("✓ Found {} environment objects", objects.len());

    if !objects.is_empty() {
        // Try to disable some objects (collect IDs from first 5 objects)
        let mut ids_to_toggle = Vec::new();
        let count = objects.len().min(5);
        for i in 0..count {
            if let Some(obj) = objects.get(i) {
                ids_to_toggle.push(obj.id());
            }
        }

        if !ids_to_toggle.is_empty() {
            println!("  Disabling {} objects...", ids_to_toggle.len());
            world.enable_environment_objects(&ids_to_toggle, false);
            std::thread::sleep(Duration::from_millis(500));

            println!("  Re-enabling objects...");
            world.enable_environment_objects(&ids_to_toggle, true);
            println!("✓ Successfully toggled environment objects");
        }
    }
}

fn test_traffic_lights(world: &mut carla::client::World) {
    // Freeze all traffic lights
    println!("  Freezing all traffic lights...");
    world.freeze_all_traffic_lights(true);
    std::thread::sleep(Duration::from_millis(500));
    println!("✓ Traffic lights frozen");

    // Reset all traffic lights
    println!("  Resetting all traffic lights...");
    world.reset_all_traffic_lights();
    std::thread::sleep(Duration::from_millis(500));
    println!("✓ Traffic lights reset");

    // Unfreeze traffic lights
    println!("  Unfreezing traffic lights...");
    world.freeze_all_traffic_lights(false);
    println!("✓ Traffic lights unfrozen");
}

fn test_weather_changes(world: &mut carla::client::World) {
    // Test 1: Default weather
    let original_weather = world.weather();
    println!("  Original weather:");
    println!("    Cloudiness: {:.1}", original_weather.cloudiness);
    println!("    Precipitation: {:.1}", original_weather.precipitation);
    println!(
        "    Sun altitude: {:.1}°",
        original_weather.sun_altitude_angle
    );

    // Test 2: Set rainy weather
    println!("\n  Setting rainy weather...");
    let mut rainy_weather = world.weather();
    rainy_weather.cloudiness = 90.0;
    rainy_weather.precipitation = 80.0;
    rainy_weather.precipitation_deposits = 50.0;
    rainy_weather.wetness = 80.0;
    world.set_weather(&rainy_weather);
    std::thread::sleep(Duration::from_millis(500));

    let current_weather = world.weather();
    assert!(
        (current_weather.cloudiness - 90.0).abs() < 1.0,
        "Cloudiness should be set"
    );
    assert!(
        (current_weather.precipitation - 80.0).abs() < 1.0,
        "Precipitation should be set"
    );
    println!("✓ Rainy weather set successfully");

    // Test 3: Set sunny weather
    println!("\n  Setting sunny weather...");
    let mut sunny_weather = world.weather();
    sunny_weather.cloudiness = 10.0;
    sunny_weather.precipitation = 0.0;
    sunny_weather.wetness = 0.0;
    sunny_weather.sun_altitude_angle = 45.0;
    world.set_weather(&sunny_weather);
    std::thread::sleep(Duration::from_millis(500));

    let current_weather = world.weather();
    assert!(
        (current_weather.cloudiness - 10.0).abs() < 1.0,
        "Cloudiness should be low"
    );
    assert!(
        current_weather.precipitation < 1.0,
        "Precipitation should be zero"
    );
    println!("✓ Sunny weather set successfully");

    // Test 4: Restore original weather
    println!("\n  Restoring original weather...");
    world.set_weather(&original_weather);
    println!("✓ Weather restored");
}

fn test_map_operations(world: &carla::client::World) {
    let map = world.map();

    // Test waypoint lookup
    let test_location = Location::new(0.0, 0.0, 0.0);
    if let Some(waypoint) = map.waypoint_at(&test_location) {
        println!("✓ Found waypoint at test location");
        println!(
            "  Waypoint location: ({:.1}, {:.1}, {:.1})",
            waypoint.transform().location.x,
            waypoint.transform().location.y,
            waypoint.transform().location.z
        );

        // Test waypoint navigation
        let next_waypoints = waypoint.next(5.0);
        println!("✓ Next waypoints: {} found", next_waypoints.len());
    } else {
        println!("⚠️  No waypoint found at (0,0,0) - using spawn point instead");

        let spawn_points = map.recommended_spawn_points();
        if let Some(spawn_point) = spawn_points.get(0) {
            if let Some(waypoint) = map.waypoint_at(&spawn_point.location) {
                println!("✓ Found waypoint at spawn point");
                let next_waypoints = waypoint.next(5.0);
                println!("✓ Next waypoints: {} found", next_waypoints.len());
            }
        }
    }

    // Test topology
    let topology = map.topology();
    if !topology.is_empty() {
        println!("✓ Topology has {} road segments", topology.len());

        // Get waypoints from first topology segment
        if let Some((start_wp, _end_wp)) = topology.first() {
            let next_wps = start_wp.next(1.0);
            println!(
                "✓ Can navigate from topology waypoint: {} next waypoints",
                next_wps.len()
            );
        }
    }

    // Test waypoint generation
    let all_waypoints = map.generate_waypoints(2.0);
    println!("✓ Generated {} waypoints across map", all_waypoints.len());
    assert!(!all_waypoints.is_empty(), "Should generate waypoints");
}
