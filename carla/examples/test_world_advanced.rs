//! World Operations Tests
//!
//! Tests for traffic lights, landmarks, and environment objects (Phase 10.6).
//!
//! # Test Categories
//! - Traffic lights: State changes, timing, freezing
//! - Landmarks: Queries, filtering, waypoints
//! - Environment objects: Queries, enable/disable
//! - World operations: Tick timeout
//!
//! Run with:
//! ```bash
//! cargo run --example test_world_advanced --profile dev-release
//! ```

use carla::client::{ActorBase, Client};
use std::{thread, time::Duration};

type TestResult = Result<(), Box<dyn std::error::Error>>;

fn main() {
    println!("=== World Operations Tests ===\n");

    let client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();
    println!("Connected to CARLA server\n");

    let mut passed = 0;
    let mut failed = 0;

    // Traffic light tests
    println!("--- Traffic Light Operations ---");
    run_test(
        "test_get_traffic_lights",
        || test_get_traffic_lights(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_traffic_light_state_change",
        || test_traffic_light_state_change(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_traffic_light_timing",
        || test_traffic_light_timing(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_traffic_light_freeze",
        || test_traffic_light_freeze(&mut world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_reset_all_traffic_lights",
        || test_reset_all_traffic_lights(&mut world),
        &mut passed,
        &mut failed,
    );

    // Landmark tests
    println!("\n--- Landmark Operations ---");
    run_test(
        "test_get_landmarks",
        || test_get_landmarks(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_landmarks_by_type",
        || test_landmarks_by_type(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_landmark_waypoints",
        || test_landmark_waypoints(&world),
        &mut passed,
        &mut failed,
    );

    // Environment object tests
    println!("\n--- Environment Object Operations ---");
    run_test(
        "test_environment_objects_query",
        || test_environment_objects_query(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_environment_object_enable_disable",
        || test_environment_object_enable_disable(&world),
        &mut passed,
        &mut failed,
    );

    // World operations
    println!("\n--- World Operations ---");
    run_test(
        "test_world_tick_timeout",
        || test_world_tick_timeout(&mut world),
        &mut passed,
        &mut failed,
    );

    println!("\n=== Results ===");
    println!("Passed: {}", passed);
    println!("Failed: {}", failed);
    std::process::exit(if failed > 0 { 1 } else { 0 });
}

fn run_test<F>(name: &str, test_fn: F, passed: &mut i32, failed: &mut i32)
where
    F: FnOnce() -> TestResult,
{
    print!("Testing {}... ", name);
    match test_fn() {
        Ok(_) => {
            println!("✓ PASS");
            *passed += 1;
        }
        Err(e) => {
            println!("✗ FAIL: {}", e);
            *failed += 1;
        }
    }
}

// ===== Traffic Light Tests =====

fn test_get_traffic_lights(world: &carla::client::World) -> TestResult {
    // Query all actors and filter for traffic lights
    let actors = world.actors();

    // Traffic lights have type_id starting with "traffic.traffic_light"
    let mut traffic_light_count = 0;
    for actor in actors.iter() {
        let type_id = actor.type_id();
        if type_id.starts_with("traffic.traffic_light") {
            traffic_light_count += 1;
        }
    }

    // Most maps have some traffic lights, but we can't guarantee
    // Just verify we can query them
    println!("  Found {} traffic lights", traffic_light_count);

    Ok(())
}

fn test_traffic_light_state_change(world: &carla::client::World) -> TestResult {
    // Note: Individual traffic light state control API not yet available
    // This test verifies we can identify traffic lights through actors

    let actors = world.actors();
    let traffic_lights: Vec<_> = actors
        .iter()
        .filter(|a| a.type_id().starts_with("traffic.traffic_light"))
        .collect();

    if !traffic_lights.is_empty() {
        println!(
            "  Can identify {} traffic lights via actors",
            traffic_lights.len()
        );

        // Get properties of first traffic light
        if let Some(tl) = traffic_lights.first() {
            let location = tl.location();
            println!(
                "  Traffic light location: ({:.1}, {:.1}, {:.1})",
                location.x, location.y, location.z
            );
        }
    }

    // API limitation: Individual traffic light control not yet wrapped
    Ok(())
}

fn test_traffic_light_timing(world: &carla::client::World) -> TestResult {
    // Note: Traffic light timing API not yet available
    // This test documents the limitation

    let actors = world.actors();
    let traffic_light_count = actors
        .iter()
        .filter(|a| a.type_id().starts_with("traffic.traffic_light"))
        .count();

    println!(
        "  Timing API not yet available for {} traffic lights",
        traffic_light_count
    );

    // API limitation: Traffic light timing control not yet wrapped
    Ok(())
}

fn test_traffic_light_freeze(world: &mut carla::client::World) -> TestResult {
    // Test freezing all traffic lights
    println!("  Freezing all traffic lights");
    world.freeze_all_traffic_lights(true);
    thread::sleep(Duration::from_millis(100));

    println!("  Unfreezing all traffic lights");
    world.freeze_all_traffic_lights(false);
    thread::sleep(Duration::from_millis(100));

    // Verify freeze/unfreeze completes without error
    Ok(())
}

fn test_reset_all_traffic_lights(world: &mut carla::client::World) -> TestResult {
    // Reset all traffic lights to default state
    println!("  Resetting all traffic lights to default");
    world.reset_all_traffic_lights();
    thread::sleep(Duration::from_millis(100));

    // Verify reset completes without error
    Ok(())
}

// ===== Landmark Tests =====

fn test_get_landmarks(world: &carla::client::World) -> TestResult {
    // Note: Landmark API not yet available in Rust bindings
    // This test documents the limitation and tests what we can

    let map = world.map();
    let name = map.name();

    println!("  Current map: {}", name);
    println!("  Landmark query API not yet available");

    // Verify we can at least access the map
    assert!(!name.is_empty(), "Map name should not be empty");

    // API limitation: Landmark query not yet wrapped
    Ok(())
}

fn test_landmarks_by_type(world: &carla::client::World) -> TestResult {
    // Note: Landmark filtering API not yet available

    let map = world.map();
    let waypoints = map.generate_waypoints(10.0);

    println!(
        "  Generated {} waypoints (landmarks would be nearby)",
        waypoints.len()
    );
    println!("  Landmark type filtering API not yet available");

    // API limitation: Landmark type filtering not yet wrapped
    Ok(())
}

fn test_landmark_waypoints(world: &carla::client::World) -> TestResult {
    // Note: Landmark waypoint association API not yet available

    let map = world.map();
    let waypoints = map.generate_waypoints(10.0);

    if !waypoints.is_empty() {
        if let Some(wp) = waypoints.get(0) {
            println!(
                "  Waypoint at ({:.1}, {:.1}, {:.1})",
                wp.transform().location.x,
                wp.transform().location.y,
                wp.transform().location.z
            );
            println!("  Landmark waypoint association API not yet available");
        }
    }

    // API limitation: Landmark waypoint queries not yet wrapped
    Ok(())
}

// ===== Environment Object Tests =====

fn test_environment_objects_query(world: &carla::client::World) -> TestResult {
    // Query all environment objects using full type mask
    let objects = world.environment_objects(0xFF);

    println!("  Found {} environment objects", objects.len());

    assert!(
        !objects.is_empty(),
        "Most maps should have environment objects"
    );

    // Examine first few objects
    let sample_count = 3.min(objects.len());
    for i in 0..sample_count {
        if let Some(obj) = objects.get(i) {
            let location = obj.transform().location;
            println!(
                "  Object {}: ID={}, location=({:.1}, {:.1}, {:.1})",
                i + 1,
                obj.id(),
                location.x,
                location.y,
                location.z
            );
        }
    }

    Ok(())
}

fn test_environment_object_enable_disable(world: &carla::client::World) -> TestResult {
    // Get environment objects
    let objects = world.environment_objects(0xFF);

    if objects.is_empty() {
        println!("  No environment objects to toggle");
        return Ok(());
    }

    // Collect IDs of first few objects to toggle
    let toggle_count = 5.min(objects.len());
    let mut ids_to_toggle = Vec::new();
    for i in 0..toggle_count {
        if let Some(obj) = objects.get(i) {
            ids_to_toggle.push(obj.id());
        }
    }

    if !ids_to_toggle.is_empty() {
        println!("  Disabling {} environment objects", ids_to_toggle.len());
        world.enable_environment_objects(&ids_to_toggle, false);
        thread::sleep(Duration::from_millis(200));

        println!("  Re-enabling {} environment objects", ids_to_toggle.len());
        world.enable_environment_objects(&ids_to_toggle, true);
        thread::sleep(Duration::from_millis(200));
    }

    // Verify toggle completes without error
    Ok(())
}

// ===== World Operation Tests =====

fn test_world_tick_timeout(world: &mut carla::client::World) -> TestResult {
    // Test world tick with default timeout
    println!("  Performing world tick");
    let tick_result = world.tick();

    // Tick returns the frame number
    println!("  Tick completed, frame: {}", tick_result);

    assert!(tick_result > 0, "Tick should return valid frame number");

    // Test multiple ticks
    let tick2 = world.tick();
    println!("  Second tick completed, frame: {}", tick2);

    assert!(
        tick2 > tick_result,
        "Frame number should increment on each tick"
    );

    Ok(())
}
