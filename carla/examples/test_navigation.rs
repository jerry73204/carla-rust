//! Navigation Tests
//!
//! Tests for waypoint, topology, and routing operations (Phase 8).
//!
//! # Test Categories
//! - Waypoint queries: Location lookup, next waypoints, lane changes
//! - Topology: Road network generation
//! - Routing: Path planning, waypoint sequences
//!
//! Run with:
//! ```bash
//! cargo run --example test_navigation --profile dev-release
//! ```

use carla::{client::Client, geom::Location};

type TestResult = Result<(), Box<dyn std::error::Error>>;

fn main() {
    println!("=== Navigation Tests ===\n");

    let client = Client::connect("127.0.0.1", 2000, None);
    let world = client.world();
    println!("Connected to CARLA server\n");

    let mut passed = 0;
    let mut failed = 0;

    run_test(
        "test_get_waypoint_at_location",
        || test_get_waypoint_at_location(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_waypoint_next",
        || test_waypoint_next(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_waypoint_lane_change",
        || test_waypoint_lane_change(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_topology_generation",
        || test_topology_generation(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_route_planning",
        test_route_planning,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_route_waypoints",
        test_route_waypoints,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_waypoint_transform",
        || test_waypoint_transform(&world),
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

fn test_get_waypoint_at_location(world: &carla::client::World) -> TestResult {
    let map = world.map();
    let location = Location::new(0.0, 0.0, 0.0);

    // Get waypoint at location
    let _waypoint = map.waypoint_at(&location);
    Ok(())
}

fn test_waypoint_next(world: &carla::client::World) -> TestResult {
    let map = world.map();
    let location = Location::new(0.0, 0.0, 0.0);

    // Get waypoint at location
    if let Some(waypoint) = map.waypoint_at(&location) {
        // Get next waypoints
        let _next = waypoint.next(1.0);
    }
    Ok(())
}

fn test_waypoint_lane_change(_world: &carla::client::World) -> TestResult {
    // TODO: Implement when lane change API is available
    Ok(())
}

fn test_topology_generation(world: &carla::client::World) -> TestResult {
    let map = world.map();

    // Generate topology
    let _topology = map.topology();
    Ok(())
}

fn test_route_planning() -> TestResult {
    // TODO: Implement when routing API is available
    Ok(())
}

fn test_route_waypoints() -> TestResult {
    // TODO: Implement when route API is available
    Ok(())
}

fn test_waypoint_transform(world: &carla::client::World) -> TestResult {
    let map = world.map();
    let location = Location::new(0.0, 0.0, 0.0);

    // Get waypoint and verify transform
    if let Some(waypoint) = map.waypoint_at(&location) {
        let transform = waypoint.transform();
        assert!(transform.location.x.is_finite());
    }
    Ok(())
}
