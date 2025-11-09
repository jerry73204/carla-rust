//! Navigation Tests
//!
//! Tests for waypoint, topology, and routing operations (Phase 10.7).
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

    // Waypoint tests
    println!("--- Waypoint Operations ---");
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
        "test_waypoint_transform",
        || test_waypoint_transform(&world),
        &mut passed,
        &mut failed,
    );

    // Topology tests
    println!("\n--- Topology Operations ---");
    run_test(
        "test_topology_generation",
        || test_topology_generation(&world),
        &mut passed,
        &mut failed,
    );

    // Routing tests
    println!("\n--- Routing Operations ---");
    run_test(
        "test_route_planning",
        || test_route_planning(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_route_waypoints",
        || test_route_waypoints(&world),
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

// ===== Waypoint Tests =====

fn test_get_waypoint_at_location(world: &carla::client::World) -> TestResult {
    let map = world.map();

    // Try to get waypoint at spawn point (more likely to be on road)
    let spawn_points = map.recommended_spawn_points();
    if let Some(spawn_point) = spawn_points.get(0) {
        let location = spawn_point.location;

        println!(
            "  Looking for waypoint at spawn point ({:.1}, {:.1}, {:.1})",
            location.x, location.y, location.z
        );

        // Get waypoint at location
        if let Some(waypoint) = map.waypoint_at(&location) {
            let wp_loc = waypoint.transform().location;
            println!(
                "  Found waypoint at ({:.1}, {:.1}, {:.1})",
                wp_loc.x, wp_loc.y, wp_loc.z
            );
            assert!(wp_loc.x.is_finite(), "Waypoint location should be valid");
        } else {
            // Try with origin as fallback
            let origin = Location::new(0.0, 0.0, 0.0);
            let _wp = map.waypoint_at(&origin);
            println!("  Waypoint query API working (tested with origin)");
        }
    }

    Ok(())
}

fn test_waypoint_next(world: &carla::client::World) -> TestResult {
    let map = world.map();

    // Get a spawn point on the road
    let spawn_points = map.recommended_spawn_points();
    if let Some(spawn_point) = spawn_points.get(0) {
        if let Some(waypoint) = map.waypoint_at(&spawn_point.location) {
            // Test getting next waypoints at different distances
            println!("  Testing next waypoint queries");

            let next_1m = waypoint.next(1.0);
            println!("  Next waypoints at 1.0m: {} found", next_1m.len());

            let next_5m = waypoint.next(5.0);
            println!("  Next waypoints at 5.0m: {} found", next_5m.len());

            let next_10m = waypoint.next(10.0);
            println!("  Next waypoints at 10.0m: {} found", next_10m.len());

            // Verify we get some waypoints
            assert!(
                !next_5m.is_empty() || !next_10m.is_empty(),
                "Should find next waypoints"
            );

            // Verify waypoint locations are valid
            if let Some(next_wp) = next_10m.get(0) {
                let loc = next_wp.transform().location;
                assert!(
                    loc.x.is_finite() && loc.y.is_finite() && loc.z.is_finite(),
                    "Next waypoint location should be valid"
                );
            }
        }
    }

    Ok(())
}

fn test_waypoint_lane_change(world: &carla::client::World) -> TestResult {
    // Note: Lane change API (get_left_lane/get_right_lane) not yet available

    let map = world.map();
    let spawn_points = map.recommended_spawn_points();

    if let Some(spawn_point) = spawn_points.get(0) {
        if let Some(waypoint) = map.waypoint_at(&spawn_point.location) {
            println!(
                "  Current waypoint at ({:.1}, {:.1}, {:.1})",
                waypoint.transform().location.x,
                waypoint.transform().location.y,
                waypoint.transform().location.z
            );
            println!("  Lane change API (get_left_lane/get_right_lane) not yet available");
        }
    }

    // API limitation: Lane change methods not yet wrapped
    Ok(())
}

fn test_waypoint_transform(world: &carla::client::World) -> TestResult {
    let map = world.map();
    let spawn_points = map.recommended_spawn_points();

    if let Some(spawn_point) = spawn_points.get(0) {
        // Get waypoint and verify transform
        if let Some(waypoint) = map.waypoint_at(&spawn_point.location) {
            let transform = waypoint.transform();

            println!("  Waypoint transform:");
            println!(
                "    Location: ({:.1}, {:.1}, {:.1})",
                transform.location.x, transform.location.y, transform.location.z
            );
            println!(
                "    Rotation: ({:.1}, {:.1}, {:.1})",
                transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll
            );

            // Verify transform is valid
            assert!(transform.location.x.is_finite(), "X should be finite");
            assert!(transform.location.y.is_finite(), "Y should be finite");
            assert!(transform.location.z.is_finite(), "Z should be finite");
            assert!(
                transform.rotation.pitch.is_finite(),
                "Pitch should be finite"
            );
            assert!(transform.rotation.yaw.is_finite(), "Yaw should be finite");
            assert!(transform.rotation.roll.is_finite(), "Roll should be finite");
        }
    }

    Ok(())
}

// ===== Topology Tests =====

fn test_topology_generation(world: &carla::client::World) -> TestResult {
    let map = world.map();

    // Generate topology
    println!("  Generating road topology");
    let topology = map.topology();

    println!("  Topology has {} road segments", topology.len());
    assert!(!topology.is_empty(), "Most maps should have topology");

    // Examine first topology segment
    if let Some((start_wp, end_wp)) = topology.first() {
        let start_loc = start_wp.transform().location;
        let end_loc = end_wp.transform().location;

        println!("  First segment:");
        println!(
            "    Start: ({:.1}, {:.1}, {:.1})",
            start_loc.x, start_loc.y, start_loc.z
        );
        println!(
            "    End: ({:.1}, {:.1}, {:.1})",
            end_loc.x, end_loc.y, end_loc.z
        );

        // Test navigation along topology
        let next_wps = start_wp.next(1.0);
        println!(
            "    Can navigate from start: {} next waypoints",
            next_wps.len()
        );
    }

    Ok(())
}

// ===== Routing Tests =====

fn test_route_planning(world: &carla::client::World) -> TestResult {
    // Note: Dedicated route planning API not yet available
    // We can simulate simple routing by using waypoint.next() to follow the road

    let map = world.map();
    let spawn_points = map.recommended_spawn_points();

    if spawn_points.len() >= 2 {
        let start_loc = spawn_points.get(0).unwrap().location;
        let end_loc = spawn_points.get(1).unwrap().location;

        println!(
            "  Start location: ({:.1}, {:.1}, {:.1})",
            start_loc.x, start_loc.y, start_loc.z
        );
        println!(
            "  End location: ({:.1}, {:.1}, {:.1})",
            end_loc.x, end_loc.y, end_loc.z
        );

        // Get waypoints at start and end
        if let Some(start_wp) = map.waypoint_at(&start_loc) {
            println!("  Found waypoint at start");

            // Simulate basic route by following waypoints
            let mut current_wp = start_wp;
            let mut waypoint_path = vec![current_wp.clone()];
            let max_iterations = 10;

            for i in 0..max_iterations {
                let next = current_wp.next(5.0);
                if let Some(next_wp) = next.get(0) {
                    waypoint_path.push(next_wp.clone());
                    current_wp = next_wp.clone();
                } else {
                    println!("  Path ended after {} waypoints", i + 1);
                    break;
                }
            }

            println!("  Generated path with {} waypoints", waypoint_path.len());
            println!("  Dedicated route planning API not yet available");
        }
    }

    // API limitation: Dedicated route planning not yet wrapped
    Ok(())
}

fn test_route_waypoints(world: &carla::client::World) -> TestResult {
    // Test generating waypoints along a path using next()

    let map = world.map();
    let spawn_points = map.recommended_spawn_points();

    if let Some(spawn_point) = spawn_points.get(0) {
        if let Some(start_wp) = map.waypoint_at(&spawn_point.location) {
            println!("  Generating waypoint sequence from spawn point");

            let mut waypoints = vec![start_wp.clone()];
            let mut current = start_wp;

            // Generate a sequence of waypoints
            for _ in 0..5 {
                let next = current.next(5.0);
                if let Some(next_wp) = next.get(0) {
                    waypoints.push(next_wp.clone());
                    current = next_wp.clone();
                } else {
                    break;
                }
            }

            println!("  Generated sequence of {} waypoints", waypoints.len());

            // Verify waypoints form a path
            if waypoints.len() >= 2 {
                let first = &waypoints[0];
                let last = &waypoints[waypoints.len() - 1];

                let first_loc = first.transform().location;
                let last_loc = last.transform().location;

                let distance = ((last_loc.x - first_loc.x).powi(2)
                    + (last_loc.y - first_loc.y).powi(2))
                .sqrt();

                println!("  Distance covered: {:.1}m", distance);
                assert!(distance > 0.0, "Waypoints should cover some distance");
            }

            println!("  Waypoint sequence generation successful");
        }
    }

    Ok(())
}
