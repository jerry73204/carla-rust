//! Integration tests for simulator connection and basic operations
//!
//! These tests require a running CARLA simulator. Run with:
//! ```bash
//! cargo test --test test_simulator_connection -- --ignored
//! ```

#[path = "test_utils/mod.rs"]
mod test_utils;

use carla::client::ActorBase;
use serial_test::serial;
use test_utils::{
    cleanup_all_actors, connect_to_simulator, wait_for_simulator_ready, with_simulator,
    with_simulator_client, SimulatorConfig,
};

#[test]
#[serial]
#[ignore] // Only run when simulator is available
fn test_simulator_connection() {
    let config = SimulatorConfig::default();
    let client = connect_to_simulator(&config).expect("Failed to connect to simulator");

    // Verify we can get the server version
    let version = client.server_version();
    println!("Server version: {}", version);
}

#[test]
#[serial]
#[ignore]
fn test_simulator_reconnection() {
    let config = SimulatorConfig::default();

    // First connection
    let client1 = connect_to_simulator(&config).expect("First connection should succeed");
    let version1 = client1.server_version();

    // Drop first client
    drop(client1);

    // Second connection should also work
    let client2 = connect_to_simulator(&config).expect("Second connection should succeed");
    let version2 = client2.server_version();

    assert_eq!(version1, version2, "Server version should be consistent");
}

#[test]
#[serial]
#[ignore]
fn test_simulator_ready_detection() {
    let config = SimulatorConfig::default();
    let client = connect_to_simulator(&config).expect("Failed to connect");
    let world = client.world();

    // Should detect simulator is ready
    let result = wait_for_simulator_ready(&world, std::time::Duration::from_secs(10));
    assert!(result.is_ok(), "Simulator should be ready: {:?}", result);
}

#[test]
#[serial]
#[ignore]
fn test_cleanup_between_tests() {
    with_simulator(|world| {
        // Get blueprint library
        let blueprint_library = world.blueprint_library();
        let vehicle_bp = blueprint_library
            .find("vehicle.tesla.model3")
            .expect("Should find vehicle blueprint");

        // Get spawn points
        let spawn_points = world.map().recommended_spawn_points();
        assert!(!spawn_points.is_empty(), "Map should have spawn points");

        // Spawn a vehicle
        let spawn_point = spawn_points
            .get(0)
            .expect("Should have at least one spawn point");
        let vehicle = world
            .spawn_actor(&vehicle_bp, &spawn_point)
            .expect("Should spawn vehicle");

        assert!(vehicle.is_alive(), "Vehicle should be alive after spawn");

        // Cleanup (currently just logs actors)
        cleanup_all_actors(world).expect("Cleanup should succeed");

        // Note: Actor destruction not yet implemented in Rust API
        // So we can't verify actors are destroyed
    });
}

#[test]
#[serial]
#[ignore]
fn test_with_simulator_helper() {
    with_simulator(|world| {
        // Should have a valid world
        let map = world.map();
        let map_name = map.name();
        assert!(!map_name.is_empty(), "Map should have a name");

        println!("Current map: {}", map_name);

        // Should have spawn points
        let spawn_points = map.recommended_spawn_points();
        assert!(!spawn_points.is_empty(), "Map should have spawn points");

        println!("Spawn points: {}", spawn_points.len());
    });
}

#[test]
#[serial]
#[ignore]
fn test_with_simulator_client_helper() {
    with_simulator_client(|client| {
        // Should have access to client methods
        let version = client.server_version();
        println!("Server version: {}", version);

        // Should be able to get world
        let world = client.world();
        let map_name = world.map().name();
        assert!(!map_name.is_empty(), "Map should have a name");
    });
}

#[test]
#[serial]
#[ignore]
fn test_exclusive_simulator_access() {
    // This test verifies that with_simulator provides exclusive access
    // by checking that we can spawn and cleanup without interference

    with_simulator(|world| {
        let blueprint_library = world.blueprint_library();
        let vehicle_bp = blueprint_library
            .find("vehicle.tesla.model3")
            .expect("Should find vehicle blueprint");

        let spawn_points = world.map().recommended_spawn_points();
        let spawn_point = spawn_points.get(0).expect("Should have spawn point");

        // Spawn vehicle
        let _vehicle = world
            .spawn_actor(&vehicle_bp, &spawn_point)
            .expect("Should spawn vehicle with exclusive access");

        // If another test was running concurrently, we might have conflicts
        // The #[serial] attribute prevents this
    });

    // After the test, cleanup should have removed all actors
    // Next test should have a clean slate
}
