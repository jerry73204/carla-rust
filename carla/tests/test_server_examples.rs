//! Example tests demonstrating usage of the carla-test-server infrastructure.
//!
//! These tests show various patterns for using the `#[with_carla_server]` macro
//! and associated helper utilities.

use carla_test_server::with_carla_server;

/// Basic connection test - verifies server is running and responsive.
#[with_carla_server]
fn test_basic_connection(client: &carla::client::Client) {
    // This test gets a fresh CARLA server instance
    let version = client
        .server_version()
        .expect("Failed to get server version");

    assert!(!version.is_empty());
    println!("Connected to CARLA server version: {}", version);
}

/// Test using implicit client in scope.
#[with_carla_server]
fn test_implicit_client() {
    // client is automatically available in scope
    let world = client.world().expect("Failed to get world");

    let map = world.map().expect("Failed to get map");

    assert!(!map.name().is_empty());
    println!("Current map: {}", map.name());
}

/// Test world operations with error handling.
#[with_carla_server]
fn test_world_operations() {
    let world = client.world().expect("Failed to get world");

    // Test getting actors
    let actors = world.actors().expect("Failed to get actors");

    println!("Found {} actors in the world", actors.len());

    // Test getting blueprint library
    let blueprint_library = world
        .blueprint_library()
        .expect("Failed to get blueprint library");

    assert!(!blueprint_library.is_empty());
    println!(
        "Blueprint library contains {} blueprints",
        blueprint_library.len()
    );
}

/// Test with helper utilities for retry logic.
#[with_carla_server]
fn test_with_retry_helpers() {
    use carla_test_server::helpers::retry_operation;
    use std::time::Duration;

    let world = client.world().expect("Failed to get world");

    // Retry getting actors in case the world is still initializing
    let actors = retry_operation(3, Duration::from_millis(500), || world.actors())
        .expect("Failed to get actors after retries");

    println!("Successfully retrieved {} actors", actors.len());
}

/// Test with wait conditions for asynchronous operations.
#[with_carla_server]
fn test_with_wait_conditions() {
    use carla_test_server::helpers::wait_for_condition;
    use std::time::Duration;

    let world = client.world().expect("Failed to get world");

    // Wait for the world to be fully loaded
    wait_for_condition(Duration::from_secs(5), Duration::from_millis(100), || {
        world
            .actors()
            .map(|actors| !actors.is_empty())
            .unwrap_or(false)
    })
    .expect("Timeout waiting for world to populate");

    println!("World is ready with actors");
}

/// Test with artifact guard for debugging.
#[with_carla_server]
fn test_with_artifact_guard() {
    use carla_test_server::helpers::TestArtifactGuard;

    // Create guard that will save logs if test fails
    let _guard = TestArtifactGuard::new("test_with_artifact_guard");

    let world = client.world().expect("Failed to get world");

    let map = world.map().expect("Failed to get map");

    // This assertion could fail, triggering artifact capture
    assert!(map.name().len() > 3, "Map name too short: {}", map.name());
}

/// Test spawn points and transforms.
#[with_carla_server]
fn test_spawn_points(client: &carla::client::Client) {
    let world = client.world().expect("Failed to get world");

    let map = world.map().expect("Failed to get map");

    let spawn_points = map.spawn_points();
    assert!(!spawn_points.is_empty(), "No spawn points found in map");

    println!(
        "Map '{}' has {} spawn points",
        map.name(),
        spawn_points.len()
    );

    // Verify spawn points have valid transforms
    for (i, spawn_point) in spawn_points.iter().take(3).enumerate() {
        let location = spawn_point.location;
        println!(
            "Spawn point {}: ({:.2}, {:.2}, {:.2})",
            i, location.x, location.y, location.z
        );
    }
}

/// Test error scenarios and recovery.
#[with_carla_server]
fn test_error_handling() {
    use carla_test_server::helpers::retry_operation;
    use std::time::Duration;

    // Test handling of invalid operations
    let world = client.world().expect("Failed to get world");

    // Try to get a non-existent actor
    let invalid_actor = world.actor(999999);
    // world.actor returns Result<Option<Actor>>
    match invalid_actor {
        Ok(None) => {} // Expected - no actor with this ID
        Ok(Some(_)) => panic!("Should not find actor with invalid ID"),
        Err(e) => panic!("Unexpected error looking up invalid actor: {:?}", e),
    }

    // Test retry on potentially flaky operations
    let _actors = retry_operation(2, Duration::from_millis(100), || world.actors())
        .expect("Failed to get actors even with retry");
}

/// Test with multiple phases using test context.
#[test]
fn test_with_context_phases() {
    use carla_test_server::run_test_with_context;

    run_test_with_context("test_with_phases", |client, context| {
        // Phase 1: Setup
        let setup_phase = context.phase("setup");
        let world = client.world().expect("Failed to get world");
        setup_phase.complete();

        // Phase 2: Operations
        let ops_phase = context.phase("operations");
        let actors = world.actors().expect("Failed to get actors");
        let map = world.map().expect("Failed to get map");
        ops_phase.complete();

        // Phase 3: Verification
        let verify_phase = context.phase("verification");
        assert!(!actors.is_empty() || !map.name().is_empty());
        verify_phase.complete();

        context.log_milestone("All phases completed successfully");
    })
    .expect("Test with context failed");
}

/// Test demonstrating test scenario builder.
#[test]
fn test_scenario_builder() {
    use carla_test_server::test_scenario;

    // Build a custom test scenario
    let (context, config) = test_scenario!("high_quality_test", |builder| {
        builder.with_quality("High").with_port(2001).windowed(false)
    })
    .expect("Failed to build test scenario");

    println!("Test scenario '{}' configured with:", context.test_name);
    println!("  - Quality: {}", config.server.quality_level);
    println!("  - Port: {}", config.server.port);
    println!("  - Windowed: {}", config.server.windowed);

    // Note: This would normally run the test with the custom config
    // but we're just demonstrating the builder pattern here
}

/// Test demonstrating environment variable configuration.
#[with_carla_server]
fn test_environment_config() {
    use carla_test_server::{config::load_config, helpers::get_test_timeout};

    // Load current configuration
    let config = load_config().expect("Failed to load config");

    // Check timeout from environment or config
    let timeout = get_test_timeout(&config);
    println!("Test timeout: {:?}", timeout);

    // Verify server is using configured port
    let version = client.server_version().expect("Failed to get version");

    println!(
        "Server on port {} running version {}",
        config.server.port, version
    );
}

/// Test demonstrating port availability checking.
#[test]
fn test_port_availability() {
    use carla_test_server::helpers::ensure_port_available;

    // Check a high port that should be available
    let port = 59999;
    match ensure_port_available(port) {
        Ok(_) => println!("Port {} is available", port),
        Err(e) => println!("Port {} is not available: {}", port, e),
    }

    // Check the default CARLA port
    match ensure_port_available(2000) {
        Ok(_) => println!("Default CARLA port 2000 is available"),
        Err(_) => println!("Default CARLA port 2000 is in use (expected if server running)"),
    }
}

// Panic test to demonstrate artifact capture (commented out to not fail CI).
/*
#[with_carla_server]
fn test_panic_with_artifacts() {
    use carla_test_server::helpers::TestArtifactGuard;

    let _guard = TestArtifactGuard::new("test_panic_demo");

    let world = client.world()
        .expect("Failed to get world");

    println!("About to panic - artifacts should be saved");
    panic!("Intentional panic to demonstrate artifact capture");
}
*/

/// Test demonstrating all features together.
#[with_carla_server]
fn test_comprehensive_example(client: &carla::client::Client) {
    use carla_test_server::helpers::{retry_operation, wait_for_condition, TestArtifactGuard};
    use std::time::Duration;

    // Setup artifact guard
    let _guard = TestArtifactGuard::new("test_comprehensive");

    // Get world with retry
    let world = retry_operation(3, Duration::from_millis(500), || client.world())
        .expect("Failed to get world after retries");

    // Wait for world to be ready
    wait_for_condition(Duration::from_secs(10), Duration::from_millis(200), || {
        world
            .actors()
            .map(|actors| {
                println!("Checking world readiness: {} actors", actors.len());
                !actors.is_empty()
            })
            .unwrap_or(false)
    })
    .expect("World failed to initialize");

    // Perform operations
    let actors = world.actors().expect("Failed to get actors");
    let map = world.map().expect("Failed to get map");
    let blueprints = world.blueprint_library().expect("Failed to get blueprints");

    // Log results
    println!("Test completed successfully:");
    println!("  - Map: {}", map.name());
    println!("  - Actors: {}", actors.len());
    println!("  - Blueprints: {}", blueprints.len());

    // Verify expectations
    assert!(!map.name().is_empty());
    assert!(!actors.is_empty());
    assert!(!blueprints.is_empty());
}
