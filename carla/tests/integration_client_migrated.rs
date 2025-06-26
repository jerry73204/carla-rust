//! Client integration tests - Migrated to use carla-test-server
//!
//! This file demonstrates how to migrate existing tests to use the new
//! #[with_carla_server] macro for automatic server management.

use carla::{actor::ActorExt, client::WeatherParameters};
use carla_test_server::with_carla_server;
use std::time::Duration;

/// Test basic client connection - migrated version.
///
/// Before:
/// ```rust
/// #[test]
/// #[serial]
/// #[cfg(feature = "test-carla-server")]
/// fn test_client_connection() -> CarlaResult<()> {
///     let client = get_test_client()?;
///     let version = client.server_version()?;
///     // ... test logic
/// }
/// ```
///
/// After: Using the macro with explicit client parameter
#[with_carla_server]
fn test_client_connection_migrated(client: &carla::client::Client) {
    // No need to manually create client - it's provided
    let version = client
        .server_version()
        .expect("Failed to get server version");

    println!("Connected to CARLA server version: {}", version);
    assert!(version.starts_with("0.10"));
}

/// Test available maps - migrated to implicit client style.
#[with_carla_server]
fn test_available_maps_migrated() {
    // client is automatically available in scope
    let maps = client
        .available_maps()
        .expect("Failed to get available maps");

    println!("Available maps: {:?}", maps);
    assert!(!maps.is_empty());

    // Maps should contain some expected CARLA maps
    let map_names: Vec<String> = maps.iter().map(|m| m.to_lowercase()).collect();
    let has_town = map_names.iter().any(|name| name.contains("town"));
    assert!(has_town, "Should have at least one Town map available");
}

/// Test world access - demonstrating error handling patterns.
#[with_carla_server]
fn test_world_access_migrated(client: &carla::client::Client) {
    let world = client.world().expect("Failed to get world");

    // Get world ID
    let world_id = world.id();
    println!("Current world ID: {}", world_id);
    assert!(world_id > 0);

    // Get world settings
    let settings = world.settings().expect("Failed to get world settings");

    println!(
        "World settings: synchronous_mode={}, no_rendering_mode={}",
        settings.synchronous_mode, settings.no_rendering_mode
    );
}

/// Test world reset - showing how cleanup is automatic.
#[with_carla_server]
fn test_world_reset_migrated() {
    let world = client.world().expect("Failed to get world");

    // Note: Each test gets a fresh server, so no need for manual reset
    // But we can still test the reset functionality

    // Get all actors
    let actors_before = world.actors().expect("Failed to get actors");
    let count_before = actors_before.len();

    // Destroy all actors except spectator
    for mut actor in actors_before {
        if !actor.type_id().starts_with("spectator") {
            actor.destroy().expect("Failed to destroy actor");
        }
    }

    // After cleanup, should have minimal actors
    let actors_after = world.actors().expect("Failed to get actors after cleanup");

    println!(
        "Actors before: {}, after: {}",
        count_before,
        actors_after.len()
    );
    assert!(actors_after.len() <= count_before);

    // Check that we still have a spectator
    let has_spectator = actors_after
        .iter()
        .any(|actor| actor.type_id().contains("spectator"));
    assert!(has_spectator, "Should have spectator actor");
}

/// Test map loading with retry helpers.
#[with_carla_server]
fn test_map_loading_with_retry() {
    use carla_test_server::helpers::retry_operation;
    use std::time::Duration;

    // Use retry helper for potentially flaky operations
    let world = retry_operation(3, Duration::from_millis(500), || client.world())
        .expect("Failed to get world after retries");

    let map = world.map().expect("Failed to get map");

    // Test basic map properties
    let name = map.name();
    println!("Current map name: {}", name);
    assert!(!name.is_empty());

    // Test spawn points
    let spawn_points = map.spawn_points();
    println!("Map has {} spawn points", spawn_points.len());
    assert!(!spawn_points.is_empty(), "Map should have spawn points");
}

/// Test weather modifications with wait conditions.
#[with_carla_server]
fn test_weather_with_wait_conditions(client: &carla::client::Client) {
    use carla_test_server::helpers::wait_for_condition;
    use std::time::Duration;

    let world = client.world().expect("Failed to get world");

    // Get current weather
    let current_weather = world.weather().expect("Failed to get weather");
    println!("Current weather cloudiness: {}", current_weather.cloudiness);

    // Set new weather
    let new_weather = WeatherParameters {
        cloudiness: 80.0,
        precipitation: 50.0,
        ..Default::default()
    };

    world
        .set_weather(&new_weather)
        .expect("Failed to set weather");

    // Wait for weather to be applied
    wait_for_condition(Duration::from_secs(5), Duration::from_millis(100), || {
        world
            .weather()
            .map(|w| (w.cloudiness - 80.0).abs() < 1.0)
            .unwrap_or(false)
    })
    .expect("Weather change timeout");

    // Verify weather was set
    let updated_weather = world.weather().expect("Failed to get updated weather");
    assert!((updated_weather.cloudiness - 80.0).abs() < 1.0);
    assert!((updated_weather.precipitation - 50.0).abs() < 1.0);
}

/// Test world settings with artifact guard.
#[with_carla_server]
fn test_settings_with_artifacts() {
    use carla_test_server::helpers::TestArtifactGuard;

    // Create artifact guard for debugging
    let _guard = TestArtifactGuard::new("test_settings_with_artifacts");

    let world = client.world().expect("Failed to get world");

    // Get current settings
    let original_settings = world.settings().expect("Failed to get settings");

    println!(
        "Original synchronous mode: {}",
        original_settings.synchronous_mode
    );

    // Modify settings
    let mut new_settings = original_settings.clone();
    new_settings.synchronous_mode = !original_settings.synchronous_mode;

    world
        .apply_settings(&new_settings)
        .expect("Failed to apply settings");

    // Verify settings were applied
    let updated_settings = world.settings().expect("Failed to get updated settings");

    assert_eq!(
        updated_settings.synchronous_mode,
        new_settings.synchronous_mode
    );

    // Note: No need to restore - each test gets fresh server
}

/// Test world tick operations with test context.
#[test]
fn test_world_tick_with_context() {
    use carla_test_server::run_test_with_context;

    run_test_with_context("world_tick_test", |client, context| {
        context.log_milestone("Starting tick test");

        let world = client.world().expect("Failed to get world");

        // Setup phase
        let setup_phase = context.phase("setup_synchronous_mode");
        let original_settings = world.settings().expect("Failed to get settings");

        let mut sync_settings = original_settings.clone();
        sync_settings.synchronous_mode = true;
        sync_settings.fixed_delta_seconds = Some(0.05); // 20 FPS

        world
            .apply_settings(&sync_settings)
            .expect("Failed to apply sync settings");
        setup_phase.complete();

        // Test phase
        let test_phase = context.phase("test_ticking");
        let frame_before = world
            .wait_for_tick(Some(Duration::from_secs(10)))
            .expect("Failed to wait for tick");
        let frame_id_before = frame_before.timestamp.frame;

        world.tick().expect("Failed to tick");

        let frame_after = world
            .wait_for_tick(Some(Duration::from_secs(10)))
            .expect("Failed to wait for tick after");
        let frame_id_after = frame_after.timestamp.frame;

        assert!(
            frame_id_after > frame_id_before,
            "Frame should advance after tick: {} -> {}",
            frame_id_before,
            frame_id_after
        );
        test_phase.complete();

        context.log_milestone("Tick test completed successfully");
    })
    .expect("World tick test failed");
}

/// Comprehensive test showing migration best practices.
#[with_carla_server]
fn test_comprehensive_migration_example(client: &carla::client::Client) {
    use carla_test_server::helpers::{retry_operation, wait_for_condition, TestArtifactGuard};
    use std::time::Duration;

    // 1. Setup debugging support
    let _guard = TestArtifactGuard::new("comprehensive_migration");

    // 2. Get world with retry
    let world = retry_operation(3, Duration::from_millis(500), || client.world())
        .expect("Failed to get world");

    // 3. Wait for world initialization
    wait_for_condition(Duration::from_secs(10), Duration::from_millis(200), || {
        world.actors().map(|a| !a.is_empty()).unwrap_or(false)
    })
    .expect("World initialization timeout");

    // 4. Perform test operations
    let actors = world.actors().expect("Failed to get actors");
    let map = world.map().expect("Failed to get map");
    let blueprints = world.blueprint_library().expect("Failed to get blueprints");

    // 5. Assertions with helpful messages
    assert!(
        !actors.is_empty(),
        "World should have at least spectator actor"
    );
    assert!(!map.name().is_empty(), "Map should have a name");
    assert!(
        !blueprints.is_empty(),
        "Blueprint library should not be empty"
    );

    println!("Migration test completed:");
    println!("  - Actors: {}", actors.len());
    println!("  - Map: {}", map.name());
    println!("  - Blueprints: {}", blueprints.len());
}

/// Test showing how to handle configuration in migrated tests.
#[test]
fn test_custom_configuration() {
    use carla_test_server::test_scenario;

    // Create custom configuration
    let (_context, config) = test_scenario!("custom_config_test", |builder| {
        builder
            .with_port(2001)
            .with_quality("Medium")
            .windowed(false)
            .with_args(vec!["-benchmark".to_string()])
    })
    .expect("Failed to create test scenario");

    println!("Custom test configuration:");
    println!("  - Port: {}", config.server.port);
    println!("  - Quality: {}", config.server.quality_level);
    println!("  - Additional args: {:?}", config.server.additional_args);

    // Note: In a real test, you would use this config with a coordinator
    // to start a server with these specific settings
}

/// Test demonstrating error recovery patterns.
#[with_carla_server]
fn test_error_recovery_patterns() {
    use carla_test_server::helpers::retry_operation;
    use std::time::Duration;

    let world = client.world().expect("Failed to get world");

    // Pattern 1: Retry on transient failures
    let _actors = retry_operation(3, Duration::from_millis(100), || world.actors())
        .expect("Failed to get actors after retries");

    // Pattern 2: Handle expected failures gracefully
    let invalid_actor = world.actor(999999);
    // world.actor returns Result<Option<Actor>>
    match invalid_actor {
        Ok(None) => {} // Expected - no actor with this ID
        Ok(Some(_)) => panic!("Should not find invalid actor"),
        Err(e) => panic!("Unexpected error looking up invalid actor: {:?}", e),
    }

    // Pattern 3: Use Result for recoverable operations
    let blueprint_result = world
        .blueprint_library()
        .and_then(|lib| lib.find("vehicle.invalid.model"));

    match blueprint_result {
        Ok(Some(_)) => panic!("Should not find invalid blueprint"),
        Ok(None) => println!("Correctly returned None for invalid blueprint"),
        Err(e) => println!("Blueprint search error (expected): {:?}", e),
    }
}
