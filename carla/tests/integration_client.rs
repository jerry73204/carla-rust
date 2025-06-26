//! Client integration tests
//!
//! Tests for basic client functionality using carla-test-server infrastructure

use carla::{actor::ActorExt, client::WeatherParameters};
use carla_test_server::with_carla_server;
use std::time::Duration;

// Keep TEST_TIMEOUT for consistency
const TEST_TIMEOUT: Duration = Duration::from_secs(10);

#[with_carla_server]
fn test_client_connection(client: &carla::client::Client) {
    // Verify we can get server version
    let version = client
        .server_version()
        .expect("Failed to get server version");
    println!("Connected to CARLA server version: {}", version);
    assert!(version.starts_with("0.10"));
}

#[with_carla_server]
fn test_client_timeout() {
    // Client is automatically provided with proper timeout
    // Verify connection works
    let version = client
        .server_version()
        .expect("Failed to get server version");
    assert!(!version.is_empty());
}

#[with_carla_server]
fn test_available_maps(client: &carla::client::Client) {
    // Get available maps
    let maps = client
        .available_maps()
        .expect("Failed to get available maps");
    println!("Available maps: {:?}", maps);

    // Should have at least one map
    assert!(!maps.is_empty());

    // Maps should contain some expected CARLA maps
    let map_names: Vec<String> = maps.iter().map(|m| m.to_lowercase()).collect();
    let has_town = map_names.iter().any(|name| name.contains("town"));
    assert!(has_town, "Should have at least one Town map available");
}

#[with_carla_server]
fn test_world_access() {
    // Get current world
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

#[with_carla_server]
fn test_world_reset(client: &carla::client::Client) {
    let world = client.world().expect("Failed to get world");

    // Each test gets a fresh server, but we can still test cleanup
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

    // Should have at least the spectator
    assert!(!actors_after.is_empty());

    // Check that we have a spectator
    let has_spectator = actors_after
        .iter()
        .any(|actor| actor.type_id().contains("spectator"));
    assert!(has_spectator, "Should have spectator actor");
}

#[with_carla_server]
fn test_map_loading() {
    use carla_test_server::helpers::retry_operation;

    // Get current map with retry for stability
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

    // Test first spawn point
    let first_spawn = spawn_points
        .get(0)
        .expect("Should have at least one spawn point");
    println!("First spawn point: {:?}", first_spawn);
}

#[with_carla_server]
fn test_world_weather(client: &carla::client::Client) {
    use carla_test_server::helpers::wait_for_condition;

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

#[with_carla_server]
fn test_world_settings_modification() {
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

#[with_carla_server]
fn test_world_tick(client: &carla::client::Client) {
    use carla_test_server::helpers::TestArtifactGuard;

    // Save artifacts if test fails
    let _guard = TestArtifactGuard::new("test_world_tick");

    let world = client.world().expect("Failed to get world");

    // Get current settings
    let original_settings = world.settings().expect("Failed to get settings");

    // Enable synchronous mode for testing
    let mut sync_settings = original_settings.clone();
    sync_settings.synchronous_mode = true;
    sync_settings.fixed_delta_seconds = Some(0.05); // 20 FPS

    world
        .apply_settings(&sync_settings)
        .expect("Failed to apply sync settings");

    // Test manual tick
    let frame_before = world
        .wait_for_tick(Some(TEST_TIMEOUT))
        .expect("Failed to wait for tick");
    let frame_id_before = frame_before.timestamp.frame;

    world.tick().expect("Failed to tick");

    let frame_after = world
        .wait_for_tick(Some(TEST_TIMEOUT))
        .expect("Failed to wait for tick after");
    let frame_id_after = frame_after.timestamp.frame;

    // Frame should have advanced
    assert!(
        frame_id_after > frame_id_before,
        "Frame should advance after tick: {} -> {}",
        frame_id_before,
        frame_id_after
    );
}
