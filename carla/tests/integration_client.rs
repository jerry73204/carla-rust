//! Client integration tests
//!
//! Tests for basic client functionality that require a CARLA server

mod common;

use carla::{actor::ActorExt, client::Client, error::CarlaResult};
use common::{get_test_client, reset_world, TEST_TIMEOUT};
use serial_test::serial;

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_client_connection() -> CarlaResult<()> {
    // Test basic client connection
    let client = get_test_client()?;

    // Verify we can get server version
    let version = client.server_version()?;
    println!("Connected to CARLA server version: {}", version);
    assert!(version.starts_with("0.10"));

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_client_timeout() -> CarlaResult<()> {
    // Test client with custom timeout
    let client = Client::new("localhost", 2000, None::<usize>)?;

    // Should still be able to connect and get version
    let version = client.server_version()?;
    assert!(!version.is_empty());

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_available_maps() -> CarlaResult<()> {
    let client = get_test_client()?;

    // Get available maps
    let maps = client.available_maps()?;
    println!("Available maps: {:?}", maps);

    // Should have at least one map
    assert!(!maps.is_empty());

    // Maps should contain some expected CARLA maps
    let map_names: Vec<String> = maps.iter().map(|m| m.to_lowercase()).collect();
    let has_town = map_names.iter().any(|name| name.contains("town"));
    assert!(has_town, "Should have at least one Town map available");

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_world_access() -> CarlaResult<()> {
    let client = get_test_client()?;

    // Get current world
    let world = client.world()?;

    // Get world ID
    let world_id = world.id();
    println!("Current world ID: {}", world_id);
    assert!(world_id > 0);

    // Get world settings
    let settings = world.settings()?;
    println!(
        "World settings: synchronous_mode={}, no_rendering_mode={}",
        settings.synchronous_mode, settings.no_rendering_mode
    );

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_world_reset() -> CarlaResult<()> {
    let client = get_test_client()?;

    // Reset world to clean state
    reset_world(&client)?;

    let world = client.world()?;

    // After reset, should have minimal actors (just spectator)
    let actors = world.actors()?;
    println!("Actors after reset: {}", actors.len());

    // Should have at least the spectator
    assert!(actors.len() >= 1);

    // Check that we have a spectator
    let has_spectator = actors
        .iter()
        .any(|actor| actor.type_id().contains("spectator"));
    assert!(has_spectator, "Should have spectator actor after reset");

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_map_loading() -> CarlaResult<()> {
    let client = get_test_client()?;

    // Get current map
    let world = client.world()?;
    let map = world.map()?;

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

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_world_weather() -> CarlaResult<()> {
    let client = get_test_client()?;
    let world = client.world()?;

    // Get current weather
    let current_weather = world.weather()?;
    println!("Current weather cloudiness: {}", current_weather.cloudiness);

    // Set new weather
    use carla::client::WeatherParameters;
    let mut new_weather = WeatherParameters::default();
    new_weather.cloudiness = 80.0;
    new_weather.precipitation = 50.0;

    world.set_weather(&new_weather)?;

    // Verify weather was set
    let updated_weather = world.weather()?;
    assert!((updated_weather.cloudiness - 80.0).abs() < 1.0);
    assert!((updated_weather.precipitation - 50.0).abs() < 1.0);

    // Reset to default
    world.set_weather(&WeatherParameters::default())?;

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_world_settings_modification() -> CarlaResult<()> {
    let client = get_test_client()?;
    let world = client.world()?;

    // Get current settings
    let original_settings = world.settings()?;
    println!(
        "Original synchronous mode: {}",
        original_settings.synchronous_mode
    );

    // Modify settings
    use carla::client::WorldSettings;
    let mut new_settings = original_settings.clone();
    new_settings.synchronous_mode = !original_settings.synchronous_mode;

    world.apply_settings(&new_settings)?;

    // Verify settings were applied
    let updated_settings = world.settings()?;
    assert_eq!(
        updated_settings.synchronous_mode,
        new_settings.synchronous_mode
    );

    // Restore original settings
    world.apply_settings(&original_settings)?;

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_world_tick() -> CarlaResult<()> {
    let client = get_test_client()?;
    let world = client.world()?;

    // Get current settings
    let original_settings = world.settings()?;

    // Enable synchronous mode for testing
    use carla::client::WorldSettings;
    let mut sync_settings = original_settings.clone();
    sync_settings.synchronous_mode = true;
    sync_settings.fixed_delta_seconds = Some(0.05); // 20 FPS

    world.apply_settings(&sync_settings)?;

    // Test manual tick
    let frame_before = world.wait_for_tick(Some(TEST_TIMEOUT))?;
    let frame_id_before = frame_before.timestamp.frame;

    world.tick()?;

    let frame_after = world.wait_for_tick(Some(TEST_TIMEOUT))?;
    let frame_id_after = frame_after.timestamp.frame;

    // Frame should have advanced
    assert!(
        frame_id_after > frame_id_before,
        "Frame should advance after tick: {} -> {}",
        frame_id_before,
        frame_id_after
    );

    // Restore original settings
    world.apply_settings(&original_settings)?;

    Ok(())
}
