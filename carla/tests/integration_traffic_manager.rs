//! Traffic Manager integration tests
//!
//! Tests for traffic management functionality that require a CARLA server

mod common;

use carla::{
    actor::{ActorExt, Vehicle},
    error::CarlaResult,
    geom::Transform,
    traffic_manager::{TrafficManager, TrafficManagerConfig},
};
use common::{get_test_client, reset_world, spawn_test_vehicle};
use serial_test::serial;

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_traffic_manager_connection() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    // Create traffic manager with default port
    let tm = TrafficManager::get_instance(&client, 8000)?;

    // Verify basic properties
    let port = tm.port();
    assert!(port > 0);
    assert_eq!(port, 8000); // Default port

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_traffic_manager_custom_port() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    // Create traffic manager with custom port
    let custom_port = 8001;
    let tm = TrafficManager::get_instance(&client, custom_port)?;

    // Verify port
    assert_eq!(tm.port(), custom_port);

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_traffic_manager_global_settings() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let tm = TrafficManager::get_instance(&client, 8000)?;

    // Test global speed percentage
    tm.set_global_speed_percentage(50.0)?;

    // Test global distance to leading vehicle
    tm.set_global_distance_to_leading_vehicle(5.0)?;

    // Test hybrid physics mode
    tm.set_hybrid_physics_mode(true)?;

    // Test synchronous mode
    tm.set_synchronous_mode(true)?;

    // Reset to defaults
    tm.set_global_speed_percentage(0.0)?;
    tm.set_hybrid_physics_mode(false)?;
    tm.set_synchronous_mode(false)?;

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_vehicle_autopilot() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    // Spawn a vehicle
    let vehicle = spawn_test_vehicle(&client)?;

    // Enable autopilot
    vehicle.set_autopilot(true, None)?;

    // TODO: Verify autopilot is enabled
    // This would require additional FFI methods to check autopilot state

    // Disable autopilot
    vehicle.set_autopilot(false, None)?;

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_vehicle_autopilot_with_traffic_manager() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    // Create traffic manager
    let tm_port = 8002;
    let tm = TrafficManager::get_instance(&client, tm_port)?;

    // Spawn a vehicle
    let vehicle = spawn_test_vehicle(&client)?;
    let vehicle_id = vehicle.id();

    // Enable autopilot with specific traffic manager
    vehicle.set_autopilot(true, Some(tm_port))?;

    // Configure vehicle behavior
    tm.set_vehicle_speed_percentage(&vehicle, 20.0)?;
    tm.set_vehicle_distance_to_leading_vehicle(&vehicle, 3.0)?;
    tm.set_vehicle_keep_right_percentage(&vehicle, 0.8)?;

    // Test ignore lights percentage
    tm.set_vehicle_percentage_running_light(&vehicle, 0.1)?;

    // Test ignore signs percentage
    tm.set_vehicle_percentage_running_sign(&vehicle, 0.05)?;

    // Test ignore walkers percentage
    tm.set_vehicle_percentage_ignore_walkers(&vehicle, 0.0)?;

    // Disable autopilot
    vehicle.set_autopilot(false, Some(tm_port))?;

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_traffic_manager_lane_change() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let tm = TrafficManager::get_instance(&client, 8000)?;
    let vehicle = spawn_test_vehicle(&client)?;
    let vehicle_id = vehicle.id();

    // Enable autopilot
    vehicle.set_autopilot(true, None)?;

    // Test auto lane change
    tm.set_vehicle_auto_lane_change(&vehicle, false)?;

    // Test force lane change
    tm.force_vehicle_lane_change(&vehicle, true)?; // Change to left
    tm.force_vehicle_lane_change(&vehicle, false)?; // Change to right

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_traffic_manager_desired_speed() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let tm = TrafficManager::get_instance(&client, 8000)?;
    let vehicle = spawn_test_vehicle(&client)?;
    let vehicle_id = vehicle.id();

    // Enable autopilot
    vehicle.set_autopilot(true, None)?;

    // Set desired speed (50 km/h)
    tm.set_vehicle_desired_speed(&vehicle, 50.0)?;

    // Change speed
    tm.set_vehicle_desired_speed(&vehicle, 30.0)?;

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_traffic_manager_collision_detection() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let tm = TrafficManager::get_instance(&client, 8000)?;

    // Spawn two vehicles
    let vehicle1 = spawn_test_vehicle(&client)?;
    let vehicle2 = spawn_test_vehicle(&client)?;

    let vehicle1_id = vehicle1.id();
    let vehicle2_id = vehicle2.id();

    // Enable autopilot for both
    vehicle1.set_autopilot(true, None)?;
    vehicle2.set_autopilot(true, None)?;

    // Configure collision detection
    // Vehicle1 should ignore collisions with vehicle2
    tm.set_collision_detection(&vehicle1, &vehicle2, false)?;

    // Re-enable collision detection
    tm.set_collision_detection(&vehicle1, &vehicle2, true)?;

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_traffic_manager_boundaries() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let tm = TrafficManager::get_instance(&client, 8000)?;

    // Test respawn dormant vehicles boundaries
    tm.set_respawn_boundaries(10.0, 70.0)?;

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_traffic_manager_multiple_vehicles() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let tm = TrafficManager::get_instance(&client, 8000)?;
    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Spawn multiple vehicles
    let mut vehicles = Vec::new();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")?
        .or_else(|| blueprint_library.filter("vehicle.*").ok()?.first().cloned())
        .expect("No vehicle blueprints found");

    // Get spawn points
    let spawn_points = world.map()?.spawn_points();

    for i in 0..3.min(spawn_points.len()) {
        if let Some(spawn_point) = spawn_points.get(i) {
            let actor = world
                .try_spawn_actor(&vehicle_bp, &spawn_point, None)?
                .expect("Failed to spawn vehicle");

            if let Ok(vehicle) = actor.into_vehicle() {
                vehicles.push(vehicle);
            }
        }
    }

    // Enable autopilot for all vehicles with different settings
    for (i, vehicle) in vehicles.iter().enumerate() {
        vehicle.set_autopilot(true, None)?;

        // Configure each vehicle differently
        let speed_diff = (i as f32) * 10.0;
        tm.set_vehicle_speed_percentage(vehicle, speed_diff)?;

        let keep_right = 0.9 - (i as f32) * 0.2;
        tm.set_vehicle_keep_right_percentage(vehicle, keep_right)?;
    }

    // Set global settings
    tm.set_global_distance_to_leading_vehicle(4.0)?;

    // Clean up - vehicles destroyed when dropped

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_traffic_manager_shutdown_restart() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    // Create and shut down traffic manager
    let tm = TrafficManager::get_instance(&client, 8003)?;
    tm.shutdown()?;

    // Create new one on same port should work
    let _tm2 = TrafficManager::get_instance(&client, 8003)?;

    Ok(())
}
