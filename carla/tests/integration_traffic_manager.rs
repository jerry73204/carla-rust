//! Traffic Manager integration tests
//!
//! Tests for traffic management functionality that require a CARLA server

mod common;

use carla::traffic_manager::TrafficManager;
use carla_test_server::with_carla_server;
use common::spawn_test_vehicle;

#[with_carla_server]
fn test_traffic_manager_connection(client: &carla::client::Client) {
    // Create traffic manager with default port
    let tm = TrafficManager::get_instance(&client, 8000)
        .expect("Failed to get traffic manager instance");

    // Verify basic properties
    let port = tm.port();
    assert!(port > 0);
    assert_eq!(port, 8000); // Default port
}

#[with_carla_server]

fn test_traffic_manager_custom_port(client: &carla::client::Client) {
    // Create traffic manager with custom port
    let custom_port = 8001;
    let tm = TrafficManager::get_instance(&client, custom_port)
        .expect("Failed to get traffic manager instance with custom port");

    // Verify port
    assert_eq!(tm.port(), custom_port);
}

#[with_carla_server]

fn test_traffic_manager_global_settings(client: &carla::client::Client) {
    let tm = TrafficManager::get_instance(&client, 8000)
        .expect("Failed to get traffic manager instance");

    // Test global speed percentage
    tm.set_global_speed_percentage(50.0)
        .expect("Failed to set global speed percentage");

    // Test global distance to leading vehicle
    tm.set_global_distance_to_leading_vehicle(5.0)
        .expect("Failed to set global distance to leading vehicle");

    // Test hybrid physics mode
    tm.set_hybrid_physics_mode(true)
        .expect("Failed to set hybrid physics mode");

    // Test synchronous mode
    tm.set_synchronous_mode(true)
        .expect("Failed to set synchronous mode");

    // Reset to defaults
    tm.set_global_speed_percentage(0.0)
        .expect("Failed to set global speed percentage");
    tm.set_hybrid_physics_mode(false)
        .expect("Failed to set hybrid physics mode");
    tm.set_synchronous_mode(false)
        .expect("Failed to set synchronous mode");
}

#[with_carla_server]

fn test_vehicle_autopilot(client: &carla::client::Client) {
    // Spawn a vehicle
    let vehicle = spawn_test_vehicle(&client).expect("Failed to spawn test vehicle");

    // Enable autopilot
    vehicle
        .set_autopilot(true, None)
        .expect("Failed to set autopilot");

    // TODO: Verify autopilot is enabled
    // This would require additional FFI methods to check autopilot state

    // Disable autopilot
    vehicle
        .set_autopilot(false, None)
        .expect("Failed to set autopilot");
}

#[with_carla_server]

fn test_vehicle_autopilot_with_traffic_manager(client: &carla::client::Client) {
    // Create traffic manager
    let tm_port = 8002;
    let tm = TrafficManager::get_instance(&client, tm_port)
        .expect("Failed to get traffic manager instance");

    // Spawn a vehicle
    let vehicle = spawn_test_vehicle(&client).expect("Failed to spawn test vehicle");
    let _vehicle_id = vehicle.id();
    // TODO: Use vehicle_id for traffic manager operations when FFI is implemented

    // Enable autopilot with specific traffic manager
    vehicle
        .set_autopilot(true, Some(tm_port))
        .expect("Failed to enable autopilot with traffic manager");

    // Configure vehicle behavior
    tm.set_vehicle_speed_percentage(&vehicle, 20.0)
        .expect("Failed to set vehicle speed percentage");
    tm.set_vehicle_distance_to_leading_vehicle(&vehicle, 3.0)
        .expect("Failed to set vehicle distance to leading vehicle");
    tm.set_vehicle_keep_right_percentage(&vehicle, 0.8)
        .expect("Failed to set vehicle keep right percentage");

    // Test ignore lights percentage
    tm.set_vehicle_percentage_running_light(&vehicle, 0.1)
        .expect("Failed to set vehicle percentage running light");

    // Test ignore signs percentage
    tm.set_vehicle_percentage_running_sign(&vehicle, 0.05)
        .expect("Failed to set vehicle percentage running sign");

    // Test ignore walkers percentage
    tm.set_vehicle_percentage_ignore_walkers(&vehicle, 0.0)
        .expect("Failed to set vehicle percentage ignore walkers");

    // Disable autopilot
    vehicle
        .set_autopilot(false, Some(tm_port))
        .expect("Failed to disable autopilot with traffic manager");
}

#[with_carla_server]

fn test_traffic_manager_lane_change(client: &carla::client::Client) {
    let tm = TrafficManager::get_instance(&client, 8000)
        .expect("Failed to get traffic manager instance");
    let vehicle = spawn_test_vehicle(&client).expect("Failed to spawn test vehicle");
    let _vehicle_id = vehicle.id();
    // TODO: Use vehicle_id for traffic manager lane change operations when FFI is implemented

    // Enable autopilot
    vehicle
        .set_autopilot(true, None)
        .expect("Failed to set autopilot");

    // Test auto lane change
    tm.set_vehicle_auto_lane_change(&vehicle, false)
        .expect("Failed to set vehicle auto lane change");

    // Test force lane change
    tm.force_vehicle_lane_change(&vehicle, true)
        .expect("Failed to force vehicle lane change"); // Change to left
    tm.force_vehicle_lane_change(&vehicle, false)
        .expect("Failed to force vehicle lane change"); // Change to right
}

#[with_carla_server]

fn test_traffic_manager_desired_speed(client: &carla::client::Client) {
    let tm = TrafficManager::get_instance(&client, 8000)
        .expect("Failed to get traffic manager instance");
    let vehicle = spawn_test_vehicle(&client).expect("Failed to spawn test vehicle");
    let _vehicle_id = vehicle.id();
    // TODO: Use vehicle_id for traffic manager speed operations when FFI is implemented

    // Enable autopilot
    vehicle
        .set_autopilot(true, None)
        .expect("Failed to set autopilot");

    // Set desired speed (50 km/h)
    tm.set_vehicle_desired_speed(&vehicle, 50.0)
        .expect("Failed to set vehicle desired speed");

    // Change speed
    tm.set_vehicle_desired_speed(&vehicle, 30.0)
        .expect("Failed to set vehicle desired speed");
}

#[with_carla_server]

fn test_traffic_manager_collision_detection(client: &carla::client::Client) {
    let tm = TrafficManager::get_instance(&client, 8000)
        .expect("Failed to get traffic manager instance");

    // Spawn two vehicles
    let vehicle1 = spawn_test_vehicle(&client).expect("Failed to spawn test vehicle");
    let vehicle2 = spawn_test_vehicle(&client).expect("Failed to spawn test vehicle");

    let _vehicle1_id = vehicle1.id();
    let _vehicle2_id = vehicle2.id();
    // TODO: Use vehicle IDs for collision detection configuration when FFI is implemented

    // Enable autopilot for both
    vehicle1
        .set_autopilot(true, None)
        .expect("Failed to enable autopilot for vehicle1");
    vehicle2
        .set_autopilot(true, None)
        .expect("Failed to enable autopilot for vehicle2");

    // Configure collision detection
    // Vehicle1 should ignore collisions with vehicle2
    tm.set_collision_detection(&vehicle1, &vehicle2, false)
        .expect("Failed to set collision detection");

    // Re-enable collision detection
    tm.set_collision_detection(&vehicle1, &vehicle2, true)
        .expect("Failed to set collision detection");
}

#[with_carla_server]

fn test_traffic_manager_boundaries(client: &carla::client::Client) {
    let tm = TrafficManager::get_instance(&client, 8000)
        .expect("Failed to get traffic manager instance");

    // Test respawn dormant vehicles boundaries
    tm.set_respawn_boundaries(10.0, 70.0)
        .expect("Failed to set respawn boundaries");
}

#[with_carla_server]

fn test_traffic_manager_multiple_vehicles(client: &carla::client::Client) {
    let tm = TrafficManager::get_instance(&client, 8000)
        .expect("Failed to get traffic manager instance");
    let world = client.world().expect("Failed to get world");
    let blueprint_library = world
        .blueprint_library()
        .expect("Failed to get blueprint library");

    // Spawn multiple vehicles
    let mut vehicles = Vec::new();
    let vehicle_bp = blueprint_library
        .find("vehicle.dodge.charger")
        .expect("Failed to find blueprint")
        .or_else(|| {
            blueprint_library
                .filter("vehicle.*")
                .ok()
                .and_then(|bps| bps.first().cloned())
        })
        .expect("No vehicle blueprints found");

    // Get spawn points
    let spawn_points = world.map().expect("Failed to get map").spawn_points();

    for i in 0..3.min(spawn_points.len()) {
        if let Some(spawn_point) = spawn_points.get(i) {
            let actor = world
                .try_spawn_actor(&vehicle_bp, &spawn_point, None)
                .expect("Failed to try spawn actor")
                .expect("Failed to spawn vehicle");

            if let Ok(vehicle) = actor.into_vehicle() {
                vehicles.push(vehicle);
            }
        }
    }

    // Enable autopilot for all vehicles with different settings
    for (i, vehicle) in vehicles.iter().enumerate() {
        vehicle
            .set_autopilot(true, None)
            .expect("Failed to set autopilot");

        // Configure each vehicle differently
        let speed_diff = (i as f32) * 10.0;
        tm.set_vehicle_speed_percentage(vehicle, speed_diff)
            .expect("Failed to set vehicle speed percentage");

        let keep_right = 0.9 - (i as f32) * 0.2;
        tm.set_vehicle_keep_right_percentage(vehicle, keep_right)
            .expect("Failed to set vehicle keep right percentage");
    }

    // Set global settings
    tm.set_global_distance_to_leading_vehicle(4.0)
        .expect("Failed to set global distance to leading vehicle");

    // Clean up - vehicles destroyed when dropped
}

#[with_carla_server]

fn test_traffic_manager_shutdown_restart(client: &carla::client::Client) {
    // Create and shut down traffic manager
    let tm = TrafficManager::get_instance(&client, 8003)
        .expect("Failed to get traffic manager instance");
    tm.shutdown().expect("Failed to shutdown traffic manager");

    // Create new one on same port should work
    let _tm2 = TrafficManager::get_instance(&client, 8003)
        .expect("Failed to get traffic manager instance");
}
