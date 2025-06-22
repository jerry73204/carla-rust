//! Actor management integration tests
//!
//! Tests for actor spawning, management, and destruction that require a CARLA server

mod common;

use carla::{
    actor::ActorExt,
    error::{CarlaError, CarlaResult, DestroyError},
    geom::{Location, Rotation, Transform},
};
use common::{get_test_client, reset_world, spawn_test_vehicle, with_spawned_actor};
use serial_test::serial;

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_blueprint_library_access() -> CarlaResult<()> {
    let client = get_test_client()?;
    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Test library size
    let size = blueprint_library.len();
    println!("Blueprint library contains {} blueprints", size);
    assert!(size > 0);

    // Test finding specific blueprints
    let vehicle_bps = blueprint_library.filter("vehicle.*")?;
    println!("Found {} vehicle blueprints", vehicle_bps.len());
    assert!(!vehicle_bps.is_empty());

    let sensor_bps = blueprint_library.filter("sensor.*")?;
    println!("Found {} sensor blueprints", sensor_bps.len());
    assert!(!sensor_bps.is_empty());

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_blueprint_attributes() -> CarlaResult<()> {
    let client = get_test_client()?;
    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Find a vehicle blueprint
    if let Some(vehicle_bp) = blueprint_library.find("vehicle.tesla.model3")? {
        println!("Tesla Model 3 blueprint found");
        println!("ID: {}", vehicle_bp.id());
        println!("Tags: {:?}", vehicle_bp.tags());

        // Test attributes
        let attributes: Vec<String> = vehicle_bp.attribute_ids().collect();
        println!("Attributes: {:?}", attributes);
        assert!(!attributes.is_empty());

        // Common vehicle attributes - test specific ones
        if vehicle_bp.contains_attribute("role_name") {
            println!("Found role_name attribute");
        }
        if vehicle_bp.contains_attribute("color") {
            println!("Found color attribute");
        }
    } else {
        // Try any vehicle if Tesla is not available
        let vehicle_bps = blueprint_library.filter("vehicle.*")?;
        assert!(
            !vehicle_bps.is_empty(),
            "Should have at least one vehicle blueprint"
        );

        let vehicle_bp = &vehicle_bps[0];
        println!("Using vehicle blueprint: {}", vehicle_bp.id());

        let attributes: Vec<String> = vehicle_bp.attribute_ids().collect();
        println!("Blueprint attributes: {:?}", attributes);
        assert!(!attributes.is_empty());
    }

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_vehicle_spawning() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Find a vehicle blueprint
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")?
        .or_else(|| blueprint_library.filter("vehicle.*").ok()?.first().cloned())
        .expect("No vehicle blueprints found");

    println!("Spawning vehicle: {}", vehicle_bp.id());

    // Get a spawn point
    let spawn_points = world.map()?.spawn_points();
    let spawn_point = spawn_points
        .get(0)
        .unwrap_or_else(|| Transform::new(Location::new(0.0, 0.0, 1.0), Rotation::default()));

    // Spawn the vehicle
    let actor = world
        .try_spawn_actor(&vehicle_bp, &spawn_point, None)?
        .expect("Failed to spawn vehicle");

    // Convert to vehicle
    let vehicle = match actor.into_vehicle() {
        Ok(vehicle) => vehicle,
        Err(_) => panic!("Expected vehicle actor, got something else"),
    };

    println!("Successfully spawned vehicle with ID: {}", vehicle.id());

    // Test vehicle properties
    let vehicle_id = vehicle.id();
    assert!(vehicle_id > 0);

    let type_id = vehicle.type_id();
    println!("Vehicle type: {}", type_id);
    assert!(type_id.contains("vehicle"));

    // Test transform
    let transform = vehicle.transform();
    println!("Vehicle transform: {:?}", transform);

    // Clean up - vehicle will be destroyed automatically when dropped
    drop(vehicle);
    println!("Vehicle destroyed");

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_actor_lifecycle() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;

    // Count actors before spawning
    let actors_before = world.actors()?.len();
    println!("Actors before spawning: {}", actors_before);

    // Spawn a test vehicle
    let vehicle = spawn_test_vehicle(&client)?;
    let vehicle_id = vehicle.id();

    // Count actors after spawning
    let actors_after = world.actors()?.len();
    println!("Actors after spawning: {}", actors_after);
    assert_eq!(actors_after, actors_before + 1);

    // Verify we can find the actor
    let found_actor = world.actor(vehicle_id)?;
    assert!(found_actor.is_some());

    // Destroy the actor - automatically destroyed when dropped

    // Verify it's gone
    let actors_final = world.actors()?.len();
    println!("Actors after destruction: {}", actors_final);
    assert_eq!(actors_final, actors_before);

    // Should not be able to find it anymore
    let not_found = world.actor(vehicle_id)?;
    assert!(not_found.is_none());

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_actor_transform_updates() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    // Spawn a test vehicle
    let vehicle = spawn_test_vehicle(&client)?;

    // Get initial transform
    let initial_transform = vehicle.transform();
    println!("Initial transform: {:?}", initial_transform);

    // Set new transform
    let new_transform = Transform::new(
        Location::new(10.0, 20.0, 1.0),
        Rotation::new(0.0, 90.0, 0.0),
    );

    vehicle.set_transform(&new_transform)?;

    // Verify transform was updated
    let updated_transform = vehicle.transform();
    println!("Updated transform: {:?}", updated_transform);

    // Check location (with some tolerance for floating point)
    let loc_diff = (updated_transform.location.x - 10.0).abs()
        + (updated_transform.location.y - 20.0).abs()
        + (updated_transform.location.z - 1.0).abs();
    assert!(loc_diff < 0.1, "Location should be close to target");

    // Check rotation (with tolerance)
    let yaw_diff = (updated_transform.rotation.yaw - 90.0).abs();
    assert!(yaw_diff < 1.0, "Yaw should be close to 90 degrees");

    // Clean up - vehicle automatically destroyed when dropped

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_vehicle_attributes() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    // Spawn a test vehicle
    let vehicle = spawn_test_vehicle(&client)?;

    // Test basic actor properties
    let actor_id = vehicle.id();
    let type_id = vehicle.type_id();

    println!("Vehicle ID: {}", actor_id);
    println!("Type ID: {}", type_id);

    assert!(actor_id > 0);
    assert!(type_id.contains("vehicle"));

    // Test vehicle-specific properties if available
    // Note: Some properties might require FFI implementation

    // Clean up - vehicle automatically destroyed when dropped

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_multiple_actors() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let initial_count = world.actors()?.len();

    // Spawn multiple vehicles
    let mut vehicles = Vec::new();
    for i in 0..3 {
        let vehicle = spawn_test_vehicle(&client)?;
        println!("Spawned vehicle {} with ID: {}", i, vehicle.id());
        vehicles.push(vehicle);
    }

    // Verify all actors exist
    let current_count = world.actors()?.len();
    assert_eq!(current_count, initial_count + 3);

    // Verify each vehicle can be found
    for vehicle in &vehicles {
        let found = world.actor(vehicle.id())?;
        assert!(found.is_some());
    }

    // Clean up all vehicles - they will be destroyed automatically when dropped
    for (i, vehicle) in vehicles.into_iter().enumerate() {
        drop(vehicle);
        println!("Destroyed vehicle {}", i);
    }

    // Verify count is back to original
    let final_count = world.actors()?.len();
    assert_eq!(final_count, initial_count);

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_actor_filtering() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;

    // Spawn a test vehicle
    let vehicle = spawn_test_vehicle(&client)?;
    let vehicle_type = vehicle.type_id();

    // Get all actors
    let all_actors = world.actors()?;
    println!("Total actors: {}", all_actors.len());

    // Find vehicles
    let vehicles: Vec<_> = all_actors
        .iter()
        .filter(|actor| actor.type_id().contains("vehicle"))
        .collect();
    println!("Vehicle actors: {}", vehicles.len());
    assert!(vehicles.len() >= 1);

    // Find our specific vehicle
    let our_vehicle = vehicles.iter().find(|actor| actor.id() == vehicle.id());
    assert!(our_vehicle.is_some());

    println!("Found our vehicle with type: {}", vehicle_type);

    // Clean up - vehicle automatically destroyed when dropped

    Ok(())
}

// ============================================================================
// Actor Destruction Tests
// ============================================================================

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_explicit_actor_destruction() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;

    // Spawn an actor
    let blueprint_library = world.blueprint_library()?;
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")?
        .or_else(|| blueprint_library.filter("vehicle.*").ok()?.first().cloned())
        .expect("No vehicle blueprints found");

    let transform = Transform::default();
    let mut actor = world.spawn_actor(&vehicle_bp, &transform, None)?;

    // Verify actor is alive
    assert!(actor.is_alive());
    let actor_id = actor.id();

    // Explicitly destroy
    actor.destroy().expect("Failed to destroy actor");

    // Verify actor is gone from world
    let found = world.actor(actor_id)?;
    assert!(
        found.is_none(),
        "Actor should not be found after destruction"
    );

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_vehicle_destruction() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;

    // Create vehicle
    let blueprint_library = world.blueprint_library()?;
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")?
        .or_else(|| blueprint_library.filter("vehicle.*").ok()?.first().cloned())
        .expect("No vehicle blueprints found");

    let mut vehicle = world
        .spawn_actor(&vehicle_bp, &Transform::default(), None)?
        .into_vehicle()
        .expect("Failed to cast to vehicle");

    let vehicle_id = vehicle.id();

    // Vehicle-specific operations
    use carla::actor::VehicleControl;
    vehicle.apply_control(&VehicleControl::default()).unwrap();

    // Destroy via ActorExt trait
    vehicle.destroy().expect("Failed to destroy vehicle");

    // Verify it's gone
    let found = world.actor(vehicle_id)?;
    assert!(
        found.is_none(),
        "Vehicle should not be found after destruction"
    );

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_sensor_destruction_while_listening() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;

    // Create camera sensor
    let blueprint_library = world.blueprint_library()?;
    let sensor_bp = blueprint_library
        .find("sensor.camera.rgb")?
        .ok_or_else(|| {
            CarlaError::World(carla::error::WorldError::BlueprintNotFound(
                "sensor.camera.rgb".to_string(),
            ))
        })?;

    let mut sensor = world
        .spawn_actor(&sensor_bp, &Transform::default(), None)?
        .into_sensor()
        .expect("Failed to cast to sensor");

    let sensor_id = sensor.id();

    // Start listening
    sensor
        .listen(|_data| {
            // Process sensor data
        })
        .unwrap();

    assert!(sensor.is_listening());

    // Destroy while listening (should stop listening first)
    sensor.destroy().expect("Failed to destroy sensor");

    // Verify it's gone
    let found = world.actor(sensor_id)?;
    assert!(
        found.is_none(),
        "Sensor should not be found after destruction"
    );

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_double_destruction_error() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")?
        .or_else(|| blueprint_library.filter("vehicle.*").ok()?.first().cloned())
        .expect("No vehicle blueprints found");

    let mut actor = world.spawn_actor(&vehicle_bp, &Transform::default(), None)?;

    // First destruction should succeed
    actor.destroy().expect("First destruction should succeed");

    // Second destruction should return an error
    match actor.destroy() {
        Err(CarlaError::Destroy(DestroyError::InvalidActor { .. })) => {
            // Expected error
        }
        Ok(_) => panic!("Second destruction should fail"),
        Err(e) => panic!("Unexpected error: {:?}", e),
    }

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_actor_destruction_with_helper() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;

    // Use helper to ensure cleanup
    let result = with_spawned_actor(
        &world,
        "vehicle.tesla.model3",
        &Transform::default(),
        |actor| {
            // Test operations
            assert!(actor.is_alive());
            let actor_id = actor.id();

            // Destroy explicitly
            actor.destroy()?;

            // Verify can't find it
            let found = world.actor(actor_id)?;
            assert!(found.is_none());

            Ok(())
        },
    );

    result
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_mass_actor_destruction() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")?
        .or_else(|| blueprint_library.filter("vehicle.*").ok()?.first().cloned())
        .expect("No vehicle blueprints found");

    let initial_count = world.actors()?.len();

    // Spawn multiple actors
    let mut actors = Vec::new();
    for i in 0..5 {
        let actor = world.spawn_actor(&vehicle_bp, &Transform::default(), None)?;
        println!("Spawned actor {} with ID: {}", i, actor.id());
        actors.push(actor);
    }

    // Verify all spawned
    assert_eq!(world.actors()?.len(), initial_count + 5);

    // Destroy all explicitly
    for (i, mut actor) in actors.into_iter().enumerate() {
        actor.destroy().expect("Failed to destroy actor");
        println!("Destroyed actor {}", i);
    }

    // Verify all destroyed
    assert_eq!(world.actors()?.len(), initial_count);

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_traffic_sign_destruction() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Try to find a traffic sign blueprint
    if let Some(sign_bp) = blueprint_library.find("static.prop.trafficcone01")? {
        let mut sign = world.spawn_actor(&sign_bp, &Transform::default(), None)?;
        let sign_id = sign.id();

        // Destroy traffic sign
        sign.destroy().expect("Failed to destroy traffic sign");

        // Verify it's gone
        let found = world.actor(sign_id)?;
        assert!(found.is_none());
    } else {
        println!("No traffic sign blueprint found, skipping test");
    }

    Ok(())
}
