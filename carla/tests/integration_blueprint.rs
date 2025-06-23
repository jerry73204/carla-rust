//! Integration tests for blueprint functionality.

#[cfg(feature = "test-carla-server")]
use carla::{client::Client, error::CarlaResult};

#[cfg(feature = "test-carla-server")]
use serial_test::serial;

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_blueprint_library() -> CarlaResult<()> {
    // Connect to CARLA server
    let client = Client::new("localhost", 2000, None)?;
    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Test library size
    let size = blueprint_library.len();
    println!("Blueprint library contains {} blueprints", size);
    assert!(size > 0);

    // Test finding a specific blueprint
    if let Some(vehicle_bp) = blueprint_library.find("vehicle.dodge.charger")? {
        println!("Found Tesla Model 3 blueprint");
        println!("ID: {}", vehicle_bp.id());
        println!("Tags: {:?}", vehicle_bp.tags());
    }

    // Test finding sensor blueprints
    let sensor_bps = blueprint_library.filter("sensor.*")?;
    println!("Found {} sensor blueprints", sensor_bps.len());
    assert!(!sensor_bps.is_empty());

    Ok(())
}

#[test]
#[cfg(feature = "test-carla-server")]
fn test_blueprint_attributes() -> CarlaResult<()> {
    // Connect to CARLA server
    let client = Client::new("localhost", 2000, None)?;
    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Get a camera blueprint
    if let Some(mut camera_bp) = blueprint_library.find("sensor.camera.rgb")? {
        println!("Found RGB camera blueprint");

        // Test attribute checking
        assert!(camera_bp.contains_attribute("image_size_x"));
        assert!(camera_bp.contains_attribute("image_size_y"));
        assert!(!camera_bp.contains_attribute("non_existent_attribute"));

        // Test attribute setting
        camera_bp.set_attribute("image_size_x", 1920)?;
        camera_bp.set_attribute("image_size_y", 1080)?;
        camera_bp.set_attribute("fov", 110)?;

        println!("Successfully set camera attributes");
    }

    Ok(())
}

#[test]
#[cfg(feature = "test-carla-server")]
fn test_blueprint_spawning() -> CarlaResult<()> {
    use carla::geom::{Location, Rotation, Transform};

    // Connect to CARLA server
    let client = Client::new("localhost", 2000, None)?;
    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Get a vehicle blueprint
    if let Some(vehicle_bp) = blueprint_library.find("vehicle.dodge.charger")? {
        // Define spawn transform
        let spawn_point =
            Transform::new(Location::new(0.0, 0.0, 0.5), Rotation::new(0.0, 0.0, 0.0));

        // Spawn the vehicle
        let vehicle = world.spawn_actor(&vehicle_bp, &spawn_point, None)?;
        println!("Spawned vehicle with ID: {}", vehicle.id());

        // Vehicle will be automatically destroyed when it goes out of scope
    }

    Ok(())
}
