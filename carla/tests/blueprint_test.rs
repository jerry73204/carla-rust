//! Integration tests for blueprint functionality.

use carla::{client::Client, error::CarlaResult, traits::ActorT};

#[test]
#[ignore] // Requires CARLA server running
fn test_blueprint_library() -> CarlaResult<()> {
    // Connect to CARLA server
    let client = Client::new("localhost", 2000, None)?;
    let world = client.get_world()?;
    let blueprint_library = world.get_blueprint_library()?;

    // Test library size
    let size = blueprint_library.len();
    println!("Blueprint library contains {} blueprints", size);
    assert!(size > 0);

    // Test finding a specific blueprint
    if let Some(vehicle_bp) = blueprint_library.find("vehicle.tesla.model3")? {
        println!("Found Tesla Model 3 blueprint");
        println!("ID: {}", vehicle_bp.get_id());
        println!("Tags: {:?}", vehicle_bp.get_tags());
    }

    // TODO: Rewrite the code due to API change
    // Test finding sensor blueprints
    // let sensor_bps = blueprint_library.get_sensors()?;
    // println!("Found {} sensor blueprints", sensor_bps.len());

    Ok(())
}

#[test]
#[ignore] // Requires CARLA server running
fn test_blueprint_attributes() -> CarlaResult<()> {
    // Connect to CARLA server
    let client = Client::new("localhost", 2000, None)?;
    let world = client.get_world()?;
    let blueprint_library = world.get_blueprint_library()?;

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
#[ignore] // Requires CARLA server running
fn test_blueprint_spawning() -> CarlaResult<()> {
    use carla::geom::{Location, Rotation, Transform};

    // Connect to CARLA server
    let client = Client::new("localhost", 2000, None)?;
    let world = client.get_world()?;
    let blueprint_library = world.get_blueprint_library()?;

    // Get a vehicle blueprint
    if let Some(vehicle_bp) = blueprint_library.find("vehicle.tesla.model3")? {
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
