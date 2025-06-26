//! Integration tests for blueprint functionality.

use carla_test_server::with_carla_server;

#[with_carla_server]
fn test_blueprint_library(client: &carla::client::Client) {
    let world = client.world().expect("Failed to get world");
    let blueprint_library = world
        .blueprint_library()
        .expect("Failed to get blueprint library");

    // Test library size
    let size = blueprint_library.len();
    println!("Blueprint library contains {} blueprints", size);
    assert!(size > 0);

    // Test finding a specific blueprint
    if let Some(vehicle_bp) = blueprint_library
        .find("vehicle.dodge.charger")
        .expect("Failed to find blueprint")
    {
        println!("Found Dodge Charger blueprint");
        println!("ID: {}", vehicle_bp.id());
        println!("Tags: {:?}", vehicle_bp.tags());
    }

    // Test finding sensor blueprints
    let sensor_bps = blueprint_library
        .filter("sensor.*")
        .expect("Failed to filter blueprints");
    println!("Found {} sensor blueprints", sensor_bps.len());
    assert!(!sensor_bps.is_empty());
}

#[with_carla_server]
fn test_blueprint_attributes(client: &carla::client::Client) {
    let world = client.world().expect("Failed to get world");
    let blueprint_library = world
        .blueprint_library()
        .expect("Failed to get blueprint library");

    // Get a camera blueprint
    if let Some(mut camera_bp) = blueprint_library
        .find("sensor.camera.rgb")
        .expect("Failed to find camera blueprint")
    {
        println!("Found RGB camera blueprint");

        // Test attribute checking
        assert!(camera_bp.contains_attribute("image_size_x"));
        assert!(camera_bp.contains_attribute("image_size_y"));
        assert!(!camera_bp.contains_attribute("non_existent_attribute"));

        // Test attribute setting
        camera_bp
            .set_attribute("image_size_x", 1920)
            .expect("Failed to set image_size_x");
        camera_bp
            .set_attribute("image_size_y", 1080)
            .expect("Failed to set image_size_y");
        camera_bp
            .set_attribute("fov", 110)
            .expect("Failed to set fov");

        println!("Successfully set camera attributes");
    }
}

#[with_carla_server]
fn test_blueprint_spawning(client: &carla::client::Client) {
    use carla::geom::{Location, Rotation, Transform};

    let world = client.world().expect("Failed to get world");
    let blueprint_library = world
        .blueprint_library()
        .expect("Failed to get blueprint library");

    // Get a vehicle blueprint
    if let Some(vehicle_bp) = blueprint_library
        .find("vehicle.dodge.charger")
        .expect("Failed to find vehicle blueprint")
    {
        // Define spawn transform
        let spawn_point =
            Transform::new(Location::new(0.0, 0.0, 0.5), Rotation::new(0.0, 0.0, 0.0));

        // Spawn the vehicle
        let vehicle = world
            .spawn_actor(&vehicle_bp, &spawn_point, None)
            .expect("Failed to spawn vehicle");
        println!("Spawned vehicle with ID: {}", vehicle.id());

        // Vehicle will be automatically destroyed when it goes out of scope
    }
}
