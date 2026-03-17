mod common;

use carla::prelude::*;
use common::{client, world};

#[test]
fn t01_connect_and_version() {
    let c = client();
    let server_version = c.server_version().expect("Failed to get server version");
    let client_version = c.client_version().expect("Failed to get client version");

    assert!(
        !server_version.is_empty(),
        "Server version must not be empty"
    );
    assert!(
        !client_version.is_empty(),
        "Client version must not be empty"
    );

    println!("Server version: {server_version}");
    println!("Client version: {client_version}");
}

#[test]
fn t02_world_id_and_map() {
    let w = world();
    let id = w.id().expect("Failed to get world id");
    println!("World episode id: {id}");

    let map = w.map().expect("Failed to get map");
    let map_name = map.name();
    assert!(!map_name.is_empty(), "Map name must not be empty");
    println!("Map name: {map_name}");
}

#[test]
fn t03_spawn_points() {
    let w = world();
    let map = w.map().expect("Failed to get map");
    let spawn_points = map
        .recommended_spawn_points()
        .expect("Failed to get spawn points");
    assert!(
        !spawn_points.is_empty(),
        "Recommended spawn points must not be empty"
    );
    println!("Number of spawn points: {}", spawn_points.len());
}

#[test]
fn t04_blueprint_library() {
    let w = world();
    let library = w
        .blueprint_library()
        .expect("Failed to get blueprint library");

    assert!(!library.is_empty(), "Blueprint library must not be empty");
    println!("Total blueprints: {}", library.len());

    // Filter for vehicles
    let vehicles = library
        .filter("vehicle.*")
        .expect("Failed to filter vehicles");
    assert!(!vehicles.is_empty(), "Must have vehicle blueprints");
    println!("Vehicle blueprints: {}", vehicles.len());

    // Find a specific blueprint
    let tesla = library
        .find("vehicle.tesla.model3")
        .expect("Failed to find blueprint");
    // tesla may be None depending on CARLA version, so we just test find() works
    if let Some(bp) = tesla {
        println!("Found: {}", bp.id());
    }

    // Iterate
    let count = library.iter().count();
    assert_eq!(count, library.len(), "Iterator count must match len()");
}

#[test]
fn t05_spawn_and_destroy_vehicle() {
    let mut w = world();
    let library = w
        .blueprint_library()
        .expect("Failed to get blueprint library");

    let vehicles = library
        .filter("vehicle.*")
        .expect("Failed to filter vehicles");
    let vehicle_bp = vehicles
        .get(0)
        .expect("Failed to get blueprint")
        .expect("No vehicle blueprint at index 0");

    let map = w.map().expect("Failed to get map");
    let spawn_points = map
        .recommended_spawn_points()
        .expect("Failed to get spawn points");
    let spawn_point = spawn_points.get(0).expect("No spawn point at index 0");

    let actor = w
        .spawn_actor(&vehicle_bp, spawn_point)
        .expect("Failed to spawn vehicle");

    let actor_id = actor.id();
    println!("Spawned vehicle with id: {actor_id}");

    assert!(actor_id > 0, "Actor id must be positive");

    actor.destroy().expect("Failed to destroy vehicle");
    println!("Destroyed vehicle {actor_id}");
}
