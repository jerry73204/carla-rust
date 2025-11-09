//! Spawn vehicle example
//!
//! This example demonstrates how to spawn a single vehicle
//! in the CARLA simulator.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example spawn_vehicle
//! ```

use carla::client::{ActorBase, Client};

fn main() {
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    println!("Connected!");

    // Get the current world
    println!("\nGetting current world...");
    let mut world = client.world();
    println!("World ready!");

    // Get blueprint library
    let blueprint_library = world.blueprint_library();

    // Find Tesla Model 3 blueprint
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .expect("Tesla Model 3 not found");

    println!("Found blueprint: {}", vehicle_bp.id());

    // Get spawn points
    let spawn_points = world.map().recommended_spawn_points();
    println!("Available spawn points: {}", spawn_points.len());

    // Use first spawn point
    let spawn_point = spawn_points.get(0).expect("No spawn points available");

    println!("Spawning vehicle at spawn point 0...");

    // Spawn vehicle
    let vehicle = world
        .spawn_actor(&vehicle_bp, spawn_point)
        .expect("Failed to spawn vehicle");

    println!("✓ Vehicle spawned successfully!");
    println!("  Type: {}", vehicle.type_id());
    println!("  ID: {}", vehicle.id());
    println!("  Alive: {}", vehicle.is_alive());

    // Get location
    let location = vehicle.location();
    println!(
        "  Location: x={:.2}, y={:.2}, z={:.2}",
        location.x, location.y, location.z
    );

    println!("\nVehicle will remain in the simulation.");
    println!("Restart CARLA to clean up spawned actors.");
}
