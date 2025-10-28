//! Spawn walker example
//!
//! This example demonstrates how to spawn a single walker (pedestrian)
//! in the CARLA simulator.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example spawn_walker
//! ```

use carla::client::{ActorBase, Client};

fn main() {
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    println!("Connected!");

    // Load default map for clean world state
    println!("\nLoading map: Town10HD_Opt...");
    let mut world = client.load_world("Town10HD_Opt");
    println!("Map loaded!");
    let blueprint_library = world.blueprint_library();

    // Find a walker blueprint
    let walker_bp = blueprint_library
        .filter("walker.pedestrian.*")
        .get(0)
        .expect("Failed to find walker blueprint");

    println!("Found blueprint: {}", walker_bp.id());

    // Get spawn points
    let spawn_points = world.map().recommended_spawn_points();
    println!("Available spawn points: {}", spawn_points.len());

    // Use first spawn point
    let spawn_point = spawn_points.get(0).expect("No spawn points available");

    println!("Spawning walker at spawn point 0...");

    // Spawn walker
    let walker = world
        .spawn_actor(&walker_bp, &spawn_point)
        .expect("Failed to spawn walker");

    println!("✓ Walker spawned successfully!");
    println!("  Type: {}", walker.type_id());
    println!("  ID: {}", walker.id());
    println!("  Alive: {}", walker.is_alive());

    // Get location
    let location = walker.location();
    println!(
        "  Location: x={:.2}, y={:.2}, z={:.2}",
        location.x, location.y, location.z
    );

    println!("\nWalker will remain in the simulation.");
    println!("Note: Walker has no AI controller and won't move automatically.");
    println!("Use walker_control example to control walker movement.");
}
