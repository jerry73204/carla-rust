//! World information example
//!
//! This example shows how to retrieve information about the current world,
//! including the map name, spawn points, and actors.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example world_info
//! ```

use carla::client::{ActorBase, Client};

fn main() {
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    println!("Connected!");

    // Load default map for clean world state
    println!("\nLoading map: Town10HD_Opt...");
    let world = client.load_world("Town10HD_Opt");
    println!("Map loaded!");

    // Get map information
    let map = world.map();
    let map_name = map.name();
    println!("\nCurrent map: {}", map_name);

    // Get spawn points
    let spawn_points = map.recommended_spawn_points();
    println!("Available spawn points: {}", spawn_points.len());

    // Display first few spawn points
    println!("\nFirst 3 spawn points:");
    for (i, point) in spawn_points.iter().take(3).enumerate() {
        let loc = &point.location;
        println!("  {}: x={:.2}, y={:.2}, z={:.2}", i, loc.x, loc.y, loc.z);
    }

    // Get all actors in the world
    let actors = world.actors();
    println!("\nActors in world: {}", actors.len());

    // List actors by type
    let mut actor_types: std::collections::HashMap<String, usize> =
        std::collections::HashMap::new();
    for actor in actors.iter() {
        let type_id = actor.type_id();
        let category = type_id.split('.').next().unwrap_or("unknown").to_string();
        *actor_types.entry(category).or_insert(0) += 1;
    }

    println!("\nActors by category:");
    for (category, count) in actor_types.iter() {
        println!("  {}: {}", category, count);
    }
}
