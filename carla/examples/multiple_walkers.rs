//! Multiple walkers example
//!
//! This example demonstrates how to spawn multiple walkers
//! at different spawn points.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example multiple_walkers
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
    let blueprint_library = world.blueprint_library();

    // Get all walker blueprints
    let walker_blueprints = blueprint_library.filter("walker.pedestrian.*");
    println!("Found {} walker blueprints", walker_blueprints.len());

    // Use first walker blueprint
    let walker_bp = walker_blueprints
        .get(0)
        .expect("No walker blueprints available");

    // Get spawn points
    let spawn_points = world.map().recommended_spawn_points();
    println!("Available spawn points: {}", spawn_points.len());

    // Spawn 3 walkers (or fewer if not enough spawn points)
    let spawn_count = 3.min(spawn_points.len());
    println!("\nSpawning {} walkers...", spawn_count);

    let mut walkers = Vec::new();

    for i in 0..spawn_count {
        let spawn_point = spawn_points.get(i).unwrap();

        match world.spawn_actor(&walker_bp, spawn_point) {
            Ok(walker) => {
                println!("  ✓ Walker {} spawned (ID: {})", i + 1, walker.id());
                walkers.push(walker);
            }
            Err(e) => {
                eprintln!("  ✗ Failed to spawn walker {}: {}", i + 1, e);
            }
        }
    }

    println!("\nSuccessfully spawned {} walkers", walkers.len());

    // Display information about spawned walkers
    println!("\nWalker summary:");
    for (i, walker) in walkers.iter().enumerate() {
        let location = walker.location();
        println!(
            "  Walker {}: ID={}, type={}, alive={}, location=({:.1}, {:.1}, {:.1})",
            i + 1,
            walker.id(),
            walker.type_id(),
            walker.is_alive(),
            location.x,
            location.y,
            location.z
        );
    }

    println!("\nWalkers will remain in the simulation.");
    println!("Note: Walkers have no AI controller and won't move automatically.");
}
