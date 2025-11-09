//! Blueprint library example
//!
//! This example demonstrates how to query the blueprint library
//! and filter blueprints by category.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example blueprints
//! ```

use carla::client::Client;

fn main() {
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    println!("Connected!");

    // Get the current world
    println!("\nGetting current world...");
    let world = client.world();
    println!("World ready!");

    let blueprint_library = world.blueprint_library();

    // Get all blueprints
    let all_blueprints = blueprint_library.filter("*");
    println!("\nTotal blueprints: {}", all_blueprints.len());

    // Filter vehicles
    let vehicles = blueprint_library.filter("vehicle.*");
    println!("\nVehicle blueprints: {}", vehicles.len());
    println!("First 5 vehicles:");
    for (i, bp) in vehicles.iter().take(5).enumerate() {
        println!("  {}: {}", i + 1, bp.id());

        // Show some tags
        let tags = bp.tags();
        if !tags.is_empty() {
            print!("     Tags:");
            for tag in tags.iter().take(3) {
                print!(" {}", tag);
            }
            println!();
        }
    }

    // Filter walkers/pedestrians
    let walkers = blueprint_library.filter("walker.pedestrian.*");
    println!("\nWalker blueprints: {}", walkers.len());
    println!("First 5 walkers:");
    for (i, bp) in walkers.iter().take(5).enumerate() {
        println!("  {}: {}", i + 1, bp.id());
    }

    // Filter sensors
    let sensors = blueprint_library.filter("sensor.*");
    println!("\nSensor blueprints: {}", sensors.len());
    println!("First 5 sensors:");
    for (i, bp) in sensors.iter().take(5).enumerate() {
        println!("  {}: {}", i + 1, bp.id());
    }

    // Find specific blueprint
    println!("\nSearching for Tesla Model 3...");
    if let Some(tesla) = blueprint_library.find("vehicle.tesla.model3") {
        println!("Found: {}", tesla.id());

        println!("Tags:");
        for tag in tesla.tags().iter() {
            println!("  - {}", tag);
        }
    } else {
        println!("Tesla Model 3 not found");
    }
}
