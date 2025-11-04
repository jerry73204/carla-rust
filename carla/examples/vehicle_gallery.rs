//! Vehicle Gallery Example
//!
//! This example demonstrates:
//! - Iterating through all vehicle blueprints
//! - Spawning each vehicle temporarily
//! - Controlling the spectator camera to view vehicles
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example vehicle_gallery
//! ```

use carla::{
    client::{ActorBase, Client, Vehicle},
    geom::{Location, Rotation, Transform},
};
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CARLA Vehicle Gallery ===\n");

    // Connect to CARLA
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    let mut world = client.world();
    println!("✓ Connected! Current map: {}\n", world.map().name());

    // Get all vehicle blueprints
    let blueprint_library = world.blueprint_library();
    let vehicle_blueprints: Vec<_> = blueprint_library
        .iter()
        .filter(|bp| bp.id().starts_with("vehicle."))
        .collect();

    println!("Found {} vehicle blueprints\n", vehicle_blueprints.len());

    // Get spawn point
    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).ok_or("No spawn points available")?;

    // Get spectator for camera control
    let spectator = world.spectator();
    println!("Spectator camera ready\n");

    println!("=== Vehicle Gallery ===");
    println!("Displaying each vehicle for 3 seconds...\n");

    // Iterate through each vehicle blueprint
    for (index, blueprint) in vehicle_blueprints.iter().enumerate() {
        println!(
            "[{}/{}] {}",
            index + 1,
            vehicle_blueprints.len(),
            blueprint.id()
        );

        // Display tags
        if !blueprint.tags().is_empty() {
            println!("  Tags: {}", blueprint.tags().join(", "));
        }

        // Spawn the vehicle
        match world.spawn_actor(blueprint, spawn_point) {
            Ok(actor) => {
                if let Ok(vehicle) = Vehicle::try_from(actor) {
                    // Position camera to view the vehicle
                    let vehicle_transform = vehicle.transform();
                    let vehicle_location = vehicle_transform.location;

                    // Create camera transform: 8m behind and 3m above the vehicle
                    let camera_transform = Transform {
                        location: Location::new(
                            vehicle_location.x - 8.0,
                            vehicle_location.y,
                            vehicle_location.z + 3.0,
                        ),
                        rotation: Rotation::new(0.0, 0.0, -20.0_f32.to_degrees()),
                    };

                    spectator.set_transform(&camera_transform);

                    // Display vehicle for 3 seconds
                    thread::sleep(Duration::from_secs(3));

                    // Clean up vehicle
                    vehicle.destroy();
                    println!("  ✓ Displayed and cleaned up\n");
                } else {
                    println!("  ⚠ Failed to convert to vehicle\n");
                }
            }
            Err(e) => {
                println!("  ⚠ Failed to spawn: {}\n", e);
            }
        }
    }

    println!("=== Gallery Complete ===");
    println!("Displayed {} vehicles", vehicle_blueprints.len());

    Ok(())
}
