//! Multiple vehicles example
//!
//! This example demonstrates how to spawn multiple vehicles
//! at different spawn points.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example multiple_vehicles
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

    // Find Tesla Model 3 blueprint
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .expect("Tesla Model 3 not found");

    // Get spawn points
    let spawn_points = world.map().recommended_spawn_points();
    println!("Available spawn points: {}", spawn_points.len());

    // Spawn 5 vehicles (or fewer if not enough spawn points)
    let spawn_count = 5.min(spawn_points.len());
    println!("\nSpawning {} vehicles...", spawn_count);

    let mut vehicles = Vec::new();

    for i in 0..spawn_count {
        let spawn_point = spawn_points.get(i).unwrap();

        match world.spawn_actor(&vehicle_bp, &spawn_point) {
            Ok(vehicle) => {
                println!("  ✓ Vehicle {} spawned (ID: {})", i + 1, vehicle.id());
                vehicles.push(vehicle);
            }
            Err(e) => {
                eprintln!("  ✗ Failed to spawn vehicle {}: {}", i + 1, e);
            }
        }
    }

    println!("\nSuccessfully spawned {} vehicles", vehicles.len());

    // Display information about spawned vehicles
    println!("\nVehicle summary:");
    for (i, vehicle) in vehicles.iter().enumerate() {
        let location = vehicle.location();
        println!(
            "  Vehicle {}: ID={}, alive={}, location=({:.1}, {:.1}, {:.1})",
            i + 1,
            vehicle.id(),
            vehicle.is_alive(),
            location.x,
            location.y,
            location.z
        );
    }

    println!("\nVehicles will remain in the simulation.");
    println!("Restart CARLA to clean up spawned actors.");
}
