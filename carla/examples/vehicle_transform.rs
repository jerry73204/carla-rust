//! Vehicle transform example
//!
//! This example demonstrates how to get the location and transform
//! of a spawned vehicle.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example vehicle_transform
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

    // Spawn a vehicle
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .expect("Tesla Model 3 not found");

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).expect("No spawn points available");

    let vehicle = world
        .spawn_actor(&vehicle_bp, &spawn_point)
        .expect("Failed to spawn vehicle");

    println!("Vehicle spawned: ID {}\n", vehicle.id());

    // Get vehicle transform (position + rotation)
    let transform = vehicle.transform();
    println!("Transform:");
    println!("  Translation:");
    println!("    x: {:.2}", transform.translation.x);
    println!("    y: {:.2}", transform.translation.y);
    println!("    z: {:.2}", transform.translation.z);
    println!("  Rotation: {:?}", transform.rotation);

    // Get just the location (convenience method)
    let location = vehicle.location();
    println!("\nLocation (shortcut):");
    println!("  x: {:.2}", location.x);
    println!("  y: {:.2}", location.y);
    println!("  z: {:.2}", location.z);

    // Get velocity
    let velocity = vehicle.velocity();
    println!("\nVelocity:");
    println!("  x: {:.2} m/s", velocity.x);
    println!("  y: {:.2} m/s", velocity.y);
    println!("  z: {:.2} m/s", velocity.z);

    // Calculate speed
    let speed = (velocity.x.powi(2) + velocity.y.powi(2) + velocity.z.powi(2)).sqrt();
    println!("  Speed: {:.2} m/s ({:.2} km/h)", speed, speed * 3.6);

    println!("\nVehicle will remain in the simulation.");
}
