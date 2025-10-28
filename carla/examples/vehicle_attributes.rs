//! Vehicle attributes example
//!
//! This example demonstrates how to query vehicle attributes
//! from the blueprint and spawned actor.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example vehicle_attributes
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

    // Find vehicle blueprint
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .expect("Tesla Model 3 not found");

    println!("Blueprint: {}\n", vehicle_bp.id());

    // Spawn the vehicle
    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).expect("No spawn points available");

    let vehicle = world
        .spawn_actor(&vehicle_bp, &spawn_point)
        .expect("Failed to spawn vehicle");

    println!("Vehicle spawned: ID {}\n", vehicle.id());

    // Get all attributes
    let attributes = vehicle.attributes();
    println!("Vehicle attributes ({} total):", attributes.len());

    for attr in attributes.iter() {
        println!("  {} = {:?}", attr.id(), attr.value());
    }

    // Query specific attributes
    println!("\nCommon attributes:");

    // Role name
    if let Some(role_name) = attributes.iter().find(|a| a.id() == "role_name") {
        println!("  Role name: {:?}", role_name.value());
    }

    // Number of wheels
    if let Some(wheels) = attributes.iter().find(|a| a.id() == "number_of_wheels") {
        println!("  Number of wheels: {:?}", wheels.value());
    }

    // Object type
    if let Some(obj_type) = attributes.iter().find(|a| a.id() == "object_type") {
        println!("  Object type: {:?}", obj_type.value());
    }

    println!("\nVehicle will remain in the simulation.");
}
