//! Vehicle Bounding Box API Example
//!
//! This example demonstrates querying a vehicle's bounding box.
//!
//! # Usage
//! ```bash
//! cargo run --example vehicle_bounding_box --profile dev-release
//! ```

use anyhow::Result;
use carla::client::{ActorBase, Client};

fn main() -> Result<()> {
    println!("Vehicle Bounding Box Demo - Connecting to CARLA...");

    // Connect to CARLA
    let client = Client::connect("localhost", 2000, 0);
    let mut world = client.world();

    println!("Connected! Spawning vehicle...");

    // Get spawn points
    let map = world.map();
    let spawn_points = map.recommended_spawn_points();

    if spawn_points.is_empty() {
        return Err(anyhow::anyhow!("Need at least 1 spawn point"));
    }

    // Spawn a vehicle
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .filter("vehicle.*")
        .iter()
        .next()
        .ok_or_else(|| anyhow::anyhow!("No vehicle blueprints found"))?;

    let spawn_point = &spawn_points.as_slice()[0].to_na();
    let vehicle = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle = carla::client::Vehicle::try_from(vehicle)
        .map_err(|_| anyhow::anyhow!("Failed to cast to Vehicle"))?;

    println!("Spawned vehicle: {}", vehicle.type_id());

    // Query bounding box
    println!("\nQuerying vehicle bounding box...");
    let bbox = vehicle.bounding_box();

    println!("\nBounding Box Information:");
    println!("  Extent (half-dimensions):");
    println!("    X (length/2): {:.3} m", bbox.extent.x);
    println!("    Y (width/2):  {:.3} m", bbox.extent.y);
    println!("    Z (height/2): {:.3} m", bbox.extent.z);

    println!("\n  Full dimensions:");
    println!("    Length: {:.3} m", bbox.extent.x * 2.0);
    println!("    Width:  {:.3} m", bbox.extent.y * 2.0);
    println!("    Height: {:.3} m", bbox.extent.z * 2.0);

    println!("\n  Center (transform):");
    println!("    X: {:.3} m", bbox.transform.translation.x);
    println!("    Y: {:.3} m", bbox.transform.translation.y);
    println!("    Z: {:.3} m", bbox.transform.translation.z);

    // Verify bounding box is non-zero
    assert!(
        bbox.extent.x > 0.0,
        "Bounding box X extent should be positive"
    );
    assert!(
        bbox.extent.y > 0.0,
        "Bounding box Y extent should be positive"
    );
    assert!(
        bbox.extent.z > 0.0,
        "Bounding box Z extent should be positive"
    );

    println!("\n✅ SUCCESS: Bounding box query works correctly!");

    // Cleanup
    println!("\nCleaning up...");
    vehicle.destroy();

    println!("Done!");
    Ok(())
}
