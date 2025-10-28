//! Start Recording Example
//!
//! This example demonstrates how to start and stop recording a CARLA session.
//! Recordings capture all actor movements and can be replayed later.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example start_recording
//! ```

use carla::client::{ActorBase, Client, Vehicle};
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CARLA Start Recording Example ===\n");

    // Connect to CARLA
    println!("Connecting to CARLA simulator...");
    let mut client = Client::connect("localhost", 2000, None);
    let mut world = client.world();
    println!("✓ Connected! Current map: {}\n", world.map().name());

    // Spawn a vehicle to record
    println!("Spawning vehicle...");
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Tesla Model 3 blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).ok_or("No spawn points available")?;

    let actor = world.spawn_actor(&vehicle_bp, &spawn_point)?;
    let vehicle = Vehicle::try_from(actor).map_err(|_| "Failed to convert to vehicle")?;

    println!("✓ Vehicle spawned (ID: {})\n", vehicle.id());

    // Enable autopilot to create interesting movement
    vehicle.set_autopilot(true);
    println!("✓ Autopilot enabled\n");

    // Start recording
    let recording_filename = "recording01.log";
    println!("Starting recorder...");
    println!("  Filename: {}", recording_filename);
    println!("  Additional data: enabled");

    let result = client.start_recorder(recording_filename, true);
    println!("✓ Recording started: {}\n", result);

    // Let the simulation run and record
    println!("Recording for 30 seconds...");
    for i in 1..=30 {
        thread::sleep(Duration::from_secs(1));
        println!("  Recording... {} seconds", i);
    }

    // Stop recording
    println!("\nStopping recorder...");
    client.stop_recorder();
    println!("✓ Recording stopped");

    // Cleanup
    vehicle.destroy();
    println!("✓ Vehicle destroyed");

    println!("\n=== Recording Complete ===");
    println!("Recording saved to: {}", recording_filename);
    println!("\nTo replay this recording, run:");
    println!("  cargo run --example start_replaying");
    println!("\nTo query recording info, run:");
    println!("  cargo run --example show_recorder_info");

    Ok(())
}
