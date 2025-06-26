//! CARLA vehicle spawning example
//!
//! This example demonstrates how to:
//! - Find vehicle blueprints
//! - Get spawn points from the map
//! - Spawn a vehicle at a valid location
//! - Control basic vehicle properties
//! - Clean up actors when done

use anyhow::Result;
use carla::{actor::ActorExt, client::Client};
use clap::Parser;
use std::{thread, time::Duration};

#[path = "../common/mod.rs"]
mod common;
use common::{cli::CommonArgs, connect_with_retry, ensure_clean_world};

#[derive(Parser, Debug)]
#[command(author, version, about = "CARLA example: Spawn and control a vehicle")]
struct Args {
    #[command(flatten)]
    common: CommonArgs,

    /// Vehicle blueprint ID to spawn (e.g., vehicle.tesla.model3)
    #[arg(long, default_value = "vehicle.tesla.model3")]
    vehicle: String,

    /// How long to keep the vehicle alive (seconds)
    #[arg(short, long, default_value_t = 10)]
    duration: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();
    args.common.init_logging();

    println!("=== CARLA Vehicle Spawning Example ===\n");

    // Connect to CARLA
    args.common.print_connection_info();
    let client = connect_with_retry(&args.common.host, args.common.port, args.common.timeout)?;
    println!("✓ Connected successfully!\n");

    // Run the example
    run_example(&client, &args)?;

    println!("\n✓ Example completed successfully!");
    Ok(())
}

fn run_example(client: &Client, args: &Args) -> Result<()> {
    let world = client.world()?;

    // Clean world if requested
    if args.common.clean {
        ensure_clean_world(&world)?;
    }

    // Get blueprint library
    let blueprint_library = world.blueprint_library()?;

    // Find the vehicle blueprint
    println!("Finding vehicle blueprint: {}", args.vehicle);
    let vehicle_bp = blueprint_library
        .find(&args.vehicle)?
        .ok_or_else(|| anyhow::anyhow!("Vehicle blueprint '{}' not found", args.vehicle))?;
    println!("✓ Found blueprint: {}\n", vehicle_bp.id());

    // Get spawn points from the map
    let map = world.map()?;
    let spawn_points = map.spawn_points();
    if spawn_points.is_empty() {
        anyhow::bail!("No spawn points available on this map");
    }
    println!(
        "Found {} spawn points on map '{}'",
        spawn_points.len(),
        map.name()
    );

    // Choose a spawn point (first one for simplicity)
    let spawn_point = spawn_points.get(0).unwrap();
    println!("\nSpawning vehicle at:");
    common::print_transform(&spawn_point);

    // Spawn the vehicle
    let actor = world.spawn_actor(&vehicle_bp, &spawn_point, None)?;
    println!("\n✓ Vehicle spawned successfully!");
    println!("  Actor ID: {}", actor.id());
    println!("  Type ID: {}", actor.type_id());

    // Convert to vehicle to enable vehicle-specific control
    let vehicle = match actor.into_vehicle() {
        Ok(v) => v,
        Err(_) => anyhow::bail!("Spawned actor is not a vehicle"),
    };

    // Note: Autopilot and manual control may cause issues in CARLA 0.10.0
    // For this example, we'll just observe the vehicle without controlling it
    println!(
        "\nVehicle spawned at default location. Observing for {} seconds...",
        args.duration
    );

    for i in 0..args.duration {
        thread::sleep(Duration::from_secs(1));

        // Get vehicle status every second
        // Note: In CARLA 0.10.0, accessing vehicle properties may be unstable
        println!("  [{:2}s] Vehicle is alive: {}", i + 1, vehicle.is_alive());
    }

    // Clean up
    println!("\nCleaning up...");
    // Convert back to actor for destruction
    let mut actor = vehicle.into_actor();
    actor.destroy()?;
    println!("✓ Vehicle destroyed");

    Ok(())
}
