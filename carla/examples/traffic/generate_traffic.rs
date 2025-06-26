// Simplified version of generate_traffic.py
// This is a minimal example that compiles without external dependencies

use anyhow::Result;
use carla::actor::ActorExt;
use clap::Parser;
use std::{thread, time::Duration};

#[path = "../common/mod.rs"]
mod common;
use common::{cli::CommonArgs, connect_with_retry};

#[derive(Parser, Debug)]
#[command(author, version, about = "CARLA example: Generate traffic (minimal)")]
struct Args {
    #[command(flatten)]
    common: CommonArgs,

    #[arg(long, default_value_t = 10)]
    number_of_vehicles: u32,

    #[arg(long, default_value_t = 30)]
    duration: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();

    println!("CARLA Traffic Generation Example (Minimal)");
    println!("=========================================");

    // Connect to CARLA
    let client = connect_with_retry(&args.common.host, args.common.port, args.common.timeout)?;
    let world = client.world()?;

    // Get blueprint library
    let blueprint_library = world.blueprint_library()?;
    let vehicle_blueprints = blueprint_library.filter("vehicle.*")?;

    if vehicle_blueprints.is_empty() {
        anyhow::bail!("No vehicle blueprints found");
    }

    println!("Found {} vehicle blueprints", vehicle_blueprints.len());

    // Get spawn points
    let map = world.map()?;
    let spawn_points = map.spawn_points();
    let available_spawn_points = spawn_points.len() as u32;

    let actual_vehicles = args.number_of_vehicles.min(available_spawn_points);
    println!("Will spawn {} vehicles", actual_vehicles);

    // Spawn vehicles
    let mut spawned_vehicles = Vec::new();
    for i in 0..actual_vehicles {
        let blueprint = &vehicle_blueprints[i as usize % vehicle_blueprints.len()];
        let spawn_point = spawn_points.get(i as usize).unwrap();

        match world.try_spawn_actor(blueprint, &spawn_point, None) {
            Ok(Some(actor)) => {
                spawned_vehicles.push(actor);
                println!("Spawned vehicle {}/{}", i + 1, actual_vehicles);
            }
            Ok(None) => {
                eprintln!("Failed to spawn vehicle {} - spawn point occupied", i);
            }
            Err(e) => {
                eprintln!("Failed to spawn vehicle {}: {}", i, e);
            }
        }

        thread::sleep(Duration::from_millis(10));
    }

    println!("Successfully spawned {} vehicles", spawned_vehicles.len());

    // Run simulation
    println!("\nRunning simulation for {} seconds...", args.duration);
    for i in 0..args.duration {
        thread::sleep(Duration::from_secs(1));
        if i % 10 == 0 {
            println!("Simulation time: {}s / {}s", i + 1, args.duration);
        }
    }

    // Cleanup
    println!("\nCleaning up {} vehicles...", spawned_vehicles.len());
    for (idx, mut actor) in spawned_vehicles.into_iter().enumerate() {
        if let Err(e) = actor.destroy() {
            eprintln!("Failed to destroy vehicle {}: {}", idx, e);
        }
    }

    println!("\nTraffic generation completed!");
    Ok(())
}
