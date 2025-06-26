//! CARLA world information example
//!
//! This example demonstrates how to:
//! - Get detailed world information
//! - Access weather parameters
//! - List all actors in the world
//! - Get world snapshots

use anyhow::Result;
use carla::{actor::ActorExt, client::Client};
use clap::Parser;

#[path = "../common/mod.rs"]
mod common;
use common::{cli::CommonArgs, connect_with_retry, ensure_clean_world};

#[derive(Parser, Debug)]
#[command(author, version, about = "CARLA example: World information and actors")]
struct Args {
    #[command(flatten)]
    common: CommonArgs,
}

fn main() -> Result<()> {
    let args = Args::parse();
    args.common.init_logging();

    println!("=== CARLA World Information Example ===\n");

    // Connect to CARLA
    args.common.print_connection_info();
    let client = connect_with_retry(&args.common.host, args.common.port, args.common.timeout)?;
    println!("✓ Connected successfully!\n");

    // Run the example
    run_example(&client, &args.common)?;

    println!("\n✓ Example completed successfully!");
    Ok(())
}

fn run_example(client: &Client, args: &CommonArgs) -> Result<()> {
    let world = client.world()?;

    // Clean world if requested
    if args.clean {
        ensure_clean_world(&world)?;
    }

    // Get world ID
    println!("World Information:");
    println!("==================");
    println!("World ID: {}", world.id());

    // Get weather parameters
    let weather = world.weather()?;
    println!("\nWeather Parameters:");
    println!("===================");
    println!("Cloudiness: {:.1}%", weather.cloudiness);
    println!("Precipitation: {:.1}%", weather.precipitation);
    println!(
        "Precipitation deposits: {:.1}%",
        weather.precipitation_deposits
    );
    println!("Wind intensity: {:.1}%", weather.wind_intensity);
    println!("Sun azimuth angle: {:.1}°", weather.sun_azimuth_angle);
    println!("Sun altitude angle: {:.1}°", weather.sun_altitude_angle);
    println!("Fog density: {:.1}%", weather.fog_density);
    println!("Fog distance: {:.1}m", weather.fog_distance);
    println!("Wetness: {:.1}%", weather.wetness);
    println!("Fog falloff: {:.1}", weather.fog_falloff);
    println!("Scattering intensity: {:.1}", weather.scattering_intensity);
    println!("Mie scattering scale: {:.1}", weather.mie_scattering_scale);
    println!(
        "Rayleigh scattering scale: {:.1}",
        weather.rayleigh_scattering_scale
    );

    // Get all actors
    let actors = world.actors()?;
    println!("\nActors in World: {} total", actors.len());
    println!("===================");

    // Group actors by type
    let mut actor_types: std::collections::HashMap<String, usize> =
        std::collections::HashMap::new();
    for actor in actors.iter() {
        let type_id = actor.type_id();
        *actor_types.entry(type_id.to_string()).or_insert(0) += 1;
    }

    // Display actor counts by type
    let mut sorted_types: Vec<_> = actor_types.into_iter().collect();
    sorted_types.sort_by(|a, b| b.1.cmp(&a.1));

    for (type_id, count) in sorted_types {
        println!("  {}: {}", type_id, count);
    }

    // Get world snapshot
    let snapshot = world.snapshot()?;
    println!("\nWorld Snapshot:");
    println!("===============");
    println!("Frame: {}", snapshot.timestamp.frame());
    println!("Timestamp: {:.3}s", snapshot.timestamp.elapsed_seconds());
    println!(
        "Delta time: {:.3}s ({:.1} FPS)",
        snapshot.timestamp.delta_seconds(),
        1.0 / snapshot.timestamp.delta_seconds()
    );

    // Get IMU sensor data if available (example of accessing specific actor types)
    let imu_sensors: Vec<_> = actors
        .iter()
        .filter(|a| a.type_id().contains("sensor.other.imu"))
        .collect();

    if !imu_sensors.is_empty() {
        println!("\nIMU Sensors Found: {}", imu_sensors.len());
        for (i, sensor) in imu_sensors.iter().enumerate() {
            println!("  {}. ID: {}", i + 1, sensor.id());
        }
    }

    Ok(())
}
