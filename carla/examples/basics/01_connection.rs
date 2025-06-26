//! Basic CARLA connection example
//!
//! This example demonstrates how to:
//! - Connect to a CARLA server
//! - Get server version information
//! - Access the world
//! - List available maps

use anyhow::Result;
use carla::actor::ActorExt;
use clap::Parser;

#[path = "../common/mod.rs"]
mod common;
use common::{cli::CommonArgs, connect_with_retry};

#[derive(Parser, Debug)]
#[command(
    author,
    version,
    about = "CARLA example: Basic connection and server info"
)]
struct Args {
    #[command(flatten)]
    common: CommonArgs,
}

fn main() -> Result<()> {
    let args = Args::parse();
    args.common.init_logging();

    println!("=== CARLA Basic Connection Example ===\n");

    // Connect to CARLA
    args.common.print_connection_info();
    let client = connect_with_retry(&args.common.host, args.common.port, args.common.timeout)?;
    println!("✓ Connected successfully!\n");

    // Run the example
    run_example(&client)?;

    println!("\n✓ Example completed successfully!");
    Ok(())
}

fn run_example(client: &carla::client::Client) -> Result<()> {
    // Get server version
    println!("Server Information:");
    println!("==================");
    let version = client.server_version()?;
    println!("Server version: {}", version);

    // Get current world
    let world = client.world()?;
    println!("\nWorld Information:");
    println!("==================");

    // Get current map
    let map = world.map()?;
    println!("Current map: {}", map.name());

    // Get available maps
    let available_maps = client.available_maps()?;
    println!("\nAvailable maps ({}):", available_maps.len());
    for (i, map_name) in available_maps.iter().enumerate() {
        println!("  {}. {}", i + 1, map_name);
    }

    // Get world settings
    let settings = world.settings()?;
    println!("\nWorld Settings:");
    println!("==================");
    println!("Synchronous mode: {}", settings.synchronous_mode);
    println!("Fixed delta seconds: {:?}", settings.fixed_delta_seconds);
    println!("No rendering mode: {}", settings.no_rendering_mode);

    // Get spectator information
    let spectator = world.spectator()?;
    println!("\nSpectator Information:");
    println!("======================");
    println!("Type ID: {}", spectator.type_id());
    let transform = spectator.transform();
    common::print_transform(&transform);

    Ok(())
}
