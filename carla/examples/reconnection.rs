//! Demonstrates reconnection patterns for handling CARLA server disconnections.
//!
//! This example shows how to:
//! - Connect with retry logic and exponential backoff
//! - Detect connection errors during simulation
//! - Reconnect automatically after a disconnection
//! - Resume work after reconnecting
//!
//! Run with: `cargo run --example reconnection --profile dev-release`

use carla::client::{ActorBase, Client};
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CARLA Reconnection Patterns Demo ===\n");

    // Demo 1: Connect with retry and exponential backoff
    println!("--- Demo 1: Initial Connection with Retry ---");
    let client = connect_with_backoff("127.0.0.1", 2000, 5)?;
    println!("Connected to CARLA v{}\n", client.server_version()?);

    // Demo 2: Simulation loop with reconnection on failure
    println!("--- Demo 2: Simulation Loop with Reconnection ---");
    simulation_loop(client)?;

    println!("\n=== Reconnection demo complete ===");
    Ok(())
}

/// Connects to CARLA with exponential backoff retry.
fn connect_with_backoff(
    host: &str,
    port: u16,
    max_retries: u32,
) -> Result<Client, Box<dyn std::error::Error>> {
    let mut delay = Duration::from_secs(1);

    for attempt in 1..=max_retries {
        println!("Connection attempt {}/{}...", attempt, max_retries);

        match Client::connect(host, port, None) {
            Ok(client) => {
                println!("Connected on attempt {}", attempt);
                return Ok(client);
            }
            Err(e) => {
                if attempt == max_retries {
                    println!("All {} attempts failed", max_retries);
                    return Err(e.into());
                }

                if e.is_connection_error() || e.is_timeout() {
                    println!("  Connection failed: {}. Retrying in {:?}...", e, delay);
                    std::thread::sleep(delay);
                    delay = (delay * 2).min(Duration::from_secs(30));
                } else {
                    // Non-connection error, don't retry
                    return Err(e.into());
                }
            }
        }
    }

    unreachable!()
}

/// Simulation loop that detects disconnection and reconnects.
fn simulation_loop(mut client: Client) -> Result<(), Box<dyn std::error::Error>> {
    let mut world = client.world()?;
    let map = world.map()?;
    let spawn_points = map.recommended_spawn_points()?;

    if spawn_points.is_empty() {
        println!("No spawn points available");
        return Ok(());
    }

    // Spawn a vehicle
    let bp_lib = world.blueprint_library()?;
    let vehicle_bp = bp_lib
        .find("vehicle.tesla.model3")?
        .ok_or("Vehicle blueprint not found")?;
    let vehicle = world.spawn_actor(&vehicle_bp, spawn_points.get(0).unwrap())?;
    println!("Spawned vehicle: ID {}", vehicle.id());

    // Simulation loop with error recovery
    let max_ticks = 100;
    let mut tick_count = 0;

    while tick_count < max_ticks {
        match world.tick() {
            Ok(frame) => {
                tick_count += 1;
                if tick_count % 20 == 0 {
                    println!("  Tick {}/{} (frame {})", tick_count, max_ticks, frame);
                }
            }
            Err(e) if e.is_connection_error() || e.is_timeout() => {
                println!("\n  Connection lost: {}", e);
                println!("  Attempting to reconnect...");

                // Reconnect
                client = connect_with_backoff("127.0.0.1", 2000, 5)?;
                let _world = client.world()?;
                println!("  Reconnected. Resuming simulation.\n");

                // Note: actors from the previous episode are lost after reconnection.
                // In a real application, you would re-spawn actors here.
                break;
            }
            Err(e) => {
                println!("  Unexpected error: {}", e);
                if e.is_retriable() {
                    println!("  Error is retriable, continuing...");
                    std::thread::sleep(Duration::from_millis(100));
                } else {
                    return Err(e.into());
                }
            }
        }
    }

    // Clean up
    let _ = vehicle.destroy();
    println!("Simulation loop completed ({} ticks)", tick_count);

    Ok(())
}
