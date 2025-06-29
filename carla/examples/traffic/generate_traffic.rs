//! Traffic generation example - Python equivalent: generate_traffic.py
//!
//! This example demonstrates spawning multiple vehicles and pedestrians with automated
//! behavior using the Traffic Manager. It includes performance monitoring and statistics.
//!
//! Key features:
//! - Batch vehicle and pedestrian spawning
//! - Traffic Manager integration for autonomous behavior
//! - Performance monitoring and statistics
//! - CSV export of spawn results
//!
//! Note: This example includes TODO markers for missing FFI functionality
//! following CLAUDE.md guidelines.

use anyhow::Result;
use carla::{
    actor::{Actor, ActorExt},
    client::World,
};
use clap::Parser;
use std::{
    thread,
    time::{Duration, Instant},
};

#[path = "../common/mod.rs"]
mod common;
use common::{
    cli::CommonArgs,
    connect_with_retry,
    utils::{CsvWriter, PerformanceStats, ProgressTracker},
};

#[derive(Parser, Debug)]
#[command(
    author,
    version,
    about = "CARLA example: Generate traffic with Traffic Manager"
)]
struct Args {
    #[command(flatten)]
    common: CommonArgs,

    /// Number of vehicles to spawn
    #[arg(long, default_value_t = 30)]
    number_of_vehicles: u32,

    /// Number of pedestrians to spawn
    #[arg(long, default_value_t = 10)]
    number_of_pedestrians: u32,

    /// Simulation duration in seconds
    #[arg(long, default_value_t = 60)]
    duration: u64,

    /// Traffic Manager port
    #[arg(long, default_value_t = 8000)]
    tm_port: u16,

    /// Global vehicle speed percentage difference
    #[arg(long, default_value_t = 30.0)]
    speed_difference: f32,

    /// Global distance to leading vehicle
    #[arg(long, default_value_t = 2.0)]
    distance_to_leading: f32,

    /// Enable collision detection between vehicles
    #[arg(long, default_value_t = true)]
    collision_detection: bool,

    /// CSV output file for spawn statistics
    #[arg(long, default_value = "traffic_stats.csv")]
    output_csv: String,
}

fn main() -> Result<()> {
    let args = Args::parse();

    println!("CARLA Traffic Generation Example");
    println!("================================");
    println!("Vehicles: {}", args.number_of_vehicles);
    println!("Pedestrians: {}", args.number_of_pedestrians);
    println!("Duration: {}s", args.duration);
    println!("Traffic Manager Port: {}", args.tm_port);
    println!();

    // Connect to CARLA
    let client = connect_with_retry(&args.common.host, args.common.port, args.common.timeout)?;
    let world = client.world()?;

    // Initialize performance tracking
    let mut vehicle_stats = PerformanceStats::new();
    let mut pedestrian_stats = PerformanceStats::new();
    let mut csv_writer = CsvWriter::new(&args.output_csv)?;
    csv_writer.write_header(&["type", "blueprint", "spawn_time_ms", "success"])?;

    // Get Traffic Manager instance
    // TODO: Implement Traffic Manager creation in carla-sys
    // This requires adding Client::get_trafficmanager(port) FFI function
    if args.common.verbose {
        println!("Note: Traffic Manager integration not yet implemented - missing FFI functions");
        println!("Vehicles will be spawned without autonomous behavior");
    }

    // For now, we'll spawn vehicles without Traffic Manager
    // In the complete implementation, this would be:
    // let traffic_manager = client.get_traffic_manager(args.tm_port)?;
    // configure_traffic_manager(&traffic_manager, &args)?;

    // Spawn vehicles
    let spawned_vehicles = spawn_vehicles(&world, &args, &mut vehicle_stats, &mut csv_writer)?;

    // Spawn pedestrians
    let spawned_pedestrians =
        spawn_pedestrians(&world, &args, &mut pedestrian_stats, &mut csv_writer)?;

    // Print spawn statistics
    print_spawn_statistics(&vehicle_stats, &pedestrian_stats);

    // Run simulation with progress tracking
    run_simulation(&world, &args, &spawned_vehicles, &spawned_pedestrians)?;

    // Cleanup
    cleanup_actors(spawned_vehicles, spawned_pedestrians)?;

    // Save final statistics
    save_final_statistics(&vehicle_stats, &pedestrian_stats, &args)?;

    println!("\nTraffic generation completed successfully!");
    Ok(())
}

/// Configure Traffic Manager settings
#[allow(dead_code)]
fn configure_traffic_manager(_tm: &(), _args: &Args) -> Result<()> {
    // TODO: Implement Traffic Manager configuration
    // This requires the following FFI functions:
    // - TrafficManager::set_global_distance_to_leading_vehicle(distance)
    // - TrafficManager::global_percentage_speed_difference(percentage)
    // - TrafficManager::collision_detection(actor_list, enabled)

    // Example of what the implementation would look like:
    // tm.set_global_distance_to_leading_vehicle(args.distance_to_leading)?;
    // tm.global_percentage_speed_difference(args.speed_difference)?;
    // tm.collision_detection(&[], args.collision_detection)?;

    Ok(())
}

/// Spawn vehicles with Traffic Manager
fn spawn_vehicles(
    world: &World,
    args: &Args,
    stats: &mut PerformanceStats,
    csv_writer: &mut CsvWriter,
) -> Result<Vec<Actor>> {
    println!("\nSpawning {} vehicles...", args.number_of_vehicles);

    let blueprint_library = world.blueprint_library()?;
    let vehicle_blueprints = blueprint_library.filter("vehicle.*")?;

    if vehicle_blueprints.is_empty() {
        anyhow::bail!("No vehicle blueprints found");
    }

    // Get spawn points
    let map = world.map()?;
    let spawn_points = map.spawn_points();

    let mut spawned = Vec::new();
    let mut progress = ProgressTracker::new(args.number_of_vehicles as u64, 5);

    for i in 0..args.number_of_vehicles.min(spawn_points.len() as u32) {
        let start = Instant::now();
        let blueprint = &vehicle_blueprints[i as usize % vehicle_blueprints.len()];
        let spawn_point = match spawn_points.get(i as usize) {
            Some(point) => point,
            None => break, // No more spawn points available
        };

        match world.try_spawn_actor(blueprint, &spawn_point, None) {
            Ok(Some(actor)) => {
                // TODO: Enable autopilot with Traffic Manager
                // This requires Vehicle::set_autopilot(enabled, tm_port) FFI function
                // For now, we just store the actor

                // In the complete implementation:
                // if let Ok(vehicle) = actor.try_into_vehicle() {
                //     vehicle.set_autopilot(true, args.tm_port)?;
                // }

                let duration_ms = start.elapsed().as_millis() as u64;
                stats.record_operation(duration_ms, true);
                csv_writer.write_row(&[
                    "vehicle".to_string(),
                    blueprint.id(),
                    duration_ms.to_string(),
                    "true".to_string(),
                ])?;

                spawned.push(actor);
                progress.update(spawned.len() as u64);
            }
            Ok(None) => {
                let duration_ms = start.elapsed().as_millis() as u64;
                stats.record_operation(duration_ms, false);
                csv_writer.write_row(&[
                    "vehicle".to_string(),
                    blueprint.id(),
                    duration_ms.to_string(),
                    "false".to_string(),
                ])?;
            }
            Err(e) => {
                if args.common.verbose {
                    eprintln!("Failed to spawn vehicle {i}: {e}");
                }
                stats.record_operation(start.elapsed().as_millis() as u64, false);
            }
        }

        // Small delay to avoid overwhelming the server
        thread::sleep(Duration::from_millis(10));
    }

    // Progress tracker completes automatically when total is reached
    println!("Successfully spawned {} vehicles", spawned.len());
    Ok(spawned)
}

/// Spawn pedestrians with AI controllers
fn spawn_pedestrians(
    world: &World,
    args: &Args,
    stats: &mut PerformanceStats,
    csv_writer: &mut CsvWriter,
) -> Result<Vec<Actor>> {
    println!("\nSpawning {} pedestrians...", args.number_of_pedestrians);

    let blueprint_library = world.blueprint_library()?;
    let walker_blueprints = blueprint_library.filter("walker.pedestrian.*")?;

    if walker_blueprints.is_empty() {
        println!("No pedestrian blueprints found - skipping pedestrian spawning");
        return Ok(Vec::new());
    }

    // TODO: Get pedestrian spawn points
    // This requires Map::get_crosswalks() or similar FFI function
    // For now, we'll use a simplified approach with regular spawn points

    let map = world.map()?;
    let spawn_points = map.spawn_points();

    let mut spawned = Vec::new();
    let mut progress = ProgressTracker::new(args.number_of_pedestrians as u64, 5);

    // Use different spawn points from vehicles by offsetting
    let offset = args.number_of_vehicles as usize;

    for i in 0..args
        .number_of_pedestrians
        .min((spawn_points.len() - offset) as u32)
    {
        let start = Instant::now();
        let blueprint = &walker_blueprints[i as usize % walker_blueprints.len()];
        let spawn_point = match spawn_points.get(offset + i as usize) {
            Some(point) => point,
            None => break, // No more spawn points available
        };

        match world.try_spawn_actor(blueprint, &spawn_point, None) {
            Ok(Some(actor)) => {
                // TODO: Spawn walker AI controller
                // This requires spawning walker.controller blueprint and attaching it
                // For now, pedestrians will be static

                let duration_ms = start.elapsed().as_millis() as u64;
                stats.record_operation(duration_ms, true);
                csv_writer.write_row(&[
                    "pedestrian".to_string(),
                    blueprint.id(),
                    duration_ms.to_string(),
                    "true".to_string(),
                ])?;

                spawned.push(actor);
                progress.update(spawned.len() as u64);
            }
            Ok(None) => {
                let duration_ms = start.elapsed().as_millis() as u64;
                stats.record_operation(duration_ms, false);
            }
            Err(e) => {
                if args.common.verbose {
                    eprintln!("Failed to spawn pedestrian {i}: {e}");
                }
                stats.record_operation(start.elapsed().as_millis() as u64, false);
            }
        }

        thread::sleep(Duration::from_millis(10));
    }

    // Progress tracker completes automatically when total is reached
    println!("Successfully spawned {} pedestrians", spawned.len());
    Ok(spawned)
}

/// Run the simulation with monitoring
fn run_simulation(
    _world: &World,
    args: &Args,
    vehicles: &[Actor],
    pedestrians: &[Actor],
) -> Result<()> {
    println!("\nRunning simulation for {} seconds...", args.duration);
    println!(
        "Active actors: {} vehicles, {} pedestrians",
        vehicles.len(),
        pedestrians.len()
    );

    let mut progress = ProgressTracker::new(args.duration, 5);

    for i in 0..args.duration {
        // TODO: Implement world.tick() for synchronous mode
        // This requires World::tick() FFI function
        // For asynchronous mode, we just wait

        thread::sleep(Duration::from_secs(1));
        progress.update(i + 1);

        // TODO: Monitor actor states and collect telemetry
        // This would require various actor state query functions like:
        // - Actor::get_velocity()
        // - Actor::get_acceleration()
        // - Vehicle::get_traffic_light_state()
        // etc.
    }

    // Progress tracker completes automatically when total is reached
    Ok(())
}

/// Clean up spawned actors
fn cleanup_actors(vehicles: Vec<Actor>, pedestrians: Vec<Actor>) -> Result<()> {
    println!("\nCleaning up actors...");

    let total = vehicles.len() + pedestrians.len();
    let mut progress = ProgressTracker::new(total as u64, 10);
    let mut destroyed = 0;

    // Destroy vehicles
    for mut vehicle in vehicles {
        if let Err(e) = vehicle.destroy() {
            if std::env::var("VERBOSE").is_ok() {
                eprintln!("Failed to destroy vehicle: {e}");
            }
        } else {
            destroyed += 1;
        }
        progress.update(destroyed as u64);
    }

    // Destroy pedestrians
    for mut pedestrian in pedestrians {
        if let Err(e) = pedestrian.destroy() {
            if std::env::var("VERBOSE").is_ok() {
                eprintln!("Failed to destroy pedestrian: {e}");
            }
        } else {
            destroyed += 1;
        }
        progress.update(destroyed as u64);
    }

    // Progress tracker completes automatically when total is reached
    println!("Destroyed {destroyed}/{total} actors");
    Ok(())
}

/// Print spawn statistics
fn print_spawn_statistics(vehicle_stats: &PerformanceStats, pedestrian_stats: &PerformanceStats) {
    println!("\nSpawn Statistics:");
    println!("=================\n");

    println!("Vehicles:");
    print_stats(vehicle_stats);

    println!("\nPedestrians:");
    print_stats(pedestrian_stats);
}

fn print_stats(stats: &PerformanceStats) {
    println!("  Total attempts: {}", stats.total_operations);
    println!("  Successful: {}", stats.successful_operations);
    println!("  Failed: {}", stats.failed_operations);
    println!("  Success rate: {:.1}%", stats.success_rate() * 100.0);

    if stats.total_operations > 0 {
        println!("  Avg spawn time: {:.1}ms", stats.average_duration_ms());
        if let (Some(min), Some(max)) = (stats.min_duration_ms, stats.max_duration_ms) {
            println!("  Min/Max time: {min}ms / {max}ms");
        }
    }
}

/// Save final statistics to file
fn save_final_statistics(
    vehicle_stats: &PerformanceStats,
    pedestrian_stats: &PerformanceStats,
    args: &Args,
) -> Result<()> {
    let stats_file = args.output_csv.replace(".csv", "_summary.csv");
    let mut writer = CsvWriter::new(&stats_file)?;

    writer.write_header(&[
        "actor_type",
        "total_attempts",
        "successful",
        "failed",
        "success_rate",
        "avg_spawn_time_ms",
    ])?;

    writer.write_row(&[
        "vehicle".to_string(),
        vehicle_stats.total_operations.to_string(),
        vehicle_stats.successful_operations.to_string(),
        vehicle_stats.failed_operations.to_string(),
        format!("{:.2}", vehicle_stats.success_rate()),
        format!("{:.2}", vehicle_stats.average_duration_ms()),
    ])?;

    writer.write_row(&[
        "pedestrian".to_string(),
        pedestrian_stats.total_operations.to_string(),
        pedestrian_stats.successful_operations.to_string(),
        pedestrian_stats.failed_operations.to_string(),
        format!("{:.2}", pedestrian_stats.success_rate()),
        format!("{:.2}", pedestrian_stats.average_duration_ms()),
    ])?;

    println!("\nSaved statistics to: {stats_file}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cli_args() {
        let args = Args::parse_from(&[
            "generate_traffic",
            "--number-of-vehicles",
            "50",
            "--number-of-pedestrians",
            "20",
            "--duration",
            "120",
            "--tm-port",
            "8001",
            "--speed-difference",
            "20.0",
        ]);

        assert_eq!(args.number_of_vehicles, 50);
        assert_eq!(args.number_of_pedestrians, 20);
        assert_eq!(args.duration, 120);
        assert_eq!(args.tm_port, 8001);
        assert_eq!(args.speed_difference, 20.0);
    }

    #[test]
    fn test_default_args() {
        let args = Args::parse_from(&["generate_traffic"]);

        assert_eq!(args.number_of_vehicles, 30);
        assert_eq!(args.number_of_pedestrians, 10);
        assert_eq!(args.duration, 60);
        assert_eq!(args.tm_port, 8000);
        assert!(args.collision_detection);
    }

    #[test]
    fn test_performance_stats() {
        let mut stats = PerformanceStats::new();

        stats.record_operation(100, true);
        stats.record_operation(200, true);
        stats.record_operation(150, false);

        assert_eq!(stats.total_operations, 3);
        assert_eq!(stats.successful_operations, 2);
        assert_eq!(stats.failed_operations, 1);
        assert_eq!(stats.success_rate(), 2.0 / 3.0);
        assert_eq!(stats.average_duration_ms(), 150.0);
        assert_eq!(stats.min_duration_ms, Some(100));
        assert_eq!(stats.max_duration_ms, Some(200));
    }
}
