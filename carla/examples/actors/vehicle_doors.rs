// Python equivalent: carla-simulator/PythonAPI/examples/vehicle_doors_demo.py
// Expected behavior: Vehicle door control demonstration (CARLA 0.10.0 feature)
// Key features: Door opening/closing, sequential operation, door state reporting

use anyhow::Result;
use carla::{
    actor::ActorExt,
    geom::{Location, Rotation, Transform},
};
use clap::Parser;
use std::{thread, time::Duration};

#[path = "../common/mod.rs"]
mod common;
use common::{cli::CommonArgs, connect_with_retry, utils::*};

#[derive(Parser, Debug)]
#[command(
    author,
    version,
    about = "CARLA example: Vehicle door control demonstration"
)]
struct Args {
    #[command(flatten)]
    common: CommonArgs,

    #[arg(long, default_value_t = 3)]
    vehicles: u32,

    #[arg(long, default_value_t = 2.0)]
    operation_delay: f64,

    #[arg(long, default_value_t = 3)]
    cycles: u32,

    #[arg(long, default_value = "vehicle.tesla.model3")]
    vehicle_model: String,

    #[arg(long)]
    export_csv: Option<String>,

    #[arg(long, default_value = "0.0,0.0,0.3")]
    spawn_location: String,

    #[arg(long, default_value_t = 8.0)]
    vehicle_spacing: f64,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
struct DoorOperation {
    vehicle_id: String,
    door_type: String,
    operation: String, // "open" or "close"
    timestamp: std::time::Instant,
    duration_ms: u64,
    success: bool,
}

// CARLA 0.10.0 door types based on VehicleDoor enum
const DOOR_TYPES: &[(&str, &str)] = &[
    ("front_left", "Front Left Door"),
    ("front_right", "Front Right Door"),
    ("rear_left", "Rear Left Door"),
    ("rear_right", "Rear Right Door"),
    ("hood", "Hood/Bonnet"),
    ("trunk", "Trunk/Boot"),
];

fn parse_location(location_str: &str) -> Result<Location> {
    let parts: Vec<&str> = location_str.split(',').collect();
    if parts.len() != 3 {
        anyhow::bail!("Location must be in format 'x,y,z', got: {}", location_str);
    }

    let x = parts[0].trim().parse::<f64>()?;
    let y = parts[1].trim().parse::<f64>()?;
    let z = parts[2].trim().parse::<f64>()?;

    Ok(Location { x, y, z })
}

fn main() -> Result<()> {
    let args = Args::parse();

    // Initialize logging
    env_logger::Builder::from_default_env()
        .filter_level(if args.common.verbose {
            log::LevelFilter::Debug
        } else {
            log::LevelFilter::Info
        })
        .init();

    println!("CARLA Vehicle Door Control Demonstration");
    println!("Python equivalent: vehicle_doors_demo.py");
    println!("NEW CARLA 0.10.0 FEATURE");
    println!("=========================================");

    // Parse spawn location
    let base_location = parse_location(&args.spawn_location)?;

    // Connect to CARLA
    let client = connect_with_retry(&args.common.host, args.common.port, args.common.timeout)?;
    let world = client.world()?;

    // Get blueprint library
    let blueprint_library = world.blueprint_library()?;

    // Find specific vehicle blueprint
    let vehicle_blueprint = blueprint_library
        .find(&args.vehicle_model)?
        .ok_or_else(|| anyhow::anyhow!("Vehicle model not found: {}", args.vehicle_model))?;

    println!("Using vehicle model: {}", args.vehicle_model);
    println!(
        "Spawning {} vehicles for door demonstration...",
        args.vehicles
    );

    // Setup CSV export if requested
    let mut csv_writer = if let Some(csv_path) = &args.export_csv {
        let mut writer = CsvWriter::new(csv_path)?;
        writer.write_header(&[
            "vehicle_id",
            "vehicle_index",
            "cycle",
            "door_type",
            "door_name",
            "operation",
            "timestamp_ms",
            "duration_ms",
            "success",
            "x",
            "y",
            "z",
        ])?;
        Some(writer)
    } else {
        None
    };

    // Spawn vehicles
    let mut vehicles = Vec::new();
    for i in 0..args.vehicles {
        let spawn_transform = Transform {
            location: Location {
                x: base_location.x + (i as f64 * args.vehicle_spacing),
                y: base_location.y,
                z: base_location.z,
            },
            rotation: Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };

        match world.try_spawn_actor(&vehicle_blueprint, &spawn_transform, None) {
            Ok(Some(vehicle)) => {
                vehicles.push((vehicle, format!("vehicle_{:02}", i)));
                println!(
                    "  ✓ Spawned vehicle {} at x={:.1}",
                    i, spawn_transform.location.x
                );
            }
            Ok(None) => {
                log::warn!("Failed to spawn vehicle {} - spawn point occupied", i);
            }
            Err(e) => {
                log::warn!("Failed to spawn vehicle {}: {}", i, e);
            }
        }
    }

    println!("Successfully spawned {} vehicles", vehicles.len());

    // TODO: Vehicle door control FFI not implemented
    println!("\n=== Vehicle Door Control ===");
    println!("TODO: Vehicle door control not implemented in FFI");
    println!("This requires CARLA 0.10.0 FFI functions:");
    println!("  - Vehicle::open_door(door_type) FFI function");
    println!("  - Vehicle::close_door(door_type) FFI function");
    println!("  - VehicleDoor enum/struct for door types");
    println!("  - Door state query functions");

    // Print available door types
    println!("\n=== Available Door Types (CARLA 0.10.0) ===");
    for (i, (door_type, description)) in DOOR_TYPES.iter().enumerate() {
        println!("  {}: {} - {}", i + 1, door_type, description);
    }

    // Simulate door operations
    println!("\n=== Simulated Door Operations ===");
    println!(
        "Running {} cycles with {:.1}s operation delays...",
        args.cycles, args.operation_delay
    );

    let operation_delay_ms = (args.operation_delay * 1000.0) as u64;

    // Performance tracking
    let mut door_stats = PerformanceStats::new();
    let mut operation_histogram = Histogram::new();
    let mut door_type_histogram = Histogram::new();
    let mut all_operations = Vec::new();

    for cycle in 1..=args.cycles {
        println!("\n--- Cycle {}/{} ---", cycle, args.cycles);

        for (vehicle_index, (vehicle, vehicle_id)) in vehicles.iter().enumerate() {
            println!("Vehicle {} ({}):", vehicle_index, vehicle_id);

            // Perform door operations for each door type
            for (door_type, door_name) in DOOR_TYPES {
                // Open door operation
                let open_timer = Timer::new();

                // TODO: vehicle.open_door(door_type)?;
                // Simulate door opening
                thread::sleep(Duration::from_millis(operation_delay_ms / 2));

                let open_duration = open_timer.elapsed_ms();
                let open_operation = DoorOperation {
                    vehicle_id: vehicle_id.clone(),
                    door_type: door_type.to_string(),
                    operation: "open".to_string(),
                    timestamp: std::time::Instant::now(),
                    duration_ms: open_duration,
                    success: true, // Simulated success
                };

                door_stats.record_operation(open_duration, true);
                operation_histogram.add("open".to_string());
                door_type_histogram.add(door_type.to_string());
                all_operations.push(open_operation.clone());

                println!(
                    "  ✓ Opened {} ({}) in {}ms",
                    door_name, door_type, open_duration
                );

                // Small delay between open and close
                thread::sleep(Duration::from_millis(operation_delay_ms / 4));

                // Close door operation
                let close_timer = Timer::new();

                // TODO: vehicle.close_door(door_type)?;
                // Simulate door closing
                thread::sleep(Duration::from_millis(operation_delay_ms / 2));

                let close_duration = close_timer.elapsed_ms();
                let close_operation = DoorOperation {
                    vehicle_id: vehicle_id.clone(),
                    door_type: door_type.to_string(),
                    operation: "close".to_string(),
                    timestamp: std::time::Instant::now(),
                    duration_ms: close_duration,
                    success: true, // Simulated success
                };

                door_stats.record_operation(close_duration, true);
                operation_histogram.add("close".to_string());
                all_operations.push(close_operation.clone());

                println!(
                    "  ✓ Closed {} ({}) in {}ms",
                    door_name, door_type, close_duration
                );

                // Export to CSV if enabled
                if let Some(ref mut writer) = csv_writer {
                    let transform = vehicle.transform();

                    // Export both open and close operations
                    for operation in [&open_operation, &close_operation] {
                        writer.write_row(&[
                            vehicle_id.clone(),
                            vehicle_index.to_string(),
                            cycle.to_string(),
                            operation.door_type.clone(),
                            door_name.to_string(),
                            operation.operation.clone(),
                            operation.timestamp.elapsed().as_millis().to_string(),
                            operation.duration_ms.to_string(),
                            operation.success.to_string(),
                            transform.location.x.to_string(),
                            transform.location.y.to_string(),
                            transform.location.z.to_string(),
                        ])?;
                    }
                }

                // Brief pause between different door types
                thread::sleep(Duration::from_millis(100));
            }

            println!("  → All doors operated for vehicle {}", vehicle_index);
        }

        // Pause between cycles
        if cycle < args.cycles {
            println!("Waiting before next cycle...");
            thread::sleep(Duration::from_millis(operation_delay_ms));
        }
    }

    // Print door operation statistics
    println!("\n=== Door Operation Statistics ===");
    println!("Total operations performed: {}", all_operations.len());
    println!("Vehicles controlled: {}", vehicles.len());
    println!("Cycles completed: {}", args.cycles);
    println!("Door types per vehicle: {}", DOOR_TYPES.len());
    println!("Operations per vehicle per cycle: {}", DOOR_TYPES.len() * 2); // open + close
    println!(
        "Average operation time: {:.1}ms",
        door_stats.average_duration_ms()
    );
    println!("Success rate: {:.1}%", door_stats.success_rate() * 100.0);

    if let Some(min) = door_stats.min_duration_ms {
        println!("Fastest operation: {}ms", min);
    }
    if let Some(max) = door_stats.max_duration_ms {
        println!("Slowest operation: {}ms", max);
    }

    // Print operation type histogram
    println!("\n=== Operation Type Distribution ===");
    operation_histogram.print();

    // Print door type histogram
    println!("\n=== Door Type Usage ===");
    door_type_histogram.print();

    // Detailed operation summary
    println!("\n=== Door Operation Summary ===");
    for (door_type, door_name) in DOOR_TYPES {
        let door_operations: Vec<_> = all_operations
            .iter()
            .filter(|op| op.door_type == *door_type)
            .collect();

        let open_ops = door_operations
            .iter()
            .filter(|op| op.operation == "open")
            .count();
        let close_ops = door_operations
            .iter()
            .filter(|op| op.operation == "close")
            .count();

        println!(
            "  {}: {} opens, {} closes, {} total operations",
            door_name,
            open_ops,
            close_ops,
            door_operations.len()
        );
    }

    // Flush CSV if enabled
    if let Some(ref mut writer) = csv_writer {
        writer.flush()?;
        println!(
            "\nDoor operation data exported to: {}",
            args.export_csv.as_ref().unwrap()
        );
    }

    // Cleanup vehicles
    println!("\n=== Cleanup ===");
    println!("Destroying {} vehicles...", vehicles.len());

    let mut cleanup_stats = PerformanceStats::new();
    for (mut vehicle, vehicle_id) in vehicles {
        let cleanup_timer = Timer::new();
        match vehicle.destroy() {
            Ok(_) => {
                cleanup_stats.record_operation(cleanup_timer.elapsed_ms(), true);
                log::debug!("Destroyed vehicle: {}", vehicle_id);
            }
            Err(e) => {
                cleanup_stats.record_operation(cleanup_timer.elapsed_ms(), false);
                log::warn!("Failed to destroy vehicle {}: {}", vehicle_id, e);
            }
        }
    }

    println!(
        "Cleanup success rate: {:.1}%",
        cleanup_stats.success_rate() * 100.0
    );

    println!("\nVehicle door control demonstration completed!");

    // TODO: Notable missing features documented
    println!("\n=== Missing Features (TODO) ===");
    println!("1. Vehicle::open_door(door_type) FFI function");
    println!("2. Vehicle::close_door(door_type) FFI function");
    println!("3. VehicleDoor enum with door types (FrontLeft, FrontRight, etc.)");
    println!("4. Door state query functions (is_door_open, get_door_state)");
    println!("5. Animated door opening/closing with physics");
    println!("6. Door lock/unlock functionality");
    println!("7. Vehicle-specific door availability checking");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cli_args_compatibility() {
        let args = Args::try_parse_from(&[
            "vehicle_doors",
            "--host",
            "localhost",
            "--port",
            "2000",
            "--vehicles",
            "3",
            "--operation-delay",
            "2.0",
            "--cycles",
            "3",
            "--vehicle-model",
            "vehicle.tesla.model3",
            "--spawn-location",
            "0.0,0.0,0.3",
            "--vehicle-spacing",
            "8.0",
        ])
        .unwrap();

        assert_eq!(args.common.host, "localhost");
        assert_eq!(args.common.port, 2000);
        assert_eq!(args.vehicles, 3);
        assert_eq!(args.operation_delay, 2.0);
        assert_eq!(args.cycles, 3);
        assert_eq!(args.vehicle_model, "vehicle.tesla.model3");
        assert_eq!(args.vehicle_spacing, 8.0);
    }

    #[test]
    fn test_parse_location() {
        let location = parse_location("1.5,2.5,3.5").unwrap();
        assert_eq!(location.x, 1.5);
        assert_eq!(location.y, 2.5);
        assert_eq!(location.z, 3.5);

        assert!(parse_location("invalid").is_err());
        assert!(parse_location("1,2").is_err());
    }

    #[test]
    fn test_door_types_availability() {
        assert!(!DOOR_TYPES.is_empty());
        assert!(DOOR_TYPES.len() >= 6);

        // Check that all door types have names and descriptions
        for (door_type, description) in DOOR_TYPES {
            assert!(!door_type.is_empty());
            assert!(!description.is_empty());
        }

        // Verify specific door types exist
        let door_names: Vec<_> = DOOR_TYPES.iter().map(|(name, _)| *name).collect();
        assert!(door_names.contains(&"front_left"));
        assert!(door_names.contains(&"front_right"));
        assert!(door_names.contains(&"hood"));
        assert!(door_names.contains(&"trunk"));
    }

    #[test]
    fn test_door_operation_creation() {
        let operation = DoorOperation {
            vehicle_id: "test_vehicle".to_string(),
            door_type: "front_left".to_string(),
            operation: "open".to_string(),
            timestamp: std::time::Instant::now(),
            duration_ms: 100,
            success: true,
        };

        assert_eq!(operation.vehicle_id, "test_vehicle");
        assert_eq!(operation.door_type, "front_left");
        assert_eq!(operation.operation, "open");
        assert_eq!(operation.duration_ms, 100);
        assert!(operation.success);
    }
}
