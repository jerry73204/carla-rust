// Python equivalent: carla-simulator/PythonAPI/examples/vehicle_lights_demo.py
// Expected behavior: Demonstrates vehicle lighting system and state control
// Key features: Light state enumeration, automated cycling, lighting control

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
    about = "CARLA example: Vehicle lighting system demonstration"
)]
struct Args {
    #[command(flatten)]
    common: CommonArgs,

    #[arg(long, default_value_t = 10)]
    vehicles: u32,

    #[arg(long, default_value_t = 2.0)]
    cycle_duration: f64,

    #[arg(long, default_value_t = 30.0)]
    total_duration: f64,

    #[arg(long, default_value = "vehicle.tesla.model3")]
    vehicle_model: String,

    #[arg(long)]
    export_csv: Option<String>,

    #[arg(long, default_value = "0.0,0.0,0.3")]
    spawn_location: String,

    #[arg(long, default_value_t = 5.0)]
    vehicle_spacing: f64,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
struct LightingState {
    vehicle_id: String,
    cycle_time: f64,
    light_pattern: String,
    timestamp: std::time::Instant,
}

// Define lighting patterns for demonstration
const LIGHTING_PATTERNS: &[(&str, &str)] = &[
    ("off", "All lights off"),
    ("position", "Position lights only"),
    ("low_beam", "Low beam headlights"),
    ("high_beam", "High beam headlights"),
    ("brake", "Brake lights"),
    ("reverse", "Reverse lights"),
    ("turn_left", "Left turn signal"),
    ("turn_right", "Right turn signal"),
    ("hazard", "Hazard lights"),
    ("fog", "Fog lights"),
    ("all_on", "All lights on"),
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

    println!("CARLA Vehicle Lighting Demonstration");
    println!("Python equivalent: vehicle_lights_demo.py");
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
        "Spawning {} vehicles for lighting demonstration...",
        args.vehicles
    );

    // Setup CSV export if requested
    let mut csv_writer = if let Some(csv_path) = &args.export_csv {
        let mut writer = CsvWriter::new(csv_path)?;
        writer.write_header(&[
            "vehicle_id",
            "vehicle_index",
            "cycle_time",
            "light_pattern",
            "pattern_description",
            "timestamp_ms",
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
                vehicles.push((vehicle, format!("vehicle_{i:02}")));
                println!(
                    "  ✓ Spawned vehicle {} at x={:.1}",
                    i, spawn_transform.location.x
                );
            }
            Ok(None) => {
                log::warn!("Failed to spawn vehicle {i} - spawn point occupied");
            }
            Err(e) => {
                log::warn!("Failed to spawn vehicle {i}: {e}");
            }
        }
    }

    println!("Successfully spawned {} vehicles", vehicles.len());

    // TODO: Vehicle lighting control not implemented
    println!("\n=== Vehicle Lighting Control ===");
    println!("TODO: Vehicle lighting control not implemented in FFI");
    println!("This requires:");
    println!("  - Vehicle::set_light_state() or similar FFI function");
    println!("  - VehicleLightState enum/struct for light patterns");
    println!("  - Light state query functions");

    // Simulate lighting demonstration
    println!("\n=== Simulated Lighting Demonstration ===");
    println!("Running lighting cycles for {:.1}s...", args.total_duration);

    let cycle_duration_ms = (args.cycle_duration * 1000.0) as u64;
    let total_duration_ms = (args.total_duration * 1000.0) as u64;
    let total_cycles = total_duration_ms / cycle_duration_ms;

    println!("Pattern cycle duration: {:.1}s", args.cycle_duration);
    println!("Total cycles: {total_cycles}");
    println!("Available lighting patterns: {}", LIGHTING_PATTERNS.len());

    // Performance tracking
    let mut lighting_stats = PerformanceStats::new();
    let mut pattern_histogram = Histogram::new();
    let mut progress = ProgressTracker::new(total_cycles, total_cycles / 10);

    for cycle in 0..total_cycles {
        let cycle_start = std::time::Instant::now();

        // Select lighting pattern for this cycle
        let pattern_index = cycle as usize % LIGHTING_PATTERNS.len();
        let (pattern_name, pattern_description) = LIGHTING_PATTERNS[pattern_index];

        println!(
            "\nCycle {}/{}: Setting lights to '{}' ({})",
            cycle + 1,
            total_cycles,
            pattern_name,
            pattern_description
        );

        // Apply lighting pattern to all vehicles
        for (vehicle_index, (vehicle, vehicle_id)) in vehicles.iter().enumerate() {
            // TODO: Apply actual lighting control
            // vehicle.set_light_state(light_state)?;

            // Simulate lighting state application
            let lighting_state = LightingState {
                vehicle_id: vehicle_id.clone(),
                cycle_time: args.cycle_duration,
                light_pattern: pattern_name.to_string(),
                timestamp: std::time::Instant::now(),
            };

            // Export to CSV if enabled
            if let Some(ref mut writer) = csv_writer {
                let transform = vehicle.transform();
                writer.write_row(&[
                    vehicle_id.clone(),
                    vehicle_index.to_string(),
                    args.cycle_duration.to_string(),
                    pattern_name.to_string(),
                    pattern_description.to_string(),
                    lighting_state.timestamp.elapsed().as_millis().to_string(),
                    transform.location.x.to_string(),
                    transform.location.y.to_string(),
                    transform.location.z.to_string(),
                ])?;
            }

            log::debug!("  Vehicle {vehicle_index}: Applied '{pattern_name}' lighting pattern");
        }

        // Record pattern usage
        pattern_histogram.add(pattern_name.to_string());

        // Wait for cycle duration
        thread::sleep(Duration::from_millis(cycle_duration_ms));

        let cycle_time = cycle_start.elapsed().as_millis() as u64;
        lighting_stats.record_operation(cycle_time, true);

        progress.update(cycle + 1);
    }

    // Print lighting demonstration statistics
    println!("\n=== Lighting Demonstration Statistics ===");
    println!("Total cycles completed: {total_cycles}");
    println!("Vehicles controlled: {}", vehicles.len());
    println!(
        "Average cycle time: {:.1}ms",
        lighting_stats.average_duration_ms()
    );
    println!("Total demonstration time: {:.1}s", args.total_duration);

    // Print pattern usage histogram
    println!("\n=== Lighting Pattern Usage ===");
    pattern_histogram.print();

    // Print detailed pattern information
    println!("\n=== Available Lighting Patterns ===");
    for (i, (pattern_name, description)) in LIGHTING_PATTERNS.iter().enumerate() {
        println!("  {}: {} - {}", i + 1, pattern_name, description);
    }

    // Flush CSV if enabled
    if let Some(ref mut writer) = csv_writer {
        writer.flush()?;
        println!(
            "\nLighting data exported to: {}",
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
                log::debug!("Destroyed vehicle: {vehicle_id}");
            }
            Err(e) => {
                cleanup_stats.record_operation(cleanup_timer.elapsed_ms(), false);
                log::warn!("Failed to destroy vehicle {vehicle_id}: {e}");
            }
        }
    }

    println!(
        "Cleanup success rate: {:.1}%",
        cleanup_stats.success_rate() * 100.0
    );

    println!("\nVehicle lighting demonstration completed!");

    // TODO: Notable missing features documented
    println!("\n=== Missing Features (TODO) ===");
    println!("1. Vehicle::set_light_state() FFI function");
    println!("2. VehicleLightState enum/struct definitions");
    println!("3. Light state query/get functions");
    println!("4. Individual light control (headlights, brake lights, etc.)");
    println!("5. Dynamic lighting effects and animations");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cli_args_compatibility() {
        let args = Args::try_parse_from(&[
            "vehicle_lighting",
            "--host",
            "localhost",
            "--port",
            "2000",
            "--vehicles",
            "5",
            "--cycle-duration",
            "2.0",
            "--total-duration",
            "30.0",
            "--vehicle-model",
            "vehicle.tesla.model3",
            "--spawn-location",
            "0.0,0.0,0.3",
            "--vehicle-spacing",
            "5.0",
        ])
        .unwrap();

        assert_eq!(args.common.host, "localhost");
        assert_eq!(args.common.port, 2000);
        assert_eq!(args.vehicles, 5);
        assert_eq!(args.cycle_duration, 2.0);
        assert_eq!(args.total_duration, 30.0);
        assert_eq!(args.vehicle_model, "vehicle.tesla.model3");
        assert_eq!(args.vehicle_spacing, 5.0);
    }

    #[test]
    fn test_parse_location() {
        let location = parse_location("1.0,2.0,3.0").unwrap();
        assert_eq!(location.x, 1.0);
        assert_eq!(location.y, 2.0);
        assert_eq!(location.z, 3.0);

        assert!(parse_location("invalid").is_err());
        assert!(parse_location("1,2").is_err());
    }

    #[test]
    fn test_lighting_patterns() {
        assert!(!LIGHTING_PATTERNS.is_empty());
        assert!(LIGHTING_PATTERNS.len() >= 10);

        // Check that patterns have names and descriptions
        for (name, description) in LIGHTING_PATTERNS {
            assert!(!name.is_empty());
            assert!(!description.is_empty());
        }
    }

    #[test]
    fn test_lighting_state_creation() {
        let state = LightingState {
            vehicle_id: "test_vehicle".to_string(),
            cycle_time: 2.0,
            light_pattern: "high_beam".to_string(),
            timestamp: std::time::Instant::now(),
        };

        assert_eq!(state.vehicle_id, "test_vehicle");
        assert_eq!(state.cycle_time, 2.0);
        assert_eq!(state.light_pattern, "high_beam");
    }
}
