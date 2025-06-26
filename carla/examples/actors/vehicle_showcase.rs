// Python equivalent: carla-simulator/PythonAPI/examples/vehicle_gallery.py
// Expected behavior: Automated vehicle gallery with camera rotation
// Key features: Blueprint enumeration, vehicle spawning, camera control

use anyhow::Result;
use carla::{
    actor::ActorExt,
    client::{ActorBlueprint, AttributeValue},
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
    about = "CARLA example: Vehicle blueprint showcase with automated camera"
)]
struct Args {
    #[command(flatten)]
    common: CommonArgs,

    #[arg(long, default_value_t = 5.0)]
    duration_per_vehicle: f64,

    #[arg(long, default_value_t = 2.5)]
    camera_distance: f64,

    #[arg(long, default_value_t = 0.8)]
    camera_height_factor: f64,

    #[arg(long, default_value_t = 60.0)]
    rotation_speed: f64,

    #[arg(long, default_value = "vehicle.*")]
    vehicle_filter: String,

    #[arg(long)]
    generation: Option<u32>,

    #[arg(long)]
    safe_only: bool,

    #[arg(long)]
    export_csv: Option<String>,

    #[arg(long, default_value = "-47.0,20.0,0.3")]
    spawn_location: String,

    #[arg(long, default_value_t = 45.0)]
    spawn_yaw: f64,
}

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

fn filter_vehicle_blueprints(
    blueprints: &[ActorBlueprint],
    generation: Option<u32>,
    safe_only: bool,
) -> Vec<ActorBlueprint> {
    let mut filtered: Vec<_> = blueprints.to_vec();

    // Filter by generation if specified
    if let Some(gen) = generation {
        filtered.retain(|bp| {
            if let Some(AttributeValue::String(gen_str)) = bp.attribute("generation") {
                if let Ok(bp_gen) = gen_str.parse::<u32>() {
                    bp_gen == gen
                } else {
                    false
                }
            } else {
                false
            }
        });
    }

    // Filter for safe vehicles only if requested
    if safe_only {
        filtered.retain(|bp| {
            if let Some(AttributeValue::String(type_str)) = bp.attribute("base_type") {
                type_str == "car"
            } else {
                false
            }
        });
    }

    // Sort by blueprint ID for consistent ordering
    filtered.sort_by_key(|a| a.id());

    filtered
}

fn calculate_camera_transform(
    vehicle_location: &Location,
    angle_degrees: f64,
    distance: f64,
    height_factor: f64,
    vehicle_bounding_box_extent_x: f64,
) -> Transform {
    let radius = vehicle_bounding_box_extent_x * distance;
    let height = vehicle_bounding_box_extent_x * height_factor;
    let angle_radians = angle_degrees.to_radians();

    let camera_location = Location {
        x: vehicle_location.x + radius * angle_radians.cos(),
        y: vehicle_location.y + radius * angle_radians.sin(),
        z: vehicle_location.z + height,
    };

    let camera_rotation = Rotation {
        pitch: -15.0,
        yaw: (180.0 + angle_degrees) as f32,
        roll: 0.0,
    };

    Transform {
        location: camera_location,
        rotation: camera_rotation,
    }
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

    println!("CARLA Vehicle Showcase Example");
    println!("Python equivalent: vehicle_gallery.py");
    println!("=====================================");

    // Parse spawn location
    let spawn_location = parse_location(&args.spawn_location)?;
    let spawn_transform = Transform {
        location: spawn_location,
        rotation: Rotation {
            pitch: 0.0,
            yaw: -args.spawn_yaw as f32,
            roll: 0.0,
        },
    };

    // Connect to CARLA
    let client = connect_with_retry(&args.common.host, args.common.port, args.common.timeout)?;
    let world = client.world()?;

    // Get blueprint library and filter vehicles
    let blueprint_library = world.blueprint_library()?;
    let vehicle_blueprints = blueprint_library.filter(&args.vehicle_filter)?;
    let filtered_vehicles =
        filter_vehicle_blueprints(&vehicle_blueprints, args.generation, args.safe_only);

    if filtered_vehicles.is_empty() {
        anyhow::bail!(
            "No vehicle blueprints found with filter: {}",
            args.vehicle_filter
        );
    }

    println!(
        "Found {} vehicle blueprints to showcase",
        filtered_vehicles.len()
    );

    // Setup CSV export if requested
    let mut csv_writer = if let Some(csv_path) = &args.export_csv {
        let mut writer = CsvWriter::new(csv_path)?;
        writer.write_header(&[
            "blueprint_id",
            "generation",
            "base_type",
            "brand",
            "model",
            "number_of_wheels",
            "role_name",
            "showcase_duration_ms",
            "rotation_frames",
        ])?;
        Some(writer)
    } else {
        None
    };

    // TODO: Get spectator for camera control
    // This requires World::get_spectator() FFI function
    println!("TODO: Spectator camera control not implemented");
    println!("This requires World::get_spectator() and Spectator::set_transform() FFI functions");

    // Performance tracking
    let mut showcase_stats = PerformanceStats::new();
    let mut blueprint_histogram = Histogram::new();
    let mut progress = ProgressTracker::new(filtered_vehicles.len() as u64, 1);

    // Showcase each vehicle
    for (index, blueprint) in filtered_vehicles.iter().enumerate() {
        let showcase_timer = Timer::new();
        println!(
            "\n=== Vehicle {}/{}: {} ===",
            index + 1,
            filtered_vehicles.len(),
            blueprint.id()
        );

        // Extract blueprint attributes
        let generation = blueprint
            .attribute("generation")
            .and_then(|attr| {
                if let AttributeValue::String(s) = attr {
                    Some(s)
                } else {
                    None
                }
            })
            .unwrap_or_else(|| "unknown".to_string());

        let base_type = blueprint
            .attribute("base_type")
            .and_then(|attr| {
                if let AttributeValue::String(s) = attr {
                    Some(s)
                } else {
                    None
                }
            })
            .unwrap_or_else(|| "unknown".to_string());

        let brand = blueprint
            .attribute("brand")
            .and_then(|attr| {
                if let AttributeValue::String(s) = attr {
                    Some(s)
                } else {
                    None
                }
            })
            .unwrap_or_else(|| "unknown".to_string());

        let model = blueprint
            .attribute("model")
            .and_then(|attr| {
                if let AttributeValue::String(s) = attr {
                    Some(s)
                } else {
                    None
                }
            })
            .unwrap_or_else(|| "unknown".to_string());

        let number_of_wheels = blueprint
            .attribute("number_of_wheels")
            .and_then(|attr| {
                if let AttributeValue::String(s) = attr {
                    Some(s)
                } else {
                    None
                }
            })
            .unwrap_or_else(|| "unknown".to_string());

        println!("  Brand: {}", brand);
        println!("  Model: {}", model);
        println!("  Generation: {}", generation);
        println!("  Base Type: {}", base_type);
        println!("  Wheels: {}", number_of_wheels);

        // Spawn vehicle
        match world.try_spawn_actor(blueprint, &spawn_transform, None) {
            Ok(Some(mut vehicle)) => {
                println!("  ✓ Vehicle spawned successfully");

                // TODO: Get vehicle bounding box for camera positioning
                // This requires Actor::bounding_box() FFI function
                let estimated_bounding_box_extent_x = 2.5; // Estimate for demo

                // Calculate rotation parameters
                let duration_ms = (args.duration_per_vehicle * 1000.0) as u64;
                let _rotation_step = args.rotation_speed * (duration_ms as f64 / 1000.0) / 360.0;
                let frame_time_ms = 50; // 20 FPS
                let total_frames = duration_ms / frame_time_ms;
                let angle_step = 360.0 / total_frames as f64;

                println!(
                    "  Showcasing for {:.1}s with {:.1}°/frame rotation",
                    args.duration_per_vehicle, angle_step
                );

                // Perform camera rotation showcase
                let mut rotation_frames = 0;
                for frame in 0..total_frames {
                    let angle = (frame as f64) * angle_step;

                    // Calculate camera position
                    let camera_transform = calculate_camera_transform(
                        &spawn_location,
                        angle,
                        args.camera_distance,
                        args.camera_height_factor,
                        estimated_bounding_box_extent_x,
                    );

                    // TODO: Set spectator transform
                    // spectator.set_transform(&camera_transform)?;
                    if frame == 0 {
                        log::debug!(
                            "  Camera at angle {:.1}°: x={:.2}, y={:.2}, z={:.2}",
                            angle,
                            camera_transform.location.x,
                            camera_transform.location.y,
                            camera_transform.location.z
                        );
                    }

                    rotation_frames += 1;
                    thread::sleep(Duration::from_millis(frame_time_ms));
                }

                // Export to CSV if enabled
                if let Some(ref mut writer) = csv_writer {
                    writer.write_row(&[
                        blueprint.id(),
                        generation.clone(),
                        base_type.clone(),
                        brand.clone(),
                        model.clone(),
                        number_of_wheels.clone(),
                        "showcase".to_string(),
                        showcase_timer.elapsed_ms().to_string(),
                        rotation_frames.to_string(),
                    ])?;
                }

                // Clean up vehicle
                match vehicle.destroy() {
                    Ok(_) => {
                        log::debug!("  ✓ Vehicle destroyed");
                    }
                    Err(e) => {
                        log::warn!("  ⚠ Failed to destroy vehicle: {}", e);
                    }
                }

                showcase_stats.record_operation(showcase_timer.elapsed_ms(), true);
                blueprint_histogram.add(base_type);

                println!(
                    "  ✓ Showcase completed in {:.1}s",
                    showcase_timer.elapsed_ms() as f64 / 1000.0
                );
            }
            Ok(None) => {
                log::warn!("  ✗ Failed to spawn vehicle - spawn point occupied");
                showcase_stats.record_operation(showcase_timer.elapsed_ms(), false);
            }
            Err(e) => {
                log::warn!("  ✗ Failed to spawn vehicle: {}", e);
                showcase_stats.record_operation(showcase_timer.elapsed_ms(), false);
            }
        }

        progress.update(index as u64 + 1);

        // Small delay between vehicles
        thread::sleep(Duration::from_millis(500));
    }

    // Print showcase statistics
    println!("\n=== Showcase Statistics ===");
    println!("Total vehicles processed: {}", filtered_vehicles.len());
    println!(
        "Successfully showcased: {}",
        showcase_stats.successful_operations
    );
    println!("Failed to showcase: {}", showcase_stats.failed_operations);
    println!(
        "Success rate: {:.1}%",
        showcase_stats.success_rate() * 100.0
    );
    println!(
        "Average showcase time: {:.1}s",
        showcase_stats.average_duration_ms() / 1000.0
    );

    if let Some(min) = showcase_stats.min_duration_ms {
        println!("Fastest showcase: {:.1}s", min as f64 / 1000.0);
    }
    if let Some(max) = showcase_stats.max_duration_ms {
        println!("Slowest showcase: {:.1}s", max as f64 / 1000.0);
    }

    // Print vehicle type histogram
    println!("\n=== Vehicle Types Showcased ===");
    blueprint_histogram.print();

    // Flush CSV if enabled
    if let Some(ref mut writer) = csv_writer {
        writer.flush()?;
        println!(
            "\nShowcase data exported to: {}",
            args.export_csv.as_ref().unwrap()
        );
    }

    println!("\nVehicle showcase completed successfully!");

    // TODO: Notable missing features documented
    println!("\n=== Missing Features (TODO) ===");
    println!("1. Spectator camera control (World::get_spectator, Spectator::set_transform)");
    println!("2. Vehicle bounding box access (Actor::bounding_box)");
    println!("3. Real-time camera movement synchronization");
    println!("4. Screenshot capture functionality");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cli_args_match_python() {
        // Test that CLI arguments are compatible
        let args = Args::try_parse_from(&[
            "vehicle_showcase",
            "--host",
            "localhost",
            "--port",
            "2000",
            "--duration-per-vehicle",
            "5.0",
            "--camera-distance",
            "2.5",
            "--rotation-speed",
            "60.0",
            "--vehicle-filter",
            "vehicle.*",
            "--generation",
            "3",
            "--safe-only",
            "--spawn-location",
            "-47.0,20.0,0.3",
            "--spawn-yaw",
            "45.0",
        ])
        .unwrap();

        assert_eq!(args.common.host, "localhost");
        assert_eq!(args.common.port, 2000);
        assert_eq!(args.duration_per_vehicle, 5.0);
        assert_eq!(args.camera_distance, 2.5);
        assert_eq!(args.rotation_speed, 60.0);
        assert_eq!(args.generation, Some(3));
        assert!(args.safe_only);
        assert_eq!(args.spawn_yaw, 45.0);
    }

    #[test]
    fn test_parse_location() {
        let location = parse_location("-47.0,20.0,0.3").unwrap();
        assert_eq!(location.x, -47.0);
        assert_eq!(location.y, 20.0);
        assert_eq!(location.z, 0.3);

        // Test error cases
        assert!(parse_location("invalid").is_err());
        assert!(parse_location("1,2").is_err());
        assert!(parse_location("a,b,c").is_err());
    }

    #[test]
    fn test_camera_transform_calculation() {
        let vehicle_location = Location {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let transform = calculate_camera_transform(&vehicle_location, 0.0, 2.5, 0.8, 2.0);

        // At 0 degrees, camera should be at positive X
        assert!((transform.location.x - 5.0).abs() < 0.01); // 2.0 * 2.5 = 5.0
        assert!(transform.location.y.abs() < 0.01);
        assert!((transform.location.z - 1.6).abs() < 0.01); // 2.0 * 0.8 = 1.6
        assert_eq!(transform.rotation.yaw, 180.0);
        assert_eq!(transform.rotation.pitch, -15.0);
    }

    #[test]
    fn test_filter_vehicle_blueprints() {
        // This would be tested with actual blueprint data in integration tests
        let blueprints = vec![];
        let filtered = filter_vehicle_blueprints(&blueprints, None, false);
        assert_eq!(filtered.len(), 0);
    }
}
