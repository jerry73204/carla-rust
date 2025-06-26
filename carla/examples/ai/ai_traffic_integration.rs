// Python equivalent: carla-simulator/PythonAPI/examples/traffic_manager_example.py
// Expected behavior: Traffic Manager integration with AI-driven vehicle behavior
// Key features: Traffic Manager control, autopilot, collision detection, traffic rules

use anyhow::Result;
use carla::{
    actor::ActorExt,
    geom::{Location, Rotation, Transform},
};
use clap::Parser;
use std::{
    collections::HashMap,
    thread,
    time::{Duration, Instant},
};

#[path = "../common/mod.rs"]
mod common;
use common::{cli::CommonArgs, connect_with_retry, utils::*};

// Type aliases to reduce complexity
type ActorList = Vec<Box<dyn ActorExt>>;
type BehaviorList = Vec<VehicleBehavior>;
type EventList = Vec<TrafficEvent>;

#[derive(Parser, Debug)]
#[command(
    author,
    version,
    about = "CARLA example: AI Traffic Manager integration and automated behavior"
)]
struct Args {
    #[command(flatten)]
    common: CommonArgs,

    #[arg(long, default_value_t = 50)]
    vehicles: u32,

    #[arg(long, default_value_t = 20)]
    walkers: u32,

    #[arg(long, default_value_t = 60.0)]
    simulation_duration: f64,

    #[arg(long, default_value_t = 8011)]
    tm_port: u16,

    #[arg(long)]
    export_csv: Option<String>,

    #[arg(long)]
    enable_autopilot: bool,

    #[arg(long)]
    ignore_traffic_lights: bool,

    #[arg(long)]
    ignore_signs: bool,

    #[arg(long)]
    ignore_vehicles: bool,

    #[arg(long)]
    ignore_walkers: bool,

    #[arg(long, default_value_t = 30.0)]
    target_speed: f64, // km/h

    #[arg(long, default_value_t = 2.0)]
    vehicle_spacing: f64,

    #[arg(long)]
    seed: Option<u64>,

    #[arg(long)]
    no_rendering_mode: bool,

    #[arg(long, default_value_t = 10.0)]
    walker_speed: f64,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
struct TrafficManagerSettings {
    port: u16,
    ignore_traffic_lights: bool,
    ignore_signs: bool,
    ignore_vehicles: bool,
    ignore_walkers: bool,
    target_speed: f64,
    vehicle_spacing: f64,
    auto_lane_change: bool,
    distance_to_leading_vehicle: f64,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
struct VehicleBehavior {
    actor_id: String,
    autopilot_enabled: bool,
    target_speed: f64,
    aggressive_behavior: f64, // 0.0 to 1.0
    lane_change_frequency: f64,
    collision_detection: bool,
    traffic_light_compliance: bool,
}

#[derive(Debug, Clone)]
struct TrafficEvent {
    timestamp: f64,
    event_type: String,
    actor_id: String,
    location: Location,
    description: String,
    severity: String, // "info", "warning", "critical"
}

#[derive(Debug, Clone)]
struct AITrafficStats {
    total_vehicles: usize,
    autopilot_vehicles: usize,
    total_walkers: usize,
    collision_count: usize,
    traffic_violations: usize,
    avg_speed: f64,
    distance_traveled: f64,
    ai_decisions: usize,
}

impl Default for TrafficManagerSettings {
    fn default() -> Self {
        Self {
            port: 8011,
            ignore_traffic_lights: false,
            ignore_signs: false,
            ignore_vehicles: false,
            ignore_walkers: false,
            target_speed: 30.0,
            vehicle_spacing: 2.0,
            auto_lane_change: true,
            distance_to_leading_vehicle: 5.0,
        }
    }
}

fn create_traffic_manager_settings(args: &Args) -> TrafficManagerSettings {
    TrafficManagerSettings {
        port: args.tm_port,
        ignore_traffic_lights: args.ignore_traffic_lights,
        ignore_signs: args.ignore_signs,
        ignore_vehicles: args.ignore_vehicles,
        ignore_walkers: args.ignore_walkers,
        target_speed: args.target_speed,
        vehicle_spacing: args.vehicle_spacing,
        auto_lane_change: true,
        distance_to_leading_vehicle: args.vehicle_spacing * 2.0,
    }
}

fn configure_traffic_manager(
    _world: &carla::client::World,
    settings: &TrafficManagerSettings,
) -> Result<()> {
    // TODO: Traffic Manager configuration not implemented
    println!("TODO: Traffic Manager configuration not implemented");
    println!("This requires:");
    println!("  - Client::get_trafficmanager(port) FFI function");
    println!("  - TrafficManager::set_global_distance_to_leading_vehicle() FFI function");
    println!("  - TrafficManager::set_respawn_dormant_vehicles() FFI function");
    println!("  - TrafficManager::set_boundaries_respawn_dormant_vehicles() FFI function");
    println!("  - TrafficManager::global_percentage_speed_difference() FFI function");
    println!("  - TrafficManager::collision_detection() FFI function");
    println!("  - TrafficManager::ignore_lights_percentage() FFI function");
    println!("  - TrafficManager::ignore_signs_percentage() FFI function");
    println!("  - TrafficManager::ignore_vehicles_percentage() FFI function");
    println!("  - TrafficManager::ignore_walkers_percentage() FFI function");

    println!("\nSimulated Traffic Manager configuration:");
    println!("  Port: {}", settings.port);
    println!("  Target speed: {:.1} km/h", settings.target_speed);
    println!("  Vehicle spacing: {:.1}m", settings.vehicle_spacing);
    println!(
        "  Ignore traffic lights: {}",
        settings.ignore_traffic_lights
    );
    println!("  Ignore signs: {}", settings.ignore_signs);
    println!("  Ignore vehicles: {}", settings.ignore_vehicles);
    println!("  Ignore walkers: {}", settings.ignore_walkers);

    Ok(())
}

fn spawn_ai_vehicles(
    world: &carla::client::World,
    vehicle_count: u32,
    enable_autopilot: bool,
) -> Result<(ActorList, BehaviorList)> {
    let mut vehicles = Vec::new();
    let mut behaviors = Vec::new();

    println!("Spawning {} AI-controlled vehicles...", vehicle_count);

    // Get blueprint library and spawn points
    let blueprint_library = world.blueprint_library()?;
    let vehicle_blueprints = blueprint_library.filter("vehicle.*")?;

    // TODO: Get spawn points from map
    // This requires Map::get_spawn_points() FFI function
    println!("TODO: Map spawn points not implemented, using simulated positions");

    for i in 0..vehicle_count {
        if let Some(blueprint) = vehicle_blueprints.get(i as usize % vehicle_blueprints.len()) {
            // Create spawn transform (simulated)
            let spawn_transform = Transform {
                location: Location {
                    x: (i as f64) * 10.0,
                    y: (i % 4) as f64 * 4.0,
                    z: 0.3,
                },
                rotation: Rotation {
                    pitch: 0.0,
                    yaw: 0.0,
                    roll: 0.0,
                },
            };

            match world.try_spawn_actor(blueprint, &spawn_transform, None) {
                Ok(Some(vehicle)) => {
                    let actor_id = format!("ai_vehicle_{:03}", i);

                    // TODO: Enable autopilot
                    if enable_autopilot {
                        // vehicle.set_autopilot(true, tm_port)?;
                        println!("  TODO: vehicle.set_autopilot() not implemented");
                    }

                    // Create behavior profile
                    let behavior = VehicleBehavior {
                        actor_id: actor_id.clone(),
                        autopilot_enabled: enable_autopilot,
                        target_speed: 20.0 + (i % 20) as f64, // Vary speeds
                        aggressive_behavior: (i % 100) as f64 / 100.0, // 0-1 range
                        lane_change_frequency: 0.1 + (i % 30) as f64 / 300.0,
                        collision_detection: true,
                        traffic_light_compliance: (i % 10) != 0, // 10% non-compliant
                    };

                    vehicles.push(Box::new(vehicle) as Box<dyn ActorExt>);
                    behaviors.push(behavior);

                    if i < 5 {
                        println!(
                            "  ✓ Spawned AI vehicle {} at x={:.1}, autopilot={}",
                            i, spawn_transform.location.x, enable_autopilot
                        );
                    }
                }
                Ok(None) => {
                    log::warn!("Failed to spawn vehicle {} - spawn point occupied", i);
                }
                Err(e) => {
                    log::warn!("Failed to spawn vehicle {}: {}", i, e);
                }
            }
        }
    }

    println!("Successfully spawned {} AI vehicles", vehicles.len());
    Ok((vehicles, behaviors))
}

fn spawn_ai_walkers(
    _world: &carla::client::World,
    walker_count: u32,
    walker_speed: f64,
) -> Result<ActorList> {
    let walkers = Vec::new();

    println!("Spawning {} AI-controlled walkers...", walker_count);

    // TODO: Walker spawning and AI controller not implemented
    println!("TODO: Walker spawning not implemented");
    println!("This requires:");
    println!("  - Walker blueprint spawning");
    println!("  - WalkerAIController spawning and attachment");
    println!("  - Walker::start_walker_ai_controller() FFI function");
    println!("  - WalkerAIController::go_to_location() FFI function");
    println!("  - WalkerAIController::set_max_speed() FFI function");

    // Simulate walker spawning for demonstration
    for i in 0..walker_count {
        println!("  Simulated walker {} at speed {:.1} m/s", i, walker_speed);
    }

    Ok(walkers)
}

fn simulate_ai_traffic_behavior(
    vehicles: &[Box<dyn ActorExt>],
    behaviors: &[VehicleBehavior],
    duration: f64,
) -> Result<(EventList, AITrafficStats)> {
    println!("\n=== AI Traffic Simulation ===");
    println!("Running AI traffic for {:.1}s...", duration);

    let mut events = Vec::new();
    let mut stats = AITrafficStats {
        total_vehicles: vehicles.len(),
        autopilot_vehicles: behaviors.iter().filter(|b| b.autopilot_enabled).count(),
        total_walkers: 0, // Not implemented
        collision_count: 0,
        traffic_violations: 0,
        avg_speed: 0.0,
        distance_traveled: 0.0,
        ai_decisions: 0,
    };

    let start_time = Instant::now();
    let frame_duration = Duration::from_millis(100); // 10 FPS for AI decisions
    let total_frames = (duration * 10.0) as u64;
    let mut progress = ProgressTracker::new(total_frames, total_frames / 20);

    for frame in 0..total_frames {
        let frame_start = Instant::now();
        let simulation_time = start_time.elapsed().as_secs_f64();

        // Simulate AI decision making for each vehicle
        for (i, (vehicle, behavior)) in vehicles.iter().zip(behaviors.iter()).enumerate() {
            // TODO: Get actual vehicle state
            // let vehicle_state = vehicle.get_physics_control()?;
            // let current_location = vehicle.transform().location;
            // let velocity = vehicle.get_velocity()?;

            // Simulate AI decision events
            if frame % (20 + i as u64 * 3) == 0 {
                // Varying decision frequency
                stats.ai_decisions += 1;

                events.push(TrafficEvent {
                    timestamp: simulation_time,
                    event_type: "ai_decision".to_string(),
                    actor_id: behavior.actor_id.clone(),
                    location: vehicle.transform().location,
                    description: format!(
                        "AI lane change decision (aggression: {:.2})",
                        behavior.aggressive_behavior
                    ),
                    severity: "info".to_string(),
                });
            }

            // Simulate traffic violations
            if !behavior.traffic_light_compliance && frame % (200 + i as u64 * 17) == 0 {
                stats.traffic_violations += 1;

                events.push(TrafficEvent {
                    timestamp: simulation_time,
                    event_type: "traffic_violation".to_string(),
                    actor_id: behavior.actor_id.clone(),
                    location: vehicle.transform().location,
                    description: "Ran red traffic light".to_string(),
                    severity: "warning".to_string(),
                });
            }

            // Simulate potential collisions
            if behavior.aggressive_behavior > 0.7 && frame % (300 + i as u64 * 23) == 0 {
                stats.collision_count += 1;

                events.push(TrafficEvent {
                    timestamp: simulation_time,
                    event_type: "collision".to_string(),
                    actor_id: behavior.actor_id.clone(),
                    location: vehicle.transform().location,
                    description: format!(
                        "Near collision due to aggressive behavior ({:.2})",
                        behavior.aggressive_behavior
                    ),
                    severity: "critical".to_string(),
                });
            }

            // Simulate speed adjustments
            if frame % (50 + i as u64 * 7) == 0 {
                let target_speed = behavior.target_speed + (simulation_time.sin() * 5.0);

                // TODO: Apply speed control
                // vehicle.apply_control(VehicleControl {
                //     throttle: if target_speed > current_speed { 0.7 } else { 0.0 },
                //     brake: if target_speed < current_speed { 0.3 } else { 0.0 },
                //     ...
                // })?;

                events.push(TrafficEvent {
                    timestamp: simulation_time,
                    event_type: "speed_adjustment".to_string(),
                    actor_id: behavior.actor_id.clone(),
                    location: vehicle.transform().location,
                    description: format!("Target speed adjusted to {:.1} km/h", target_speed),
                    severity: "info".to_string(),
                });
            }
        }

        progress.update(frame + 1);

        // Maintain frame rate
        let elapsed = frame_start.elapsed();
        if elapsed < frame_duration {
            thread::sleep(frame_duration - elapsed);
        }
    }

    // Calculate final statistics
    stats.avg_speed =
        behaviors.iter().map(|b| b.target_speed).sum::<f64>() / behaviors.len() as f64;
    stats.distance_traveled = stats.avg_speed * duration / 3.6; // Convert km/h to m/s

    println!("AI simulation completed with {} events", events.len());
    Ok((events, stats))
}

fn analyze_traffic_performance(
    events: &[TrafficEvent],
    stats: &AITrafficStats,
) -> HashMap<String, f64> {
    let mut metrics = HashMap::new();

    // Calculate event frequencies
    let total_time = events
        .iter()
        .map(|e| e.timestamp)
        .fold(0.0f64, |max, t| max.max(t));

    metrics.insert("total_events".to_string(), events.len() as f64);
    metrics.insert(
        "events_per_second".to_string(),
        events.len() as f64 / total_time,
    );
    metrics.insert(
        "collision_rate".to_string(),
        stats.collision_count as f64 / stats.total_vehicles as f64,
    );
    metrics.insert(
        "violation_rate".to_string(),
        stats.traffic_violations as f64 / stats.total_vehicles as f64,
    );
    metrics.insert(
        "ai_decisions_per_vehicle".to_string(),
        stats.ai_decisions as f64 / stats.total_vehicles as f64,
    );
    metrics.insert("avg_speed_kmh".to_string(), stats.avg_speed);

    // Count events by severity
    let critical_events = events.iter().filter(|e| e.severity == "critical").count();
    let warning_events = events.iter().filter(|e| e.severity == "warning").count();
    let info_events = events.iter().filter(|e| e.severity == "info").count();

    metrics.insert(
        "critical_event_rate".to_string(),
        critical_events as f64 / events.len() as f64,
    );
    metrics.insert(
        "warning_event_rate".to_string(),
        warning_events as f64 / events.len() as f64,
    );
    metrics.insert(
        "info_event_rate".to_string(),
        info_events as f64 / events.len() as f64,
    );

    metrics
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

    println!("CARLA AI Traffic Integration Example");
    println!("Python equivalent: traffic_manager_example.py");
    println!("==========================================");

    // Set random seed if provided
    if let Some(seed) = args.seed {
        println!("Using random seed: {}", seed);
        // TODO: Set CARLA random seed
        // world.set_random_seed(seed)?;
    }

    // Connect to CARLA
    let client = connect_with_retry(&args.common.host, args.common.port, args.common.timeout)?;
    let world = client.world()?;

    // Setup CSV export if requested
    let mut csv_writer = if let Some(csv_path) = &args.export_csv {
        let mut writer = CsvWriter::new(csv_path)?;
        writer.write_header(&[
            "timestamp",
            "event_type",
            "actor_id",
            "severity",
            "x",
            "y",
            "z",
            "description",
        ])?;
        Some(writer)
    } else {
        None
    };

    // Configure Traffic Manager
    let tm_settings = create_traffic_manager_settings(&args);
    configure_traffic_manager(&world, &tm_settings)?;

    // TODO: Configure rendering mode
    if args.no_rendering_mode {
        println!("TODO: No rendering mode not implemented");
        println!("This requires World::set_rendering_mode(false) or similar");
    }

    // Spawn AI vehicles
    let (vehicles, behaviors) = spawn_ai_vehicles(&world, args.vehicles, args.enable_autopilot)?;

    // Spawn AI walkers
    let walkers = spawn_ai_walkers(&world, args.walkers, args.walker_speed)?;

    // Run AI traffic simulation
    let simulation_timer = Timer::new();
    let (events, stats) =
        simulate_ai_traffic_behavior(&vehicles, &behaviors, args.simulation_duration)?;
    let simulation_time = simulation_timer.elapsed_ms();

    // Export events to CSV if enabled
    if let Some(ref mut writer) = csv_writer {
        for event in &events {
            writer.write_row(&[
                event.timestamp.to_string(),
                event.event_type.clone(),
                event.actor_id.clone(),
                event.severity.clone(),
                event.location.x.to_string(),
                event.location.y.to_string(),
                event.location.z.to_string(),
                event.description.clone(),
            ])?;
        }
        writer.flush()?;
        println!(
            "AI traffic events exported to: {}",
            args.export_csv.as_ref().unwrap()
        );
    }

    // Analyze performance
    let performance_metrics = analyze_traffic_performance(&events, &stats);

    // Print comprehensive statistics
    println!("\n=== AI Traffic Statistics ===");
    println!(
        "Simulation duration: {:.1}s (actual: {:.1}s)",
        args.simulation_duration,
        simulation_time as f64 / 1000.0
    );
    println!("Total vehicles: {}", stats.total_vehicles);
    println!("Autopilot vehicles: {}", stats.autopilot_vehicles);
    println!("Total walkers: {}", stats.total_walkers);
    println!("Total events: {}", events.len());
    println!("AI decisions made: {}", stats.ai_decisions);
    println!("Collisions detected: {}", stats.collision_count);
    println!("Traffic violations: {}", stats.traffic_violations);
    println!("Average speed: {:.1} km/h", stats.avg_speed);
    println!("Distance traveled: {:.1}m", stats.distance_traveled);

    println!("\n=== Performance Metrics ===");
    for (metric, value) in &performance_metrics {
        println!("  {}: {:.3}", metric, value);
    }

    // Event analysis by type
    let mut event_histogram = Histogram::new();
    let mut severity_histogram = Histogram::new();
    for event in &events {
        event_histogram.add(event.event_type.clone());
        severity_histogram.add(event.severity.clone());
    }

    println!("\n=== Event Type Distribution ===");
    event_histogram.print();

    println!("\n=== Event Severity Distribution ===");
    severity_histogram.print();

    // Print sample critical events
    let critical_events: Vec<_> = events
        .iter()
        .filter(|e| e.severity == "critical")
        .take(5)
        .collect();

    if !critical_events.is_empty() {
        println!("\n=== Sample Critical Events ===");
        for event in critical_events {
            println!(
                "  {:.2}s: {} - {}",
                event.timestamp, event.actor_id, event.description
            );
        }
    }

    // Cleanup
    println!("\n=== Cleanup ===");
    let mut cleanup_stats = PerformanceStats::new();

    // Cleanup vehicles
    for (i, mut vehicle) in vehicles.into_iter().enumerate() {
        let cleanup_timer = Timer::new();

        // TODO: Disable autopilot before destroying
        // vehicle.set_autopilot(false)?;

        match vehicle.destroy() {
            Ok(_) => {
                cleanup_stats.record_operation(cleanup_timer.elapsed_ms(), true);
                log::debug!("Destroyed AI vehicle {}", i);
            }
            Err(e) => {
                cleanup_stats.record_operation(cleanup_timer.elapsed_ms(), false);
                log::warn!("Failed to destroy AI vehicle {}: {}", i, e);
            }
        }
    }

    // Cleanup walkers
    for (i, mut walker) in walkers.into_iter().enumerate() {
        let cleanup_timer = Timer::new();
        match walker.destroy() {
            Ok(_) => {
                cleanup_stats.record_operation(cleanup_timer.elapsed_ms(), true);
                log::debug!("Destroyed AI walker {}", i);
            }
            Err(e) => {
                cleanup_stats.record_operation(cleanup_timer.elapsed_ms(), false);
                log::warn!("Failed to destroy AI walker {}: {}", i, e);
            }
        }
    }

    println!(
        "Cleanup success rate: {:.1}%",
        cleanup_stats.success_rate() * 100.0
    );

    println!("\nAI traffic integration demonstration completed!");

    // TODO: Notable missing features documented
    println!("\n=== Missing Features (TODO) ===");
    println!("1. Client::get_trafficmanager(port) FFI function");
    println!(
        "2. TrafficManager configuration FFI functions (speed, distance, collision detection)"
    );
    println!("3. Vehicle::set_autopilot(enabled, port) FFI function");
    println!("4. Map::get_spawn_points() for proper vehicle positioning");
    println!("5. Walker spawning and WalkerAIController attachment");
    println!("6. Vehicle physics and control state access functions");
    println!("7. Collision detection and event callbacks");
    println!("8. Traffic light and sign state query functions");
    println!("9. World rendering mode control");
    println!("10. Random seed setting for reproducible AI behavior");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cli_args_compatibility() {
        let args = Args::try_parse_from(&[
            "ai_traffic_integration",
            "--host",
            "localhost",
            "--port",
            "2000",
            "--vehicles",
            "50",
            "--walkers",
            "20",
            "--simulation-duration",
            "60.0",
            "--tm-port",
            "8011",
            "--enable-autopilot",
            "--ignore-traffic-lights",
            "--target-speed",
            "40.0",
            "--vehicle-spacing",
            "3.0",
            "--seed",
            "12345",
            "--no-rendering-mode",
        ])
        .unwrap();

        assert_eq!(args.common.host, "localhost");
        assert_eq!(args.common.port, 2000);
        assert_eq!(args.vehicles, 50);
        assert_eq!(args.walkers, 20);
        assert_eq!(args.simulation_duration, 60.0);
        assert_eq!(args.tm_port, 8011);
        assert!(args.enable_autopilot);
        assert!(args.ignore_traffic_lights);
        assert_eq!(args.target_speed, 40.0);
        assert_eq!(args.seed, Some(12345));
        assert!(args.no_rendering_mode);
    }

    #[test]
    fn test_traffic_manager_settings() {
        let args = Args {
            common: CommonArgs::default(),
            vehicles: 10,
            walkers: 5,
            simulation_duration: 30.0,
            tm_port: 8012,
            export_csv: None,
            enable_autopilot: true,
            ignore_traffic_lights: true,
            ignore_signs: false,
            ignore_vehicles: false,
            ignore_walkers: true,
            target_speed: 50.0,
            vehicle_spacing: 3.0,
            seed: None,
            no_rendering_mode: false,
            walker_speed: 1.5,
        };

        let settings = create_traffic_manager_settings(&args);
        assert_eq!(settings.port, 8012);
        assert!(settings.ignore_traffic_lights);
        assert!(!settings.ignore_signs);
        assert!(settings.ignore_walkers);
        assert_eq!(settings.target_speed, 50.0);
        assert_eq!(settings.vehicle_spacing, 3.0);
    }

    #[test]
    fn test_vehicle_behavior_creation() {
        let behavior = VehicleBehavior {
            actor_id: "test_vehicle".to_string(),
            autopilot_enabled: true,
            target_speed: 30.0,
            aggressive_behavior: 0.5,
            lane_change_frequency: 0.2,
            collision_detection: true,
            traffic_light_compliance: false,
        };

        assert_eq!(behavior.actor_id, "test_vehicle");
        assert!(behavior.autopilot_enabled);
        assert_eq!(behavior.target_speed, 30.0);
        assert_eq!(behavior.aggressive_behavior, 0.5);
        assert!(!behavior.traffic_light_compliance);
    }

    #[test]
    fn test_traffic_event_creation() {
        let event = TrafficEvent {
            timestamp: 1.5,
            event_type: "collision".to_string(),
            actor_id: "vehicle_001".to_string(),
            location: Location {
                x: 100.0,
                y: 50.0,
                z: 0.5,
            },
            description: "Near collision with pedestrian".to_string(),
            severity: "critical".to_string(),
        };

        assert_eq!(event.timestamp, 1.5);
        assert_eq!(event.event_type, "collision");
        assert_eq!(event.severity, "critical");
    }

    #[test]
    fn test_ai_traffic_stats() {
        let stats = AITrafficStats {
            total_vehicles: 50,
            autopilot_vehicles: 45,
            total_walkers: 20,
            collision_count: 3,
            traffic_violations: 7,
            avg_speed: 35.5,
            distance_traveled: 1500.0,
            ai_decisions: 250,
        };

        assert_eq!(stats.total_vehicles, 50);
        assert_eq!(stats.autopilot_vehicles, 45);
        assert_eq!(stats.collision_count, 3);
        assert_eq!(stats.ai_decisions, 250);
    }
}
