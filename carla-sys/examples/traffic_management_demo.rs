//! Traffic management system demonstration for CARLA simulator.
//!
//! This example demonstrates:
//! - Creating and configuring traffic manager
//! - Registering vehicles with traffic manager
//! - Setting global and vehicle-specific behavior
//! - Monitoring traffic manager statistics
//! - Synchronous mode operation

use anyhow::Result;
use carla_sys::{ClientWrapper, SimpleTransform, TrafficManagerWrapper};
use std::time::Duration;

fn main() -> Result<()> {
    println!("=== CARLA Traffic Management Demo ===\n");

    // Connect to CARLA server
    println!("Connecting to CARLA server...");
    let mut client = ClientWrapper::new("localhost", 2000)?;
    client.set_timeout(Duration::from_secs(10));

    // Get the world
    let world = client.get_world();
    let blueprint_library = world.get_blueprint_library();

    // Get map spawn points
    let map = world.get_map();
    let spawn_points = map.get_recommended_spawn_points();
    println!("Found {} spawn points", spawn_points.len());

    if spawn_points.len() < 5 {
        println!("Not enough spawn points for demo. Need at least 5.");
        return Ok(());
    }

    // Create traffic manager
    println!("\n--- Traffic Manager Setup ---");
    let traffic_manager = TrafficManagerWrapper::get_instance(client.get_client(), 8000);
    println!(
        "Traffic Manager created on port: {}",
        traffic_manager.get_port()
    );

    // Configure traffic manager for demo
    configure_traffic_manager(&traffic_manager)?;

    // Spawn some vehicles
    println!("\n--- Vehicle Spawning ---");
    let vehicles = spawn_demo_vehicles(&world, &blueprint_library, &spawn_points, 5)?;
    println!("Spawned {} vehicles", vehicles.len());

    // Register vehicles with traffic manager
    println!("\n--- Vehicle Registration ---");
    println!("Vehicle registration would require:");
    println!("  - C++ bridge functions implementation");
    println!("  - Proper Vehicle casting from Actor");
    println!(
        "  - For now, simulating registration of {} vehicles",
        vehicles.len()
    );

    for i in 0..vehicles.len() {
        println!("  Vehicle {} would be registered", i + 1);
    }

    // Configure individual vehicles
    println!("\n--- Individual Vehicle Configuration ---");
    configure_demo_vehicles(&traffic_manager, &vehicles)?;

    // Demonstrate synchronous mode
    println!("\n--- Synchronous Mode Demo ---");
    demonstrate_synchronous_mode(&traffic_manager, &world)?;

    // Monitor traffic for a while
    println!("\n--- Traffic Monitoring ---");
    monitor_traffic(&traffic_manager, 20)?;

    // Show final statistics
    println!("\n--- Final Statistics ---");
    show_traffic_statistics(&traffic_manager)?;

    // Cleanup
    println!("\n--- Cleanup ---");
    println!(
        "Would unregister {} vehicles from traffic manager",
        vehicles.len()
    );

    // Destroy vehicles
    for (i, vehicle) in vehicles.iter().enumerate() {
        if vehicle.destroy() {
            println!("  Destroyed vehicle {}", i + 1);
        } else {
            println!("  Failed to destroy vehicle {}", i + 1);
        }
    }

    println!("\n=== Demo completed successfully! ===");
    Ok(())
}

fn configure_traffic_manager(tm: &TrafficManagerWrapper) -> Result<()> {
    // Enable synchronous mode
    tm.set_synchronous_mode(true);
    tm.set_synchronous_mode_timeout(2000.0); // 2 seconds timeout
    println!("  Enabled synchronous mode with 2s timeout");

    // Set global speed reduction (vehicles drive 20% slower)
    tm.set_global_speed_percentage(20.0);
    println!("  Set global speed reduction: 20%");

    // Set global lane offset (slightly right)
    tm.set_global_lane_offset(0.2);
    println!("  Set global lane offset: 0.2m right");

    // Set global following distance
    tm.set_global_distance_to_leading_vehicle(3.0);
    println!("  Set global following distance: 3.0m");

    // Enable hybrid physics for performance
    tm.set_hybrid_physics_mode(true);
    tm.set_hybrid_physics_radius(50.0);
    println!("  Enabled hybrid physics mode with 50m radius");

    // Set random seed for reproducible behavior
    tm.set_random_device_seed(42);
    println!("  Set random seed: 42");

    Ok(())
}

fn spawn_demo_vehicles(
    world: &carla_sys::WorldWrapper,
    blueprint_library: &carla_sys::BlueprintLibraryWrapper,
    spawn_points: &[SimpleTransform],
    count: usize,
) -> Result<Vec<carla_sys::ActorWrapper>> {
    let mut vehicles = Vec::new();

    // Find vehicle blueprints
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or_else(|| anyhow::anyhow!("Tesla Model 3 blueprint not found"))?;

    for i in 0..count.min(spawn_points.len()) {
        let spawn_point = &spawn_points[i];

        match world.try_spawn_actor(&vehicle_bp, spawn_point, None) {
            Some(vehicle) => {
                println!(
                    "  Spawned vehicle {} at ({:.1}, {:.1}, {:.1})",
                    i + 1,
                    spawn_point.location.x,
                    spawn_point.location.y,
                    spawn_point.location.z
                );
                vehicles.push(vehicle);
            }
            None => {
                println!("  Failed to spawn vehicle {} at spawn point {}", i + 1, i);
            }
        }
    }

    Ok(vehicles)
}

fn configure_demo_vehicles(
    tm: &TrafficManagerWrapper,
    vehicles: &[carla_sys::ActorWrapper],
) -> Result<()> {
    println!("Individual vehicle configuration would require:");
    println!("  - Proper Vehicle casting from Actor");
    println!("  - C++ bridge implementation");
    println!("  - For now, showing what would be configured:");

    for i in 0..vehicles.len() {
        match i {
            0 => {
                println!("  Vehicle 1: Would be configured as aggressive driver");
                println!("    - 10% faster speed");
                println!("    - 20% chance to ignore other vehicles");
                println!("    - 30% left lane change frequency");
            }
            1 => {
                println!("  Vehicle 2: Would be configured as cautious driver");
                println!("    - 30% slower speed");
                println!("    - 5m following distance");
                println!("    - No automatic lane changes");
            }
            2 => {
                println!("  Vehicle 3: Would be configured as frequent lane changer");
                println!("    - 50% left lane change frequency");
                println!("    - 50% right lane change frequency");
                println!("    - 30% keep right tendency");
            }
            3 => {
                println!("  Vehicle 4: Would be configured as rule breaker");
                println!("    - 25% chance to run red lights");
                println!("    - 15% chance to ignore stop signs");
                println!("    - 10% chance to ignore pedestrians");
            }
            4 => {
                println!("  Vehicle 5: Would be configured with custom behavior");
                println!("    - Constant 15 m/s speed");
                println!("    - -0.5m lane offset (left side)");
                println!("    - Automatic light updates");
            }
            _ => {
                println!("  Vehicle {}: Would use default configuration", i + 1);
            }
        }
    }

    Ok(())
}

fn demonstrate_synchronous_mode(
    tm: &TrafficManagerWrapper,
    world: &carla_sys::WorldWrapper,
) -> Result<()> {
    println!("Synchronous mode demonstration:");
    println!("  Would run 10 synchronous ticks");
    println!("  Each tick would:");
    println!("    1. Tick the world simulation");
    println!("    2. Tick the traffic manager");
    println!("    3. Show frame ID and success status");
    println!("  (Requires C++ bridge implementation)");

    Ok(())
}

fn monitor_traffic(tm: &TrafficManagerWrapper, ticks: u32) -> Result<()> {
    println!("Traffic monitoring simulation:");
    println!("  Would monitor for {} ticks", ticks);
    println!("  Each 5 ticks would show:");
    println!("    - Active vehicle count");
    println!("    - Collision count");
    println!("    - Lane change count");
    println!("    - Average tick time");
    println!("  (Requires C++ bridge implementation)");

    Ok(())
}

fn show_traffic_statistics(tm: &TrafficManagerWrapper) -> Result<()> {
    println!("Final statistics would show:");
    println!("Traffic Manager Statistics:");
    println!("  Port: {}", tm.get_port());
    println!("  Total registered vehicles: <would show count>");
    println!("  Active vehicles: <would show count>");
    println!("  Total ticks processed: <would show count>");
    println!("  Average tick time: <would show ms>");
    println!("  Total collisions: <would show count>");
    println!("  Total lane changes: <would show count>");
    println!("  Traffic light violations: <would show count>");
    println!("  Stop sign violations: <would show count>");
    println!("  Total simulation time: <would show seconds>");

    println!("\nTraffic Manager Configuration:");
    println!("  Global speed percentage: <would show percentage>");
    println!("  Global lane offset: <would show meters>");
    println!("  Global following distance: <would show meters>");
    println!("  Synchronous mode: <would show boolean>");
    println!("  Hybrid physics: <would show boolean>");
    println!("  Hybrid physics radius: <would show meters>");
    println!("  OSM mode: <would show boolean>");
    println!("  (Requires C++ bridge implementation)");

    Ok(())
}
