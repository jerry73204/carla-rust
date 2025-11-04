//! Automatic Control Example
//!
//! This example demonstrates autopilot control and vehicle monitoring.
//! A vehicle drives autonomously while displaying real-time telemetry.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example automatic_control
//! ```

use carla::{
    client::{ActorBase, Client, Vehicle},
    rpc::VehicleControl,
};
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CARLA Automatic Control Example ===\n");

    // Connect to CARLA
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    let mut world = client.world();
    println!("✓ Connected! Current map: {}\n", world.map().name());

    // Spawn a vehicle
    println!("Spawning vehicle...");
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Tesla Model 3 blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).ok_or("No spawn points available")?;

    let actor = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle = Vehicle::try_from(actor).map_err(|_| "Failed to convert to vehicle")?;

    println!("✓ Vehicle spawned (ID: {})", vehicle.id());
    println!("✓ Type: {}\n", vehicle.type_id());

    // Enable autopilot
    println!("Enabling autopilot...");
    vehicle.set_autopilot(true);
    println!("✓ Autopilot enabled\n");

    // Monitor vehicle for 60 seconds
    println!("Monitoring vehicle telemetry (60 seconds)...\n");
    println!(
        "┌─────────┬──────────────────────────┬────────────────────────────┬──────────────────┐"
    );
    println!(
        "│  Time   │        Location          │         Velocity           │    Control       │"
    );
    println!(
        "├─────────┼──────────────────────────┼────────────────────────────┼──────────────────┤"
    );

    for i in 0..60 {
        thread::sleep(Duration::from_secs(1));

        // Get vehicle state
        let transform = vehicle.transform();
        let location = transform.location;
        let velocity = vehicle.velocity();
        let speed = (velocity.x.powi(2) + velocity.y.powi(2) + velocity.z.powi(2)).sqrt();
        let control = vehicle.control();

        // Display telemetry every 2 seconds
        if i % 2 == 0 {
            println!(
                "│ {:>5}s  │ ({:>7.1}, {:>7.1}, {:>5.1}) │ Speed: {:>5.1} km/h          │ T:{:>4.2} S:{:>5.2} │",
                i + 1,
                location.x,
                location.y,
                location.z,
                speed * 3.6, // m/s to km/h
                control.throttle,
                control.steer
            );
        }

        // Print status updates every 10 seconds
        if i % 10 == 0 && i > 0 {
            println!("├─────────┼──────────────────────────┼────────────────────────────┼──────────────────┤");
        }
    }

    println!(
        "└─────────┴──────────────────────────┴────────────────────────────┴──────────────────┘"
    );

    // Final statistics
    let final_location = vehicle.transform().location;
    let final_velocity = vehicle.velocity();
    let final_speed =
        (final_velocity.x.powi(2) + final_velocity.y.powi(2) + final_velocity.z.powi(2)).sqrt();

    println!("\n=== Session Summary ===");
    println!("Duration: 60 seconds");
    println!(
        "Final location: ({:.1}, {:.1}, {:.1})",
        final_location.x, final_location.y, final_location.z
    );
    println!("Final speed: {:.1} km/h", final_speed * 3.6);

    // Test manual control override
    println!("\nTesting manual control override...");
    vehicle.set_autopilot(false);
    println!("✓ Autopilot disabled");

    let manual_control = VehicleControl {
        throttle: 0.5,
        steer: 0.0,
        brake: 0.0,
        hand_brake: false,
        reverse: false,
        manual_gear_shift: false,
        gear: 0,
    };

    vehicle.apply_control(&manual_control);
    println!("✓ Applied manual control (throttle: 0.5)");

    thread::sleep(Duration::from_secs(3));
    println!("✓ Manual control test complete");

    // Cleanup
    println!("\nCleaning up...");
    vehicle.destroy();
    println!("✓ Vehicle destroyed");

    println!("\n=== Automatic Control Demo Complete ===");
    println!("\nKey Features Demonstrated:");
    println!("  • Autopilot control (vehicle.set_autopilot)");
    println!("  • Real-time telemetry monitoring");
    println!("  • Vehicle state queries (location, velocity, control)");
    println!("  • Manual control override");
    println!("  • Actor cleanup");

    Ok(())
}
