//! Vehicle Physics Example
//!
//! This example demonstrates:
//! - Applying impulses to vehicles
//! - Applying forces to vehicles
//! - Comparing physics effects
//! - Using synchronous mode for precise physics control
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example vehicle_physics
//! ```

use carla::client::{ActorBase, Client, Vehicle};
use nalgebra::Vector3;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CARLA Vehicle Physics Example ===\n");

    // Connect to CARLA
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    let mut world = client.world();
    println!("✓ Connected! Current map: {}\n", world.map().name());

    // Enable synchronous mode for precise control
    println!("Enabling synchronous mode...");
    let mut settings = world.settings();
    settings.synchronous_mode = true;
    settings.fixed_delta_seconds = Some(0.05); // 20 FPS
    world.apply_settings(&settings, Duration::from_secs(5));
    println!("✓ Synchronous mode enabled (delta: 0.05s)\n");

    // Get vehicle blueprint
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Tesla Model 3 blueprint not found")?;

    // Get spawn points
    let spawn_points = world.map().recommended_spawn_points();

    // Demo 1: Apply Impulse (Instantaneous velocity change)
    println!("=== Demo 1: Apply Impulse ===");
    println!("Spawning vehicle for impulse test...");

    let spawn_point = spawn_points.get(0).ok_or("No spawn points available")?;
    let actor = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle1 = Vehicle::try_from(actor).map_err(|_| "Failed to convert to vehicle")?;

    println!("✓ Vehicle spawned (ID: {})", vehicle1.id());
    println!("Initial position: {:?}", vehicle1.transform().location);

    // Tick the simulation to settle the vehicle
    for _ in 0..10 {
        world.tick();
    }

    // Apply upward impulse (lift the vehicle)
    println!("\nApplying upward impulse (0, 0, 5000 N·s)...");
    let impulse = Vector3::new(0.0, 0.0, 5000.0);
    vehicle1.add_impulse(&impulse);

    // Observe the effect over time
    println!("Observing vehicle motion for 3 seconds...");
    for i in 0..60 {
        world.tick();

        if i % 10 == 0 {
            let pos = vehicle1.transform().location;
            let vel = vehicle1.velocity();
            println!(
                "  t={:.1}s: pos=({:.1}, {:.1}, {:.1}), vel=({:.1}, {:.1}, {:.1})",
                i as f32 * 0.05,
                pos.x,
                pos.y,
                pos.z,
                vel.x,
                vel.y,
                vel.z
            );
        }
    }

    println!("✓ Impulse demo complete\n");
    // NOTE: Actor.destroy() not yet implemented

    // Demo 2: Apply Force (Continuous acceleration)
    println!("=== Demo 2: Apply Force ===");
    println!("Spawning vehicle for force test...");

    let spawn_point = spawn_points.get(1).ok_or("Not enough spawn points")?;
    let actor = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle2 = Vehicle::try_from(actor).map_err(|_| "Failed to convert to vehicle")?;

    println!("✓ Vehicle spawned (ID: {})", vehicle2.id());
    println!("Initial position: {:?}", vehicle2.transform().location);

    // Tick to settle
    for _ in 0..10 {
        world.tick();
    }

    // Apply continuous forward force
    println!("\nApplying continuous forward force (5000, 0, 0 N) for 3 seconds...");
    let force = Vector3::new(5000.0, 0.0, 0.0);

    for i in 0..60 {
        // Apply force each tick (continuous acceleration)
        vehicle2.add_force(&force);
        world.tick();

        if i % 10 == 0 {
            let pos = vehicle2.transform().location;
            let vel = vehicle2.velocity();
            println!(
                "  t={:.1}s: pos=({:.1}, {:.1}, {:.1}), vel=({:.1}, {:.1}, {:.1})",
                i as f32 * 0.05,
                pos.x,
                pos.y,
                pos.z,
                vel.x,
                vel.y,
                vel.z
            );
        }
    }

    println!("✓ Force demo complete\n");

    // Demo 3: Torque Application (Rotation)
    println!("=== Demo 3: Apply Torque ===");
    println!("Spawning vehicle for torque test...");

    let spawn_point = spawn_points.get(2).ok_or("Not enough spawn points")?;
    let actor = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle3 = Vehicle::try_from(actor).map_err(|_| "Failed to convert to vehicle")?;

    println!("✓ Vehicle spawned (ID: {})", vehicle3.id());

    // Tick to settle
    for _ in 0..10 {
        world.tick();
    }

    // Apply torque to spin the vehicle
    println!("\nApplying Z-axis torque (0, 0, 10000 N·m) for 3 seconds...");
    let torque = Vector3::new(0.0, 0.0, 10000.0);

    for i in 0..60 {
        vehicle3.add_torque(&torque);
        world.tick();

        if i % 10 == 0 {
            let angular_vel = vehicle3.angular_velocity();
            let transform = vehicle3.transform();
            println!(
                "  t={:.1}s: angular_vel=({:.1}, {:.1}, {:.1}), yaw={:.1}°",
                i as f32 * 0.05,
                angular_vel.x,
                angular_vel.y,
                angular_vel.z,
                transform.rotation.yaw
            );
        }
    }

    println!("✓ Torque demo complete\n");

    // Demo 4: Combined Effects
    println!("=== Demo 4: Combined Physics Effects ===");
    println!("Spawning vehicle for combined test...");

    let spawn_point = spawn_points.get(3).ok_or("Not enough spawn points")?;
    let actor = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle4 = Vehicle::try_from(actor).map_err(|_| "Failed to convert to vehicle")?;

    println!("✓ Vehicle spawned (ID: {})", vehicle4.id());

    // Tick to settle
    for _ in 0..10 {
        world.tick();
    }

    // Apply multiple forces simultaneously
    println!("\nApplying combined impulse + continuous force...");
    println!("  Impulse: (2000, 0, 3000) N·s");
    println!("  Force: (1000, 500, 0) N");

    // Initial impulse
    vehicle4.add_impulse(&Vector3::new(2000.0, 0.0, 3000.0));

    // Continuous force
    for i in 0..60 {
        vehicle4.add_force(&Vector3::new(1000.0, 500.0, 0.0));
        world.tick();

        if i % 15 == 0 {
            let pos = vehicle4.transform().location;
            let vel = vehicle4.velocity();
            println!(
                "  t={:.1}s: pos=({:.1}, {:.1}, {:.1}), vel=({:.1}, {:.1}, {:.1})",
                i as f32 * 0.05,
                pos.x,
                pos.y,
                pos.z,
                vel.x,
                vel.y,
                vel.z
            );
        }
    }

    println!("✓ Combined demo complete\n");

    // Restore asynchronous mode
    println!("Restoring asynchronous mode...");
    settings.synchronous_mode = false;
    settings.fixed_delta_seconds = None;
    world.apply_settings(&settings, Duration::from_secs(5));
    println!("✓ Asynchronous mode restored");

    // Cleanup
    println!("\nCleaning up vehicles...");
    vehicle1.destroy();
    vehicle2.destroy();
    vehicle3.destroy();
    vehicle4.destroy();
    println!("✓ All vehicles destroyed");

    println!("\n=== Vehicle Physics Demo Complete ===");
    println!("Key Takeaways:");
    println!("  • Impulse: Instantaneous velocity change (one-time push)");
    println!("  • Force: Continuous acceleration (ongoing push)");
    println!("  • Torque: Rotational force (spinning)");
    println!("  • Synchronous mode enables precise physics control");

    Ok(())
}
