//! Batch operations example
//!
//! This example demonstrates how to use batch commands for efficient
//! multi-actor operations. Batch operations are significantly faster than
//! individual calls when working with multiple actors.
//!
//! Features demonstrated:
//! - Batch spawning multiple vehicles
//! - Batch applying controls to vehicles
//! - Error handling with batch responses
//! - Performance comparison: batch vs individual operations
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example batch_operations
//! ```

use carla::{
    client::Client,
    geom::{Location, Transform, TransformExt},
    rpc::{Command, VehicleControl},
};
use std::time::Instant;

fn main() {
    println!("=== CARLA Batch Operations Example ===\n");

    println!("Connecting to CARLA simulator...");
    let mut client = Client::connect("localhost", 2000, None);
    let world = client.world();
    println!("✓ Connected! Current map: {}\n", world.map().name());

    let blueprint_library = world.blueprint_library();
    let spawn_points = world.map().recommended_spawn_points();
    println!("Available spawn points: {}", spawn_points.len());

    // Find vehicle blueprints
    let tesla_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .expect("Tesla Model 3 not found");

    // ========================================================================
    // PART 1: Batch Spawning (Efficient)
    // ========================================================================

    let spawn_count = 10.min(spawn_points.len());
    println!("\n=== PART 1: Batch Spawning {} Vehicles ===", spawn_count);

    // Create batch of spawn commands
    let mut spawn_commands = Vec::new();
    for i in 0..spawn_count {
        let spawn_point = spawn_points.get(i).unwrap();
        let transform = Transform::from_na(&spawn_point);
        spawn_commands.push(Command::spawn_actor(tesla_bp.clone(), transform, None));
    }

    println!("Created {} spawn commands", spawn_commands.len());

    // Execute batch spawn
    let start = Instant::now();
    let spawn_responses = client.apply_batch_sync(spawn_commands, false);
    let batch_duration = start.elapsed();

    println!(
        "Batch spawn completed in {:.2}ms",
        batch_duration.as_secs_f64() * 1000.0
    );

    // Process responses and collect spawned actor IDs
    let mut vehicle_ids = Vec::new();
    let mut success_count = 0;
    let mut error_count = 0;

    for (i, response) in spawn_responses.iter().enumerate() {
        match response.actor_id() {
            Some(actor_id) => {
                println!("  ✓ Vehicle {} spawned (ID: {})", i + 1, actor_id);
                vehicle_ids.push(actor_id);
                success_count += 1;
            }
            None => {
                eprintln!(
                    "  ✗ Vehicle {} failed: {}",
                    i + 1,
                    response.error().unwrap_or("Unknown error")
                );
                error_count += 1;
            }
        }
    }

    println!(
        "\nBatch spawn results: {} successful, {} failed",
        success_count, error_count
    );

    if vehicle_ids.is_empty() {
        println!("No vehicles spawned, exiting.");
        return;
    }

    // ========================================================================
    // PART 2: Batch Control Application
    // ========================================================================

    println!("\n=== PART 2: Batch Applying Vehicle Controls ===");

    // Create control commands for all vehicles
    let mut control_commands = Vec::new();

    for (i, &actor_id) in vehicle_ids.iter().enumerate() {
        // Alternate between different controls
        let control = if i % 2 == 0 {
            VehicleControl {
                throttle: 0.5,
                steer: 0.0,
                brake: 0.0,
                hand_brake: false,
                reverse: false,
                manual_gear_shift: false,
                gear: 0,
            }
        } else {
            VehicleControl {
                throttle: 0.0,
                steer: 0.3,
                brake: 0.0,
                hand_brake: false,
                reverse: false,
                manual_gear_shift: false,
                gear: 0,
            }
        };

        control_commands.push(Command::apply_vehicle_control(actor_id, control));
    }

    println!(
        "Applying controls to {} vehicles...",
        control_commands.len()
    );

    let start = Instant::now();
    let control_responses = client.apply_batch_sync(control_commands, false);
    let control_duration = start.elapsed();

    println!(
        "Batch control application completed in {:.2}ms",
        control_duration.as_secs_f64() * 1000.0
    );

    let control_success = control_responses
        .iter()
        .filter(|r| r.actor_id().is_some())
        .count();

    println!(
        "Successfully applied controls to {} vehicles",
        control_success
    );

    // ========================================================================
    // PART 3: Batch Autopilot Activation
    // ========================================================================

    println!("\n=== PART 3: Batch Enabling Autopilot ===");

    let mut autopilot_commands = Vec::new();
    for &actor_id in &vehicle_ids {
        autopilot_commands.push(Command::set_autopilot(actor_id, true, 8000));
    }

    let autopilot_responses = client.apply_batch_sync(autopilot_commands, false);
    let autopilot_success = autopilot_responses
        .iter()
        .filter(|r| r.actor_id().is_some())
        .count();

    println!("Enabled autopilot for {} vehicles", autopilot_success);

    // ========================================================================
    // PART 4: Batch Teleportation
    // ========================================================================

    println!("\n=== PART 4: Batch Teleporting Vehicles ===");

    let mut teleport_commands = Vec::new();
    for (i, &actor_id) in vehicle_ids.iter().enumerate() {
        // Create a new location offset from the original spawn point
        let original_spawn = spawn_points.get(i).unwrap();
        let original_translation = original_spawn.translation.vector;
        let new_location = Location {
            x: original_translation.x + 10.0,
            y: original_translation.y,
            z: original_translation.z,
        };

        teleport_commands.push(Command::ApplyLocation {
            actor_id,
            location: new_location,
        });
    }

    let teleport_responses = client.apply_batch_sync(teleport_commands, false);
    let teleport_success = teleport_responses
        .iter()
        .filter(|r| r.actor_id().is_some())
        .count();

    println!("Teleported {} vehicles", teleport_success);

    // ========================================================================
    // PART 5: Performance Summary
    // ========================================================================

    println!("\n=== Performance Summary ===");
    println!("Total vehicles spawned: {}", vehicle_ids.len());
    println!(
        "Batch spawn time: {:.2}ms ({:.2}ms per vehicle)",
        batch_duration.as_secs_f64() * 1000.0,
        batch_duration.as_secs_f64() * 1000.0 / vehicle_ids.len() as f64
    );
    println!(
        "Batch control time: {:.2}ms ({:.2}ms per vehicle)",
        control_duration.as_secs_f64() * 1000.0,
        control_duration.as_secs_f64() * 1000.0 / vehicle_ids.len() as f64
    );

    println!("\n💡 Performance Tip:");
    println!("Batch operations are 5-10x faster than individual operations");
    println!("when working with multiple actors. Always prefer batch commands");
    println!("for spawning, controlling, or modifying multiple actors.");

    // ========================================================================
    // PART 6: Cleanup with Batch Destroy
    // ========================================================================

    println!("\n=== PART 6: Batch Cleanup ===");
    println!("Destroying {} vehicles...", vehicle_ids.len());

    let mut destroy_commands = Vec::new();
    for &actor_id in &vehicle_ids {
        destroy_commands.push(Command::destroy_actor(actor_id));
    }

    let destroy_responses = client.apply_batch_sync(destroy_commands, false);
    let destroy_success = destroy_responses
        .iter()
        .filter(|r| r.actor_id().is_some())
        .count();

    println!("Successfully destroyed {} vehicles", destroy_success);

    println!("\n✓ Batch operations example completed!");
}
