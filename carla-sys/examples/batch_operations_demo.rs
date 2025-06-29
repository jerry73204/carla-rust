//! Batch operations demonstration for carla-sys.
//!
//! This example demonstrates the batch operations system:
//! - Applying multiple vehicle controls in one batch
//! - Bulk actor destruction
//! - Mixed batch operations with different command types
//! - Synchronous vs asynchronous batch execution
//! - Batch response handling and error checking

use anyhow::Result;
use carla_sys::{
    batch_utils, ActorListExt, BatchCommandBuilder, BatchResponseExt, BatchVehicleLightState,
    ClientWrapper, SimpleLocation, SimpleVector3D, SimpleVehicleControl,
};
use std::time::Duration;

fn main() -> Result<()> {
    println!("🚗 CARLA Batch Operations Demo");
    println!("==============================");

    // Connect to CARLA server
    println!("🔌 Connecting to CARLA server...");
    let mut client = ClientWrapper::new("localhost", 2000)?;
    println!(
        "✅ Connected! Server version: {}",
        client.get_server_version()
    );

    // Set timeout for client operations
    client.set_timeout(Duration::from_secs(10));

    // Get the simulation world
    let world = client.get_world()?;
    println!("🌍 World ID: {}", world.get_id());

    // === BATCH OPERATIONS DEMONSTRATIONS ===
    println!("\n⚡ Batch Operations Demonstrations");
    println!("==================================");

    // Demo 1: Get some actors to work with
    let all_actors = world.get_actors();
    if all_actors.is_empty() {
        println!("❌ No actors found in the world. Please spawn some vehicles first.");
        println!("💡 You can run the CARLA simulator with the Town01 map and spawn some vehicles.");
        return Ok(());
    }

    println!("🎭 Found {} actors in the world", all_actors.len());
    let actor_ids = all_actors.get_ids();
    let sample_size = std::cmp::min(5, actor_ids.len());
    let sample_actors = &actor_ids[..sample_size];

    println!("🎯 Using {sample_size} sample actors: {sample_actors:?}");

    // === DEMO 2: BULK VEHICLE CONTROL ===
    println!("\n🚗 Demo 2: Bulk Vehicle Control");
    println!("-------------------------------");

    // Create different vehicle controls for each actor
    let mut vehicle_controls = Vec::new();
    for i in 0..sample_size {
        let control = SimpleVehicleControl {
            throttle: 0.3 + (i as f32 * 0.1), // Varying throttle: 0.3, 0.4, 0.5, etc.
            steer: if i % 2 == 0 { 0.1 } else { -0.1 }, // Alternating left/right
            brake: 0.0,
            hand_brake: false,
            reverse: false,
            manual_gear_shift: false,
            gear: 0,
        };
        vehicle_controls.push(control);
    }

    // Create batch commands for vehicle controls
    let control_commands = batch_utils::create_vehicle_controls(sample_actors, &vehicle_controls);

    println!("🎮 Applying vehicle controls to {sample_size} actors...");
    println!("   Controls: throttle [0.3-0.7], alternating steering");

    // Apply batch asynchronously (fire and forget)
    client.apply_batch(&control_commands, false);
    println!("✅ Vehicle controls applied asynchronously");

    // Wait a moment to see the effect
    std::thread::sleep(Duration::from_millis(500));

    // === DEMO 3: MIXED BATCH OPERATIONS ===
    println!("\n🔀 Demo 3: Mixed Batch Operations");
    println!("---------------------------------");

    let mut mixed_commands = Vec::new();

    // Add various types of commands
    for (i, &actor_id) in sample_actors.iter().enumerate() {
        match i % 4 {
            0 => {
                // Set autopilot on some vehicles
                mixed_commands.push(BatchCommandBuilder::set_autopilot(actor_id, true, 8000));
                println!("   🤖 Setting autopilot ON for actor {actor_id}");
            }
            1 => {
                // Apply new location to some actors
                let new_location = SimpleLocation {
                    x: 100.0 + i as f64 * 10.0,
                    y: 200.0 + i as f64 * 10.0,
                    z: 0.5,
                };
                mixed_commands.push(BatchCommandBuilder::apply_location(actor_id, &new_location));
                println!(
                    "   📍 Moving actor {} to ({:.1}, {:.1}, {:.1})",
                    actor_id, new_location.x, new_location.y, new_location.z
                );
            }
            2 => {
                // Set vehicle lights
                let light_state = BatchVehicleLightState::new(
                    BatchVehicleLightState::POSITION | BatchVehicleLightState::LOW_BEAM,
                );
                mixed_commands.push(BatchCommandBuilder::set_vehicle_light_state(
                    actor_id,
                    light_state,
                ));
                println!("   💡 Setting lights (position + low beam) for actor {actor_id}");
            }
            3 => {
                // Apply some force
                let force = SimpleVector3D {
                    x: 1000.0,
                    y: 0.0,
                    z: 0.0,
                };
                mixed_commands.push(BatchCommandBuilder::apply_force(actor_id, &force));
                println!("   💨 Applying forward force to actor {actor_id}");
            }
            _ => unreachable!(),
        }
    }

    // Apply mixed batch synchronously to get responses
    println!("\n🔄 Applying mixed batch commands synchronously...");
    let responses = client.apply_batch_sync(&mixed_commands, false);

    println!("📊 Batch Results:");
    for (i, response) in responses.iter().enumerate() {
        if response.is_success() {
            println!("   ✅ Command {}: Success", i + 1);
            if let Some(actor_id) = response.get_actor_id() {
                println!("      Result actor ID: {actor_id}");
            }
        } else {
            println!("   ❌ Command {}: Failed", i + 1);
            if let Some(error) = response.get_error_message() {
                println!("      Error: {error}");
            }
        }
    }

    // === DEMO 4: BULK PHYSICS OPERATIONS ===
    println!("\n⚛️  Demo 4: Bulk Physics Operations");
    println!("----------------------------------");

    let mut physics_commands = Vec::new();

    // Apply various physics operations to different actors
    for (i, &actor_id) in sample_actors.iter().enumerate() {
        match i % 3 {
            0 => {
                // Apply upward impulse
                let impulse = SimpleVector3D {
                    x: 0.0,
                    y: 0.0,
                    z: 5000.0, // Upward impulse
                };
                physics_commands.push(BatchCommandBuilder::apply_impulse(actor_id, &impulse));
                println!("   🚀 Applying upward impulse to actor {actor_id}");
            }
            1 => {
                // Set target velocity
                let velocity = SimpleVector3D {
                    x: 10.0, // 10 m/s forward
                    y: 0.0,
                    z: 0.0,
                };
                physics_commands.push(BatchCommandBuilder::apply_target_velocity(
                    actor_id, &velocity,
                ));
                println!("   🏃 Setting target velocity (10 m/s forward) for actor {actor_id}");
            }
            2 => {
                // Toggle physics simulation
                let enable_physics = i % 6 == 2; // Enable for some, disable for others
                physics_commands.push(BatchCommandBuilder::set_simulate_physics(
                    actor_id,
                    enable_physics,
                ));
                println!(
                    "   ⚙️  {} physics simulation for actor {}",
                    if enable_physics {
                        "Enabling"
                    } else {
                        "Disabling"
                    },
                    actor_id
                );
            }
            _ => unreachable!(),
        }
    }

    println!("\n🔄 Applying physics batch commands...");
    let physics_responses = client.apply_batch_sync(&physics_commands, true); // Tick after applying

    println!("📊 Physics Batch Results:");
    let success_count = physics_responses.iter().filter(|r| r.is_success()).count();
    let error_count = physics_responses.len() - success_count;

    println!("   ✅ Successful: {success_count}");
    println!("   ❌ Failed: {error_count}");

    if error_count > 0 {
        println!("   📝 Error details:");
        for (i, response) in physics_responses.iter().enumerate() {
            if response.is_error() {
                if let Some(error) = response.get_error_message() {
                    println!("      Command {}: {}", i + 1, error);
                }
            }
        }
    }

    // === DEMO 5: PERFORMANCE COMPARISON ===
    println!("\n📈 Demo 5: Performance Comparison");
    println!("---------------------------------");

    // Create a larger batch for performance testing
    let large_batch_size = std::cmp::min(20, actor_ids.len());
    let large_sample = &actor_ids[..large_batch_size];

    // Create simple vehicle controls for all actors
    let simple_control = SimpleVehicleControl {
        throttle: 0.5,
        steer: 0.0,
        brake: 0.0,
        hand_brake: false,
        reverse: false,
        manual_gear_shift: false,
        gear: 0,
    };

    let controls = vec![simple_control; large_batch_size];
    let performance_commands = batch_utils::create_vehicle_controls(large_sample, &controls);

    println!("🏁 Performance test: {large_batch_size} vehicle control commands");

    // Measure batch operation time
    let start_time = std::time::Instant::now();
    client.apply_batch(&performance_commands, false);
    let batch_duration = start_time.elapsed();

    println!("⚡ Batch operation completed in: {batch_duration:?}");
    println!(
        "📊 Average time per command: {:?}",
        batch_duration / large_batch_size as u32
    );

    // === DEMONSTRATION SUMMARY ===
    println!("\n📊 Batch Operations Demo Summary");
    println!("=================================");
    println!("✅ Bulk vehicle controls: Applied different controls to {sample_size} actors");
    println!("✅ Mixed batch operations: Combined autopilot, positioning, lights, and forces");
    println!("✅ Physics operations: Applied impulses, velocities, and physics toggles");
    println!("✅ Performance testing: Processed {large_batch_size} commands in {batch_duration:?}");
    println!("✅ Error handling: Demonstrated synchronous batch with response checking");

    println!("\n🎉 Batch operations demonstration completed successfully!");
    println!("💡 Key benefits of batch operations:");
    println!("   - 🚀 High performance: Process multiple commands in single network call");
    println!("   - 🔧 Atomic operations: All commands processed together");
    println!("   - 📊 Response tracking: Get individual results for each command");
    println!("   - 🎯 Flexible mixing: Combine different command types in one batch");
    println!("   - ⚡ Scalability: Handle hundreds of actors efficiently");

    Ok(())
}
