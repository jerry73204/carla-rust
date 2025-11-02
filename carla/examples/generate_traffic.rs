//! Traffic Generation Example
//!
//! Demonstrates how to generate realistic traffic by spawning:
//! - Multiple vehicles with autopilot enabled
//! - Pedestrians (walkers) with AI controllers
//!
//! This example shows batch operations for efficient actor spawning
//! and demonstrates the walker AI controller system.
//!
//! # What it does
//!
//! 1. Spawns vehicles at random spawn points with autopilot
//! 2. Spawns walkers at random locations on sidewalks
//! 3. Attaches AI controllers to walkers for autonomous navigation
//! 4. Commands walkers to walk to random destinations
//! 5. Monitors traffic for a specified duration
//! 6. Cleans up all spawned actors
//!
//! # Output
//!
//! Displays statistics about spawned traffic and their movement patterns.
//!
//! # Usage
//!
//! ```bash
//! cargo run --example generate_traffic
//! ```

use carla::{
    client::{Client, WalkerAIController},
    geom::{Location, Transform},
    rpc::Command,
};
use rand::Rng;
use std::{thread, time::Duration};

const NUM_VEHICLES: usize = 30;
const NUM_WALKERS: usize = 50;
const SIMULATION_DURATION_SECS: u64 = 60;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Traffic Generation Example");
    println!("==========================\n");

    // Connect to CARLA server
    let mut client = Client::default();
    let mut world = client.world();
    let mut rng = rand::thread_rng();

    println!("Connected to CARLA simulator");
    println!("Map: {}\n", world.map().name());

    // Configure pedestrian behavior
    world.set_pedestrians_seed(rng.gen_range(0..100000));
    world.set_pedestrians_cross_factor(0.2); // 20% chance to cross roads

    let bp_lib = world.blueprint_library();
    let spawn_points = world.map().recommended_spawn_points();

    // ========================================================================
    // PART 1: Spawn Vehicles with Autopilot
    // ========================================================================

    println!("=== Spawning {} Vehicles ===", NUM_VEHICLES);

    let vehicle_blueprints = bp_lib.filter("vehicle.*");
    if vehicle_blueprints.is_empty() {
        return Err("No vehicle blueprints found".into());
    }

    // Create batch spawn commands for vehicles
    let mut vehicle_spawn_commands = Vec::new();
    for i in 0..NUM_VEHICLES.min(spawn_points.len()) {
        let vehicle_bp = vehicle_blueprints
            .get(rng.gen_range(0..vehicle_blueprints.len()))
            .ok_or("Failed to get vehicle blueprint")?;
        let spawn_point = spawn_points.get(i).ok_or("No spawn point available")?;
        let transform = Transform::from_na(&spawn_point);

        vehicle_spawn_commands.push(Command::spawn_actor(vehicle_bp.clone(), transform, None));
    }

    println!("Spawning vehicles in batch...");
    let vehicle_responses = client.apply_batch_sync(vehicle_spawn_commands, false);

    let mut vehicle_ids = Vec::new();
    for (i, response) in vehicle_responses.iter().enumerate() {
        if let Some(actor_id) = response.actor_id() {
            vehicle_ids.push(actor_id);
            if i % 10 == 0 {
                println!("  Spawned {} vehicles...", i + 1);
            }
        } else if let Some(error) = response.error() {
            eprintln!("  Failed to spawn vehicle {}: {}", i, error);
        }
    }

    println!("✓ Successfully spawned {} vehicles\n", vehicle_ids.len());

    // Enable autopilot for all vehicles
    println!("Enabling autopilot for vehicles...");
    let mut autopilot_commands = Vec::new();
    for &vehicle_id in &vehicle_ids {
        autopilot_commands.push(Command::set_autopilot(vehicle_id, true, 8000));
    }
    client.apply_batch_sync(autopilot_commands, false);
    println!("✓ Autopilot enabled\n");

    // ========================================================================
    // PART 2: Spawn Walkers (Pedestrians)
    // ========================================================================

    println!("=== Spawning {} Walkers ===", NUM_WALKERS);

    let walker_blueprints = bp_lib.filter("walker.pedestrian.*");
    if walker_blueprints.is_empty() {
        return Err("No walker blueprints found".into());
    }

    // Generate random spawn locations for walkers
    let mut walker_spawn_commands = Vec::new();
    for _ in 0..NUM_WALKERS {
        let walker_bp = walker_blueprints
            .get(rng.gen_range(0..walker_blueprints.len()))
            .ok_or("Failed to get walker blueprint")?;

        // Get random location from navigation system
        let translation = world.random_location_from_navigation();
        let location = Location::from_na_translation(&translation);
        let rotation = carla::geom::Rotation {
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
        };
        let transform = Transform { location, rotation };
        walker_spawn_commands.push(Command::spawn_actor(walker_bp.clone(), transform, None));
    }

    println!("Spawning walkers in batch...");
    let walker_responses = client.apply_batch_sync(walker_spawn_commands, false);

    let mut walker_ids = Vec::new();
    for (i, response) in walker_responses.iter().enumerate() {
        if let Some(actor_id) = response.actor_id() {
            walker_ids.push(actor_id);
            if i % 10 == 0 {
                println!("  Spawned {} walkers...", i + 1);
            }
        }
    }

    println!("✓ Successfully spawned {} walkers\n", walker_ids.len());

    // ========================================================================
    // PART 3: Spawn Walker AI Controllers
    // ========================================================================

    println!("=== Spawning Walker AI Controllers ===");

    let controller_bp = bp_lib
        .find("controller.ai.walker")
        .ok_or("Walker AI controller blueprint not found")?;

    // Spawn AI controller for each walker
    let mut controller_spawn_commands = Vec::new();
    for &walker_id in &walker_ids {
        let zero_transform = Transform {
            location: Location {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: carla::geom::Rotation {
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
            },
        };
        controller_spawn_commands.push(Command::spawn_actor(
            controller_bp.clone(),
            zero_transform,
            Some(walker_id),
        ));
    }

    println!("Spawning AI controllers in batch...");
    let controller_responses = client.apply_batch_sync(controller_spawn_commands, false);

    let mut controller_ids = Vec::new();
    for response in controller_responses {
        if let Some(actor_id) = response.actor_id() {
            controller_ids.push(actor_id);
        }
    }

    println!(
        "✓ Successfully spawned {} AI controllers\n",
        controller_ids.len()
    );

    // ========================================================================
    // PART 4: Start Walker AI
    // ========================================================================

    println!("=== Starting Walker AI ===");

    // Small delay to ensure all actors are initialized
    thread::sleep(Duration::from_millis(500));

    let mut walker_controllers = Vec::new();
    for &controller_id in &controller_ids {
        if let Some(actor) = world.actor(controller_id) {
            if let Ok(controller) = WalkerAIController::try_from(actor) {
                // Start the AI
                controller.start();

                // Set walking speed (random between 0.5 and 2.0 m/s)
                let speed = rng.gen_range(0.5..2.0);
                controller.set_max_speed(speed);

                // Set random destination
                if let Some(destination) = controller.get_random_location() {
                    controller.go_to_location(&destination);
                }

                walker_controllers.push(controller);
            }
        }
    }

    println!(
        "✓ Started {} walker AI controllers\n",
        walker_controllers.len()
    );

    // ========================================================================
    // PART 5: Monitor Traffic
    // ========================================================================

    println!("=== Traffic Active ===");
    println!(
        "Running simulation for {} seconds...\n",
        SIMULATION_DURATION_SECS
    );

    for elapsed in 0..SIMULATION_DURATION_SECS {
        thread::sleep(Duration::from_secs(1));

        if elapsed % 10 == 0 && elapsed > 0 {
            println!(
                "[{:02}:{:02}] Traffic active - {} vehicles, {} walkers",
                elapsed / 60,
                elapsed % 60,
                vehicle_ids.len(),
                walker_ids.len()
            );
        }
    }

    // ========================================================================
    // PART 6: Cleanup
    // ========================================================================

    println!("\n=== Cleaning Up ===");

    // Stop walker AI controllers
    println!("Stopping walker AI controllers...");
    for controller in walker_controllers {
        controller.stop();
    }

    // Destroy all spawned actors
    println!("Destroying all spawned actors...");

    let mut destroy_commands = Vec::new();

    for &controller_id in &controller_ids {
        destroy_commands.push(Command::destroy_actor(controller_id));
    }
    for &walker_id in &walker_ids {
        destroy_commands.push(Command::destroy_actor(walker_id));
    }
    for &vehicle_id in &vehicle_ids {
        destroy_commands.push(Command::destroy_actor(vehicle_id));
    }

    client.apply_batch_sync(destroy_commands, false);

    println!("✓ Cleanup complete\n");

    println!("=== Traffic Generation Demo Complete ===");
    println!("\nSummary:");
    println!("  {} vehicles spawned with autopilot", vehicle_ids.len());
    println!("  {} walkers spawned with AI control", walker_ids.len());
    println!("  {} AI controllers created", controller_ids.len());
    println!(
        "  Simulation duration: {} seconds",
        SIMULATION_DURATION_SECS
    );

    Ok(())
}
