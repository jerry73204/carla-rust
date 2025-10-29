//! Generate Traffic Example - Spawning vehicles and pedestrians
//!
//! This example demonstrates:
//! - Spawning multiple vehicles with autopilot enabled
//! - Spawning pedestrian walkers with AI controllers
//! - Using Traffic Manager for vehicle coordination
//! - Walker AI navigation to random locations
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example generate_traffic
//! ```

use carla::{
    client::{ActorBase, Client, Vehicle, WalkerAIController},
    geom::{Location, LocationExt},
};
use nalgebra::Isometry3;
use rand::Rng;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CARLA Traffic Generation Example ===\n");

    // Step 1: Connect to CARLA
    println!("Step 1: Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    let mut world = client.world();
    println!("✓ Connected! Current map: {}\n", world.map().name());

    // Step 2: Configure settings for better performance
    println!("Step 2: Configuring simulation settings...");
    let mut settings = world.settings();
    settings.synchronous_mode = true;
    settings.fixed_delta_seconds = Some(0.05);
    world.apply_settings(&settings, Duration::from_secs(5));
    println!("✓ Synchronous mode enabled with 0.05s fixed delta\n");

    // Step 3: Spawn vehicles
    println!("Step 3: Spawning vehicles with autopilot...");
    let blueprint_library = world.blueprint_library();
    let vehicle_blueprints: Vec<_> = blueprint_library.filter("vehicle.tesla.*").iter().collect();

    if vehicle_blueprints.is_empty() {
        return Err("No vehicle blueprints found".into());
    }

    let spawn_points = world.map().recommended_spawn_points();
    let num_vehicles = 10.min(spawn_points.len());

    let mut vehicles = Vec::new();
    let mut rng = rand::thread_rng();

    for i in 0..num_vehicles {
        let spawn_point = spawn_points.get(i).unwrap();
        let blueprint = vehicle_blueprints[rng.gen_range(0..vehicle_blueprints.len())].clone();

        match world.spawn_actor(&blueprint, &spawn_point) {
            Ok(actor) => {
                if let Ok(vehicle) = Vehicle::try_from(actor) {
                    vehicle.set_autopilot(true);
                    println!("  ✓ Spawned vehicle {} (ID: {})", i + 1, vehicle.id());
                    vehicles.push(vehicle);
                } else {
                    eprintln!("  ✗ Failed to convert actor to vehicle");
                }
            }
            Err(e) => {
                eprintln!("  ✗ Failed to spawn vehicle {}: {}", i + 1, e);
            }
        }
    }
    println!("✓ Spawned {} vehicles\n", vehicles.len());

    // Step 5: Spawn walkers with AI controllers
    println!("Step 5: Spawning pedestrians with AI controllers...");
    let walker_blueprints: Vec<_> = blueprint_library
        .filter("walker.pedestrian.*")
        .iter()
        .collect();

    if walker_blueprints.is_empty() {
        return Err("No walker blueprints found".into());
    }

    let walker_controller_bp = blueprint_library
        .find("controller.ai.walker")
        .ok_or("Walker AI controller blueprint not found")?;

    let num_walkers = 5;
    let mut walkers = Vec::new();
    let mut walker_controllers = Vec::new();

    // Set pedestrian crossing factor
    world.set_pedestrians_cross_factor(0.0); // 0% will cross roads

    for i in 0..num_walkers {
        // Get random spawn location from navigation mesh
        let spawn_location = world.random_location_from_navigation();
        let spawn_transform =
            Isometry3::from_parts(spawn_location, nalgebra::UnitQuaternion::identity());

        // Spawn walker
        let walker_bp = walker_blueprints[rng.gen_range(0..walker_blueprints.len())].clone();

        match world.spawn_actor(&walker_bp, &spawn_transform) {
            Ok(walker_actor) => {
                let walker_id = walker_actor.id();
                println!("  ✓ Spawned walker {} (ID: {})", i + 1, walker_id);

                // Spawn AI controller for this walker
                let controller_transform = Isometry3::identity();
                match world.spawn_actor_opt(
                    &walker_controller_bp,
                    &controller_transform,
                    Some(&walker_actor),
                    carla::rpc::AttachmentType::Rigid,
                ) {
                    Ok(controller_actor) => {
                        if let Ok(controller) = WalkerAIController::try_from(controller_actor) {
                            // Start AI and set random destination
                            controller.start();

                            let random_dest = world.random_location_from_navigation();
                            let dest_location = Location::from_na_translation(&random_dest);
                            controller.go_to_location(&dest_location);

                            // Set walking speed (m/s)
                            let speed = if rng.gen_bool(0.8) { 1.4 } else { 2.5 }; // 80% walk, 20% run
                            controller.set_max_speed(speed);

                            println!("    ✓ AI controller started (speed: {:.1} m/s)", speed);
                            walker_controllers.push(controller);
                        }
                    }
                    Err(e) => {
                        eprintln!("    ✗ Failed to spawn AI controller: {}", e);
                    }
                }

                walkers.push(walker_actor);
            }
            Err(e) => {
                eprintln!("  ✗ Failed to spawn walker {}: {}", i + 1, e);
            }
        }
    }
    println!("✓ Spawned {} walkers with AI controllers\n", walkers.len());

    // Step 6: Run simulation
    println!("Step 6: Running simulation for 30 seconds...");
    println!(
        "  Total actors: {} vehicles + {} walkers = {} actors\n",
        vehicles.len(),
        walkers.len(),
        vehicles.len() + walkers.len()
    );

    for i in 1..=30 {
        world.tick();
        thread::sleep(Duration::from_millis(50));

        if i % 5 == 0 {
            println!("  Running... {} seconds", i);
        }
    }

    // Step 7: Cleanup
    println!("\n=== Cleanup ===");

    println!(
        "Destroying {} walker controllers...",
        walker_controllers.len()
    );
    for (i, controller) in walker_controllers.iter().enumerate() {
        if controller.destroy() {
            println!("  ✓ Controller {} destroyed", i + 1);
        }
    }

    println!("Destroying {} walkers...", walkers.len());
    for (i, walker) in walkers.iter().enumerate() {
        if walker.destroy() {
            println!("  ✓ Walker {} destroyed", i + 1);
        }
    }

    println!("Destroying {} vehicles...", vehicles.len());
    for (i, vehicle) in vehicles.iter().enumerate() {
        if vehicle.destroy() {
            println!("  ✓ Vehicle {} destroyed", i + 1);
        }
    }

    // Reset settings
    let mut settings = world.settings();
    settings.synchronous_mode = false;
    settings.fixed_delta_seconds = None;
    world.apply_settings(&settings, Duration::from_secs(5));

    println!("\n=== Traffic Generation Complete ===");
    println!("Successfully spawned and cleaned up:");
    println!("  - {} vehicles with autopilot", vehicles.len());
    println!("  - {} pedestrians with AI navigation", walkers.len());

    Ok(())
}
