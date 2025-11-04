//! BehaviorAgent Demo
//!
//! This example demonstrates the BehaviorAgent with different behavior profiles.
//!
//! # Usage
//! ```bash
//! cargo run --example behavior_agent_demo --profile dev-release
//! ```

use anyhow::Result;
use carla::{
    agents::navigation::{BehaviorAgent, BehaviorAgentConfig, BehaviorType},
    client::{ActorBase, Client},
    geom::Location,
};

fn main() -> Result<()> {
    println!("BehaviorAgent Demo - Connecting to CARLA...");

    // Connect to CARLA
    let client = Client::connect("localhost", 2000, 0);
    let mut world = client.world();

    println!("Connected! Spawning vehicle...");

    // Get spawn points
    let map = world.map();
    let spawn_points = map.recommended_spawn_points();

    if spawn_points.len() < 2 {
        return Err(anyhow::anyhow!("Need at least 2 spawn points"));
    }

    // Spawn a vehicle
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .filter("vehicle.tesla.model3")
        .iter()
        .next()
        .unwrap_or_else(|| {
            blueprint_library
                .filter("vehicle.*")
                .iter()
                .next()
                .expect("No vehicle blueprints found")
        });

    let spawn_point = &spawn_points.as_slice()[0];
    let vehicle = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle = carla::client::Vehicle::try_from(vehicle)
        .map_err(|_| anyhow::anyhow!("Failed to cast to Vehicle"))?;

    println!("Spawned vehicle: {}", vehicle.type_id());

    // Create BehaviorAgent with Normal behavior
    println!("\n=== Testing Normal Behavior ===");
    let config = BehaviorAgentConfig {
        behavior: BehaviorType::normal(),
        ..Default::default()
    };
    let mut agent = BehaviorAgent::new(vehicle.clone(), config, None, None)?;

    // Set destination to second spawn point
    let destination_isometry = &spawn_points.as_slice()[1].to_na();
    let destination = Location::from_na_translation(&destination_isometry.translation);

    println!(
        "Setting destination: ({:.1}, {:.1}, {:.1})",
        destination.x, destination.y, destination.z
    );
    agent.set_destination(destination, None, true)?;

    // Run agent for a few steps
    println!("\nRunning agent with Normal behavior...");
    for step in 0..50 {
        if agent.done() {
            println!("Agent reached destination!");
            break;
        }

        // Execute one navigation step
        let control = agent.run_step()?;

        // Apply control to vehicle
        vehicle.apply_control(&control);

        // Tick the world
        world.tick();

        if step % 10 == 0 {
            println!(
                "Step {}: throttle={:.2}, steer={:.2}, brake={:.2}",
                step, control.throttle, control.steer, control.brake
            );
        }
    }

    // Test Cautious behavior
    println!("\n=== Testing Cautious Behavior ===");
    let config_cautious = BehaviorAgentConfig {
        behavior: BehaviorType::cautious(),
        ..Default::default()
    };
    let mut cautious_agent = BehaviorAgent::new(vehicle.clone(), config_cautious, None, None)?;
    cautious_agent.set_destination(destination, None, true)?;

    println!(
        "Cautious behavior: max_speed={:.1} km/h",
        cautious_agent
            .run_step()
            .map(|_| BehaviorType::cautious().params().max_speed)
            .unwrap_or(0.0)
    );

    // Test Aggressive behavior
    println!("\n=== Testing Aggressive Behavior ===");
    let config_aggressive = BehaviorAgentConfig {
        behavior: BehaviorType::aggressive(),
        ..Default::default()
    };
    let mut aggressive_agent = BehaviorAgent::new(vehicle.clone(), config_aggressive, None, None)?;
    aggressive_agent.set_destination(destination, None, true)?;

    println!(
        "Aggressive behavior: max_speed={:.1} km/h",
        aggressive_agent
            .run_step()
            .map(|_| BehaviorType::aggressive().params().max_speed)
            .unwrap_or(0.0)
    );

    // Compare behavior profiles
    println!("\n=== Behavior Profile Comparison ===");
    println!("Cautious:");
    let cautious_behavior = BehaviorType::cautious();
    let cautious_params = cautious_behavior.params();
    println!("  Max speed: {:.1} km/h", cautious_params.max_speed);
    println!("  Safety time: {:.1}s", cautious_params.safety_time);
    println!(
        "  Min proximity: {:.1}m",
        cautious_params.min_proximity_threshold
    );

    println!("\nNormal:");
    let normal_behavior = BehaviorType::normal();
    let normal_params = normal_behavior.params();
    println!("  Max speed: {:.1} km/h", normal_params.max_speed);
    println!("  Safety time: {:.1}s", normal_params.safety_time);
    println!(
        "  Min proximity: {:.1}m",
        normal_params.min_proximity_threshold
    );

    println!("\nAggressive:");
    let aggressive_behavior = BehaviorType::aggressive();
    let aggressive_params = aggressive_behavior.params();
    println!("  Max speed: {:.1} km/h", aggressive_params.max_speed);
    println!("  Safety time: {:.1}s", aggressive_params.safety_time);
    println!(
        "  Min proximity: {:.1}m",
        aggressive_params.min_proximity_threshold
    );

    // Cleanup
    println!("\nCleaning up...");
    vehicle.destroy();

    println!("\n✅ SUCCESS: BehaviorAgent demo completed!");
    Ok(())
}
