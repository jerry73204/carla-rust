//! ConstantVelocityAgent Demo
//!
//! This example demonstrates the ConstantVelocityAgent maintaining constant speed.
//!
//! # Usage
//! ```bash
//! cargo run --example constant_velocity_agent_demo --profile dev-release
//! ```

use anyhow::Result;
use carla::{
    agents::navigation::{ConstantVelocityAgent, ConstantVelocityAgentConfig},
    client::{ActorBase, Client},
    geom::Location,
};

fn main() -> Result<()> {
    println!("ConstantVelocityAgent Demo - Connecting to CARLA...");

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

    let spawn_point = &spawn_points.as_slice()[0].to_na();
    let vehicle = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle = carla::client::Vehicle::try_from(vehicle)
        .map_err(|_| anyhow::anyhow!("Failed to cast to Vehicle"))?;

    println!("Spawned vehicle: {}", vehicle.type_id());

    // Create ConstantVelocityAgent
    println!("\n=== Testing Constant Velocity Mode ===");
    let config = ConstantVelocityAgentConfig {
        target_speed: 10.0, // 10 m/s = 36 km/h
        use_basic_behavior: false,
        ..Default::default()
    };
    let mut agent = ConstantVelocityAgent::new(vehicle.clone(), config, None, None)?;

    // Set destination to second spawn point
    let destination_isometry = &spawn_points.as_slice()[1].to_na();
    let destination = Location::from_na_translation(&destination_isometry.translation);

    println!(
        "Setting destination: ({:.1}, {:.1}, {:.1})",
        destination.x, destination.y, destination.z
    );
    agent.set_destination(destination, None, true)?;

    println!("Target constant speed: 10 m/s (36 km/h)");

    // Run agent for a few steps
    println!("\nRunning agent in constant velocity mode...");
    for step in 0..100 {
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

        if step % 20 == 0 {
            let speed = carla::agents::tools::get_speed(&vehicle);
            println!(
                "Step {}: speed={:.1} km/h, throttle={:.2}, steer={:.2}",
                step, speed, control.throttle, control.steer
            );
        }
    }

    // Test changing speed
    println!("\n=== Testing Speed Change ===");
    agent.set_constant_velocity(5.0); // Change to 5 m/s = 18 km/h
    println!("Changed target speed to: 5 m/s (18 km/h)");

    for step in 0..20 {
        if agent.done() {
            break;
        }

        let control = agent.run_step()?;
        vehicle.apply_control(&control);
        world.tick();

        if step % 5 == 0 {
            let speed = carla::agents::tools::get_speed(&vehicle);
            println!(
                "Step {}: speed={:.1} km/h, throttle={:.2}",
                step, speed, control.throttle
            );
        }
    }

    // Test basic behavior mode
    println!("\n=== Testing Basic Behavior Mode ===");
    let config_basic = ConstantVelocityAgentConfig {
        target_speed: 10.0,
        use_basic_behavior: true, // Enable hazard detection
        ..Default::default()
    };
    let mut basic_agent = ConstantVelocityAgent::new(vehicle.clone(), config_basic, None, None)?;
    basic_agent.set_destination(destination, None, true)?;

    println!("Running with basic behavior (hazard detection enabled)...");
    for step in 0..20 {
        if basic_agent.done() {
            break;
        }

        let control = basic_agent.run_step()?;
        vehicle.apply_control(&control);
        world.tick();

        if step % 5 == 0 {
            println!(
                "Step {}: throttle={:.2}, brake={:.2}",
                step, control.throttle, control.brake
            );
        }
    }

    // Demonstrate ignoring vehicles
    println!("\n=== Testing Ignore Vehicles ===");
    agent.ignore_vehicles(true);
    println!("Vehicle obstacle detection disabled");

    for _step in 0..10 {
        if agent.done() {
            break;
        }

        let control = agent.run_step()?;
        vehicle.apply_control(&control);
        world.tick();
    }

    // Cleanup
    println!("\nCleaning up...");
    vehicle.destroy();

    println!("\n✅ SUCCESS: ConstantVelocityAgent demo completed!");
    Ok(())
}
