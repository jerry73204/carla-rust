//! BasicAgent demonstration.
//!
//! This example demonstrates the BasicAgent navigating to a destination.
//!
//! # Usage
//! ```bash
//! cargo run --example basic_agent_demo --profile dev-release
//! ```

use anyhow::Result;
use carla::{
    agents::navigation::{BasicAgent, BasicAgentConfig},
    client::{ActorBase, Client},
};

fn main() -> Result<()> {
    println!("BasicAgent Demo - Connecting to CARLA...");

    // Connect to CARLA
    let client = Client::connect("localhost", 2000, 0);
    let mut world = client.world();

    println!("Connected! Getting spawn points...");

    // Get spawn points
    let map = world.map();
    let spawn_points = map.recommended_spawn_points();

    if spawn_points.len() < 2 {
        return Err(anyhow::anyhow!("Need at least 2 spawn points"));
    }

    // Spawn a vehicle
    println!("Spawning vehicle...");
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .filter("vehicle.*")
        .iter()
        .next()
        .ok_or_else(|| anyhow::anyhow!("No vehicle blueprints found"))?;

    let spawn_point = &spawn_points.as_slice()[0].to_na();
    let vehicle = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle = carla::client::Vehicle::try_from(vehicle)
        .map_err(|_| anyhow::anyhow!("Failed to cast to Vehicle"))?;

    println!("Spawned vehicle: {}", vehicle.type_id());

    // Create BasicAgent
    println!("Creating BasicAgent...");
    let config = BasicAgentConfig::default();
    let mut agent = BasicAgent::new(vehicle.clone(), config, None, None)?;

    // Set destination to another spawn point
    let dest_transform = &spawn_points.as_slice()[1];
    let dest_isometry = dest_transform.to_na();
    let destination = carla::geom::Location::from_na_translation(&dest_isometry.translation);
    println!(
        "Setting destination to ({:.1}, {:.1}, {:.1})",
        destination.x, destination.y, destination.z
    );

    agent.set_destination(destination, None, true)?;

    println!("Starting navigation loop...");
    let mut step_count = 0;
    let max_steps = 1000;

    while !agent.done() && step_count < max_steps {
        step_count += 1;

        // Tick the world
        world.tick();

        // Get control from agent
        let control = agent.run_step_debug(step_count % 100 == 0)?;

        // Apply control to vehicle
        vehicle.apply_control(&control);

        if step_count % 100 == 0 {
            let isometry = vehicle.transform();
            let location = carla::geom::Location::from_na_translation(&isometry.translation);
            println!(
                "Step {}: Location ({:.1}, {:.1}, {:.1}), throttle={:.2}, brake={:.2}, steer={:.2}",
                step_count,
                location.x,
                location.y,
                location.z,
                control.throttle,
                control.brake,
                control.steer
            );
        }
    }

    if agent.done() {
        println!("SUCCESS! Agent reached destination in {} steps", step_count);
    } else {
        println!(
            "Timeout after {} steps (destination may still be reachable)",
            step_count
        );
    }

    // Cleanup
    println!("Destroying vehicle...");
    vehicle.destroy();

    println!("Done!");
    Ok(())
}
