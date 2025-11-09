//! Custom behavior profile demonstration.
//!
//! This example demonstrates how to create and use custom behavior profiles
//! with BehaviorAgent for fine-tuned autonomous driving control.
//!
//! # Usage
//! ```bash
//! cargo run --example custom_behavior_demo --profile dev-release
//! ```

use anyhow::Result;
use carla::{
    agents::navigation::{BehaviorAgent, BehaviorAgentConfig, BehaviorParams, BehaviorType},
    client::{ActorBase, Client},
};

fn main() -> Result<()> {
    println!("Custom Behavior Profile Demo - Connecting to CARLA...");

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
        .filter("vehicle.*")
        .iter()
        .next()
        .ok_or_else(|| anyhow::anyhow!("No vehicle blueprints found"))?;

    let spawn_point = &spawn_points.as_slice()[0];
    let vehicle = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle = carla::client::Vehicle::try_from(vehicle)
        .map_err(|_| anyhow::anyhow!("Failed to cast to Vehicle"))?;

    println!("Spawned vehicle: {}", vehicle.type_id());

    // ========================================================================
    // Example 1: Super Cautious Profile (custom parameters)
    // ========================================================================
    println!("\n=== Example 1: Super Cautious Custom Profile ===");

    let super_cautious = BehaviorType::custom(BehaviorParams {
        max_speed: 30.0,               // Very low max speed
        speed_lim_dist: 10.0,          // Large speed limit distance multiplier
        speed_decrease: 15.0,          // Slow speed decrease
        safety_time: 4.0,              // Very long safety time (4 seconds)
        min_proximity_threshold: 15.0, // Large proximity threshold (15 meters)
        braking_distance: 8.0,         // Long braking distance
        tailgate_counter: -1,          // Never tailgate
    });

    println!("Super Cautious Profile:");
    println!("  Max Speed: {} km/h", super_cautious.params().max_speed);
    println!("  Safety Time: {} s", super_cautious.params().safety_time);
    println!(
        "  Proximity Threshold: {} m",
        super_cautious.params().min_proximity_threshold
    );

    let config = BehaviorAgentConfig {
        behavior: super_cautious,
        ..Default::default()
    };

    let mut agent = BehaviorAgent::new(vehicle.clone(), config, None, None)?;

    // Set destination
    let dest_transform = &spawn_points.as_slice()[1];
    let dest_isometry = dest_transform.to_na();
    let destination = carla::geom::Location::from_na_translation(&dest_isometry.translation);

    agent.set_destination(destination, None, true)?;

    println!("Running super cautious agent for 100 steps...");
    for i in 0..100 {
        world.tick();
        let control = agent.run_step()?;
        vehicle.apply_control(&control);

        if i % 50 == 0 {
            println!(
                "  Step {}: throttle={:.2}, brake={:.2}",
                i, control.throttle, control.brake
            );
        }
    }

    // ========================================================================
    // Example 2: Sporty/Performance Profile (custom parameters)
    // ========================================================================
    println!("\n=== Example 2: Sporty/Performance Custom Profile ===");

    let sporty = BehaviorType::custom(BehaviorParams {
        max_speed: 80.0,              // High max speed
        speed_lim_dist: 0.5,          // Tight speed limit distance
        speed_decrease: 6.0,          // Fast speed decrease
        safety_time: 0.8,             // Short safety time
        min_proximity_threshold: 6.0, // Small proximity threshold
        braking_distance: 3.0,        // Short braking distance
        tailgate_counter: -2,         // Tailgate aggressively
    });

    println!("Sporty Profile:");
    println!("  Max Speed: {} km/h", sporty.params().max_speed);
    println!("  Safety Time: {} s", sporty.params().safety_time);
    println!(
        "  Proximity Threshold: {} m",
        sporty.params().min_proximity_threshold
    );

    let config = BehaviorAgentConfig {
        behavior: sporty,
        ..Default::default()
    };

    let mut agent = BehaviorAgent::new(vehicle.clone(), config, None, None)?;
    agent.set_destination(destination, None, true)?;

    println!("Running sporty agent for 100 steps...");
    for i in 0..100 {
        world.tick();
        let control = agent.run_step()?;
        vehicle.apply_control(&control);

        if i % 50 == 0 {
            println!(
                "  Step {}: throttle={:.2}, brake={:.2}",
                i, control.throttle, control.brake
            );
        }
    }

    // ========================================================================
    // Example 3: Runtime Behavior Modification
    // ========================================================================
    println!("\n=== Example 3: Runtime Behavior Modification ===");

    // Start with normal behavior
    let mut behavior = BehaviorType::normal();
    println!("Starting with Normal profile:");
    println!("  Initial Max Speed: {} km/h", behavior.params().max_speed);

    // Modify parameters at runtime
    behavior.params_mut().max_speed = 35.0;
    behavior.params_mut().safety_time = 2.5;

    println!("Modified to Custom profile:");
    println!("  New Max Speed: {} km/h", behavior.params().max_speed);
    println!("  New Safety Time: {} s", behavior.params().safety_time);

    let config = BehaviorAgentConfig {
        behavior,
        ..Default::default()
    };

    let mut agent = BehaviorAgent::new(vehicle.clone(), config, None, None)?;
    agent.set_destination(destination, None, true)?;

    println!("Running modified agent for 100 steps...");
    for i in 0..100 {
        world.tick();
        let control = agent.run_step()?;
        vehicle.apply_control(&control);

        if i % 50 == 0 {
            println!(
                "  Step {}: throttle={:.2}, brake={:.2}",
                i, control.throttle, control.brake
            );
        }
    }

    // ========================================================================
    // Example 4: Comparing Built-in Profiles
    // ========================================================================
    println!("\n=== Example 4: Comparing Built-in Profiles ===");

    let profiles = vec![
        ("Cautious", BehaviorType::cautious()),
        ("Normal", BehaviorType::normal()),
        ("Aggressive", BehaviorType::aggressive()),
    ];

    for (name, profile) in profiles {
        let params = profile.params();
        println!("\n{} Profile:", name);
        println!("  Max Speed: {} km/h", params.max_speed);
        println!("  Safety Time: {} s", params.safety_time);
        println!("  Min Proximity: {} m", params.min_proximity_threshold);
        println!("  Braking Distance: {} m", params.braking_distance);
        println!("  Tailgate Counter: {}", params.tailgate_counter);
    }

    println!("\n=== Demo Complete ===");
    println!("Custom behavior profiles allow full control over:");
    println!("  - Speed limits and acceleration");
    println!("  - Safety margins and collision avoidance");
    println!("  - Following distance and tailgating behavior");
    println!("  - Braking characteristics");

    Ok(())
}
