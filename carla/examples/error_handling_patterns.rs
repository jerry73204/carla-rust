//! Demonstrates error handling patterns in the carla-rust library.
//!
//! This example shows how to:
//! - Handle different error types (Resource, Operation, Validation)
//! - Check error categories using helper methods
//! - Implement retry logic for recoverable errors
//! - Extract error details for logging and diagnostics
//! - Gracefully recover from errors
//!
//! Run with: `cargo run --example error_handling_patterns --profile dev-release`

use carla::{
    client::{ActorBase, Client},
    error::CarlaError,
    geom::{Location, Rotation, Transform},
};
use std::time::Duration;

fn main() {
    println!("=== CARLA Error Handling Patterns Demo ===\n");

    // Connect to CARLA server
    let client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();

    println!("Connected to CARLA server\n");

    // Demo 1: Resource not found errors
    demo_resource_errors(&mut world);

    // Demo 2: Operation errors with retry
    demo_operation_errors_with_retry(&mut world);

    // Demo 3: Validation errors
    demo_validation_errors(&mut world);

    // Demo 4: Error classification and logging
    demo_error_classification();

    println!("\n=== Error handling demo complete ===");
}

/// Demo 1: Handling resource not found errors
fn demo_resource_errors(world: &mut carla::client::World) {
    println!("--- Demo 1: Resource Not Found Errors ---");

    // Try to find a blueprint that doesn't exist
    match world.actor_builder("vehicle.nonexistent.model") {
        Ok(_) => println!("Found blueprint (unexpected)"),
        Err(e) => {
            if e.is_not_found() {
                println!("✓ Resource not found error: {}", e);
                println!("✓ Error is retriable: {}", e.is_retriable());

                // Try with a valid blueprint as fallback
                println!("Trying fallback blueprint...");
                match world.actor_builder("vehicle.tesla.model3") {
                    Ok(_) => println!("✓ Fallback blueprint found successfully"),
                    Err(e) => println!("Fallback also failed: {}", e),
                }
            }
        }
    }

    println!();
}

/// Demo 2: Handling operation errors with retry logic
fn demo_operation_errors_with_retry(world: &mut carla::client::World) {
    println!("--- Demo 2: Operation Errors with Retry ---");

    // Get a spawn point
    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = match spawn_points.get(0) {
        Some(p) => p.clone(),
        None => {
            println!("No spawn points available");
            println!();
            return;
        }
    };

    // Try to spawn with retry logic
    let blueprint = match world.blueprint_library().find("vehicle.tesla.model3") {
        Some(bp) => bp,
        None => {
            println!("Blueprint not found");
            println!();
            return;
        }
    };

    match spawn_with_retry(world, &blueprint, &spawn_point, 3) {
        Ok(actor) => {
            println!("✓ Successfully spawned actor: ID {}", actor.id());
            // Clean up
            let _ = actor.destroy();
        }
        Err(e) => {
            println!("✗ Failed to spawn after retries: {}", e);
        }
    }

    println!();
}

fn spawn_with_retry(
    world: &mut carla::client::World,
    blueprint: &carla::client::ActorBlueprint,
    transform: &Transform,
    max_retries: u32,
) -> Result<carla::client::Actor, CarlaError> {
    let mut attempts = 0;

    loop {
        attempts += 1;
        println!("Spawn attempt {}/{}...", attempts, max_retries);

        match world.spawn_actor(blueprint, transform) {
            Ok(actor) => {
                println!("✓ Spawn successful on attempt {}", attempts);
                return Ok(actor);
            }
            Err(e) => {
                if e.is_retriable() && attempts < max_retries {
                    println!("  Retriable error: {}. Retrying...", e);
                    std::thread::sleep(Duration::from_millis(100));
                    continue;
                } else {
                    println!("  Non-retriable error or max retries reached");
                    return Err(e);
                }
            }
        }
    }
}

/// Demo 3: Handling validation errors
fn demo_validation_errors(world: &mut carla::client::World) {
    println!("--- Demo 3: Validation Errors ---");

    // Try to set an invalid attribute
    match world
        .actor_builder("vehicle.tesla.model3")
        .and_then(|builder| builder.set_attribute("invalid_attribute", "some_value"))
    {
        Ok(_) => println!("Attribute set (unexpected)"),
        Err(e) => {
            if e.is_validation_error() {
                println!("✓ Validation error detected: {}", e);
                println!("  This indicates invalid input/configuration");
            }
        }
    }

    // Try to spawn a vehicle but use spawn_sensor method (type mismatch)
    let spawn_transform = Transform {
        location: Location::new(0.0, 0.0, 100.0),
        rotation: Rotation {
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
        },
    };

    match world
        .actor_builder("vehicle.tesla.model3")
        .and_then(|builder| builder.spawn_sensor(spawn_transform))
    {
        Ok(_) => println!("Spawned sensor (unexpected)"),
        Err(e) => {
            if e.is_validation_error() {
                println!("✓ Type mismatch validation error: {}", e);
            }
        }
    }

    println!();
}

/// Demo 4: Error classification and diagnostic logging
fn demo_error_classification() {
    println!("--- Demo 4: Error Classification ---");

    // Demonstrate error classification helpers
    let errors = vec![
        // Simulated errors for demonstration
        "connection timeout",
        "blueprint not found",
        "spawn failed",
        "invalid configuration",
    ];

    for error_msg in errors {
        println!("Classifying: \"{}\"", error_msg);
        println!("  (In real code, use error.is_*() methods)");
    }

    println!("\nError helper methods available:");
    println!("  - is_connection_error() - Network/server issues");
    println!("  - is_timeout() - Operation timeout");
    println!("  - is_not_found() - Resource doesn't exist");
    println!("  - is_validation_error() - Invalid input");
    println!("  - is_operation_error() - Operation failed");
    println!("  - is_retriable() - Can be retried");

    println!();
}

/// Extension trait to add validation error check
trait ErrorExt {
    fn is_validation_error(&self) -> bool;
}

impl ErrorExt for CarlaError {
    fn is_validation_error(&self) -> bool {
        matches!(self, CarlaError::Validation(_))
    }
}
