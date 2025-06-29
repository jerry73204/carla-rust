//! Test example for defensive actor operations.
//!
//! This example demonstrates how to use the enhanced Actor with defensive
//! operations to prevent crashes when accessing actor properties in CARLA 0.10.0.

use anyhow::Result;
use carla::{
    actor::ActorExt,
    client::Client,
    geom::{Location, Rotation, Transform},
};

fn main() -> Result<()> {
    env_logger::init();

    println!("CARLA Defensive Actor Test Example");
    println!("==================================");
    println!("Testing defensive actor operations to prevent crashes\n");

    // Connect to CARLA
    let client = Client::new("localhost", 2000, None)?;
    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Get a vehicle blueprint
    let vehicle_bp = blueprint_library
        .find("vehicle.mini.cooper")?
        .ok_or_else(|| anyhow::anyhow!("Vehicle blueprint not found"))?;

    // Create spawn point
    let spawn_transform = Transform {
        location: Location {
            x: 10.0,
            y: 10.0,
            z: 0.5,
        },
        rotation: Rotation {
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
        },
    };

    // Test 1: Spawn with retry logic
    println!("Test 1: Spawning vehicle with retry logic...");
    let actor = match world.spawn_actor_with_retry(&vehicle_bp, &spawn_transform, None, 3) {
        Ok(actor) => {
            println!("✓ Successfully spawned actor with ID: {}", actor.id());
            actor
        }
        Err(e) => {
            println!("✗ Failed to spawn actor: {e}");
            return Ok(());
        }
    };

    // Test 2: Safe property access
    println!("\nTest 2: Safe property access...");

    // Try to get transform safely
    match actor.try_transform() {
        Ok(transform) => {
            println!(
                "✓ Transform: Location({:.2}, {:.2}, {:.2})",
                transform.location.x, transform.location.y, transform.location.z
            );
        }
        Err(e) => {
            println!("✗ Failed to get transform: {e}");
        }
    }

    // Try to get velocity safely
    match actor.try_velocity() {
        Ok(velocity) => {
            println!(
                "✓ Velocity: ({:.2}, {:.2}, {:.2})",
                velocity.x, velocity.y, velocity.z
            );
        }
        Err(e) => {
            println!("✗ Failed to get velocity: {e}");
        }
    }

    // Try to get bounding box safely
    match actor.try_bounding_box() {
        Ok(bbox) => {
            println!(
                "✓ Bounding box: extent({:.2}, {:.2}, {:.2})",
                bbox.extent.x, bbox.extent.y, bbox.extent.z
            );
        }
        Err(e) => {
            println!("✗ Failed to get bounding box: {e}");
        }
    }

    // Test 3: Validation checks
    println!("\nTest 3: Actor validation...");
    println!("Is actor valid? {}", actor.is_valid());
    println!("Actor state: {:?}", actor.actor_state());

    // Test 4: Safe physics operations
    println!("\nTest 4: Safe physics operations...");
    match actor.set_simulate_physics(true) {
        Ok(_) => println!("✓ Enabled physics simulation"),
        Err(e) => println!("✗ Failed to enable physics: {e}"),
    }

    // Add a small impulse
    let impulse = carla::geom::Vector3D {
        x: 100.0,
        y: 0.0,
        z: 0.0,
    };
    match actor.add_impulse(&impulse) {
        Ok(_) => println!("✓ Applied impulse"),
        Err(e) => println!("✗ Failed to apply impulse: {e}"),
    }

    // Test 5: Destruction with enhanced error handling
    println!("\nTest 5: Safe actor destruction...");
    let mut actor = actor; // Make mutable for destroy
    match actor.destroy() {
        Ok(_) => println!("✓ Successfully destroyed actor"),
        Err(e) => println!("✗ Failed to destroy actor: {e}"),
    }

    // Test 6: Operations after destruction should fail gracefully
    println!("\nTest 6: Operations after destruction...");
    match actor.try_transform() {
        Ok(_) => println!("✗ Unexpected: Got transform after destruction"),
        Err(e) => println!("✓ Expected error: {e}"),
    }

    println!("\nDefensive actor tests completed!");
    println!("\nNote: The enhanced Actor helps prevent crashes but CARLA 0.10.0");
    println!("server may still crash in some scenarios. Always check");
    println!("actor validity before operations.");

    Ok(())
}
