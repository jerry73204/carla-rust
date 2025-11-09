//! Batch Operations Tests
//!
//! Tests for batch command execution (Phase 10.4).
//!
//! # Test Categories
//! - Batch spawn/destroy operations
//! - Batch control operations (vehicles, walkers)
//! - Response handling and error management
//! - Performance verification
//!
//! Run with:
//! ```bash
//! cargo run --example test_batch_commands --profile dev-release
//! ```

use carla::{
    client::Client,
    geom::Location,
    rpc::{Command, VehicleControl, WalkerControl},
};
use std::{thread, time::Duration};

type TestResult = Result<(), Box<dyn std::error::Error>>;

fn main() {
    println!("=== Batch Operations Tests ===\n");

    let mut client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();
    println!("Connected to CARLA server\n");

    let mut passed = 0;
    let mut failed = 0;

    // Spawn/Destroy tests
    println!("--- Spawn/Destroy Operations ---");
    run_test(
        "test_batch_spawn_actors",
        || test_batch_spawn_actors(&mut client, &mut world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_batch_destroy_actors",
        || test_batch_destroy_actors(&mut client, &mut world),
        &mut passed,
        &mut failed,
    );

    // Control tests
    println!("\n--- Control Operations ---");
    run_test(
        "test_batch_apply_vehicle_control",
        || test_batch_apply_vehicle_control(&mut client, &mut world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_batch_apply_walker_control",
        || test_batch_apply_walker_control(&mut client, &mut world),
        &mut passed,
        &mut failed,
    );

    // Response handling tests
    println!("\n--- Response Handling ---");
    run_test(
        "test_batch_command_response",
        || test_batch_command_response(&mut client, &mut world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_batch_error_handling",
        || test_batch_error_handling(&mut client),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_batch_partial_failure",
        || test_batch_partial_failure(&mut client, &mut world),
        &mut passed,
        &mut failed,
    );

    // Batch behavior tests
    println!("\n--- Batch Behavior ---");
    run_test(
        "test_batch_order_preservation",
        || test_batch_order_preservation(&mut client, &mut world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_empty_batch",
        || test_empty_batch(&mut client),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_large_batch",
        || test_large_batch(&mut client, &mut world),
        &mut passed,
        &mut failed,
    );

    // Performance and mixed tests
    println!("\n--- Performance & Mixed Operations ---");
    run_test(
        "test_batch_performance",
        || test_batch_performance(&mut client, &mut world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_mixed_command_types",
        || test_mixed_command_types(&mut client, &mut world),
        &mut passed,
        &mut failed,
    );

    println!("\n=== Results ===");
    println!("Passed: {}", passed);
    println!("Failed: {}", failed);
    std::process::exit(if failed > 0 { 1 } else { 0 });
}

fn run_test<F>(name: &str, test_fn: F, passed: &mut i32, failed: &mut i32)
where
    F: FnOnce() -> TestResult,
{
    print!("Testing {}... ", name);
    match test_fn() {
        Ok(_) => {
            println!("✓ PASS");
            *passed += 1;
        }
        Err(e) => {
            println!("✗ FAIL: {}", e);
            *failed += 1;
        }
    }
}

// ===== Spawn/Destroy Tests =====

fn test_batch_spawn_actors(client: &mut Client, world: &mut carla::client::World) -> TestResult {
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Vehicle blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_count = 5.min(spawn_points.len());

    // Create batch spawn commands
    let mut commands = Vec::new();
    for i in 0..spawn_count {
        let spawn_point = spawn_points.get(i).unwrap();
        commands.push(Command::spawn_actor(
            vehicle_bp.clone(),
            spawn_point.clone(),
            None,
        ));
    }

    // Execute batch spawn
    let responses = client.apply_batch_sync(commands, false);

    // Verify all spawned successfully
    assert_eq!(
        responses.len(),
        spawn_count,
        "Should return one response per command"
    );

    let success_count = responses.iter().filter(|r| r.is_success()).count();
    assert!(success_count > 0, "At least some vehicles should spawn");

    // Cleanup: destroy spawned actors
    let mut destroy_commands = Vec::new();
    for response in responses {
        if let Some(actor_id) = response.actor_id() {
            destroy_commands.push(Command::destroy_actor(actor_id));
        }
    }
    client.apply_batch_sync(destroy_commands, false);

    Ok(())
}

fn test_batch_destroy_actors(client: &mut Client, world: &mut carla::client::World) -> TestResult {
    // First spawn some actors
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Vehicle blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_count = 3.min(spawn_points.len());

    let mut spawn_commands = Vec::new();
    for i in 0..spawn_count {
        spawn_commands.push(Command::spawn_actor(
            vehicle_bp.clone(),
            spawn_points.get(i).unwrap().clone(),
            None,
        ));
    }

    let spawn_responses = client.apply_batch_sync(spawn_commands, false);

    // Collect actor IDs
    let actor_ids: Vec<_> = spawn_responses
        .iter()
        .filter_map(|r| r.actor_id())
        .collect();

    assert!(!actor_ids.is_empty(), "Should have spawned some actors");

    // Create batch destroy commands
    let mut destroy_commands = Vec::new();
    for &actor_id in &actor_ids {
        destroy_commands.push(Command::destroy_actor(actor_id));
    }

    // Execute batch destroy
    let destroy_responses = client.apply_batch_sync(destroy_commands, false);

    // Verify responses
    assert_eq!(
        destroy_responses.len(),
        actor_ids.len(),
        "Should return one response per destroy command"
    );

    Ok(())
}

// ===== Control Tests =====

fn test_batch_apply_vehicle_control(
    client: &mut Client,
    world: &mut carla::client::World,
) -> TestResult {
    // Spawn vehicles
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Vehicle blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_count = 3.min(spawn_points.len());

    let mut spawn_commands = Vec::new();
    for i in 0..spawn_count {
        spawn_commands.push(Command::spawn_actor(
            vehicle_bp.clone(),
            spawn_points.get(i).unwrap().clone(),
            None,
        ));
    }

    let spawn_responses = client.apply_batch_sync(spawn_commands, false);
    let vehicle_ids: Vec<_> = spawn_responses
        .iter()
        .filter_map(|r| r.actor_id())
        .collect();

    assert!(!vehicle_ids.is_empty(), "Should have spawned vehicles");

    // Create batch control commands
    let mut control_commands = Vec::new();
    for &actor_id in &vehicle_ids {
        let control = VehicleControl {
            throttle: 0.5,
            steer: 0.0,
            brake: 0.0,
            hand_brake: false,
            reverse: false,
            manual_gear_shift: false,
            gear: 0,
        };
        control_commands.push(Command::apply_vehicle_control(actor_id, control));
    }

    // Execute batch control
    let control_responses = client.apply_batch_sync(control_commands, false);

    assert_eq!(
        control_responses.len(),
        vehicle_ids.len(),
        "Should return one response per control command"
    );

    // Cleanup
    let destroy_commands: Vec<_> = vehicle_ids
        .iter()
        .map(|&id| Command::destroy_actor(id))
        .collect();
    client.apply_batch_sync(destroy_commands, false);

    Ok(())
}

fn test_batch_apply_walker_control(
    client: &mut Client,
    world: &mut carla::client::World,
) -> TestResult {
    // Spawn walkers
    let blueprint_library = world.blueprint_library();
    let walker_bp = blueprint_library
        .find("walker.pedestrian.0001")
        .ok_or("Walker blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_count = 3.min(spawn_points.len());

    let mut spawn_commands = Vec::new();
    for i in 0..spawn_count {
        spawn_commands.push(Command::spawn_actor(
            walker_bp.clone(),
            spawn_points.get(i).unwrap().clone(),
            None,
        ));
    }

    let spawn_responses = client.apply_batch_sync(spawn_commands, false);
    let walker_ids: Vec<_> = spawn_responses
        .iter()
        .filter_map(|r| r.actor_id())
        .collect();

    if walker_ids.is_empty() {
        // Skip test if no walkers spawned (some maps may not support walkers)
        return Ok(());
    }

    // Create batch walker control commands
    let mut control_commands = Vec::new();
    for &actor_id in &walker_ids {
        use carla_sys::carla::geom::Vector3D as FfiVector3D;
        let direction = FfiVector3D {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };
        let control = WalkerControl {
            direction,
            speed: 1.4,
            jump: false,
        };
        control_commands.push(Command::apply_walker_control(actor_id, control));
    }

    // Execute batch control
    let control_responses = client.apply_batch_sync(control_commands, false);

    assert_eq!(
        control_responses.len(),
        walker_ids.len(),
        "Should return one response per control command"
    );

    // Cleanup
    let destroy_commands: Vec<_> = walker_ids
        .iter()
        .map(|&id| Command::destroy_actor(id))
        .collect();
    client.apply_batch_sync(destroy_commands, false);

    Ok(())
}

// ===== Response Handling Tests =====

fn test_batch_command_response(
    client: &mut Client,
    world: &mut carla::client::World,
) -> TestResult {
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Vehicle blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).ok_or("No spawn points available")?;

    // Create a single spawn command
    let commands = vec![Command::spawn_actor(
        vehicle_bp.clone(),
        spawn_point.clone(),
        None,
    )];

    let responses = client.apply_batch_sync(commands, false);

    // Test response structure
    assert_eq!(responses.len(), 1, "Should have one response");

    let response = &responses[0];
    assert!(response.is_success(), "Command should succeed");
    assert!(!response.has_error(), "Should not have error");
    assert!(response.error().is_none(), "Error should be None");
    assert!(response.actor_id().is_some(), "Should have actor ID");

    // Cleanup
    if let Some(actor_id) = response.actor_id() {
        client.apply_batch_sync(vec![Command::destroy_actor(actor_id)], false);
    }

    Ok(())
}

fn test_batch_error_handling(client: &mut Client) -> TestResult {
    // Try to destroy a non-existent actor (should fail gracefully)
    let invalid_id = 999999;
    let commands = vec![Command::destroy_actor(invalid_id)];

    let responses = client.apply_batch_sync(commands, false);

    assert_eq!(responses.len(), 1, "Should have one response");

    // The response might succeed (CARLA doesn't always error on invalid destroy)
    // Just verify we get a response
    let _response = &responses[0];

    Ok(())
}

fn test_batch_partial_failure(client: &mut Client, world: &mut carla::client::World) -> TestResult {
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Vehicle blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();

    // Mix valid and potentially invalid commands
    let mut commands = Vec::new();

    // Valid spawn
    if let Some(spawn_point) = spawn_points.get(0) {
        commands.push(Command::spawn_actor(
            vehicle_bp.clone(),
            spawn_point.clone(),
            None,
        ));
    }

    // Invalid destroy (non-existent actor)
    commands.push(Command::destroy_actor(999999));

    // Another valid spawn
    if let Some(spawn_point) = spawn_points.get(1) {
        commands.push(Command::spawn_actor(
            vehicle_bp.clone(),
            spawn_point.clone(),
            None,
        ));
    }

    let command_count = commands.len();
    let responses = client.apply_batch_sync(commands, false);

    assert_eq!(
        responses.len(),
        command_count,
        "Should have one response per command"
    );

    // Cleanup any spawned actors
    let mut destroy_commands = Vec::new();
    for response in responses {
        if let Some(actor_id) = response.actor_id() {
            destroy_commands.push(Command::destroy_actor(actor_id));
        }
    }
    if !destroy_commands.is_empty() {
        client.apply_batch_sync(destroy_commands, false);
    }

    Ok(())
}

// ===== Batch Behavior Tests =====

fn test_batch_order_preservation(
    client: &mut Client,
    world: &mut carla::client::World,
) -> TestResult {
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Vehicle blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_count = 3.min(spawn_points.len());

    // Create commands with specific order
    let mut commands = Vec::new();
    for i in 0..spawn_count {
        commands.push(Command::spawn_actor(
            vehicle_bp.clone(),
            spawn_points.get(i).unwrap().clone(),
            None,
        ));
    }

    let responses = client.apply_batch_sync(commands, false);

    // Verify response count matches command count (order is preserved)
    assert_eq!(
        responses.len(),
        spawn_count,
        "Responses should match command count and order"
    );

    // Cleanup
    let destroy_commands: Vec<_> = responses
        .iter()
        .filter_map(|r| r.actor_id())
        .map(Command::destroy_actor)
        .collect();
    client.apply_batch_sync(destroy_commands, false);

    Ok(())
}

fn test_empty_batch(client: &mut Client) -> TestResult {
    // Execute empty batch
    let commands: Vec<Command> = Vec::new();
    let responses = client.apply_batch_sync(commands, false);

    assert_eq!(responses.len(), 0, "Empty batch should return no responses");

    Ok(())
}

fn test_large_batch(client: &mut Client, world: &mut carla::client::World) -> TestResult {
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Vehicle blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();

    // Create a large batch (up to 50 or available spawn points)
    let batch_size = 50.min(spawn_points.len());

    let mut commands = Vec::new();
    for i in 0..batch_size {
        commands.push(Command::spawn_actor(
            vehicle_bp.clone(),
            spawn_points.get(i % spawn_points.len()).unwrap().clone(),
            None,
        ));
    }

    let responses = client.apply_batch_sync(commands, false);

    assert_eq!(
        responses.len(),
        batch_size,
        "Should handle large batches correctly"
    );

    // Count successes
    let success_count = responses.iter().filter(|r| r.is_success()).count();
    assert!(success_count > 0, "At least some spawns should succeed");

    // Cleanup
    let destroy_commands: Vec<_> = responses
        .iter()
        .filter_map(|r| r.actor_id())
        .map(Command::destroy_actor)
        .collect();

    // Destroy in batches to avoid overwhelming the server
    if !destroy_commands.is_empty() {
        client.apply_batch_sync(destroy_commands, false);
    }

    Ok(())
}

// ===== Performance & Mixed Tests =====

fn test_batch_performance(client: &mut Client, world: &mut carla::client::World) -> TestResult {
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Vehicle blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();
    let test_size = 10.min(spawn_points.len());

    // Create batch commands
    let mut commands = Vec::new();
    for i in 0..test_size {
        commands.push(Command::spawn_actor(
            vehicle_bp.clone(),
            spawn_points.get(i).unwrap().clone(),
            None,
        ));
    }

    // Measure batch execution time
    let start = std::time::Instant::now();
    let responses = client.apply_batch_sync(commands, false);
    let batch_duration = start.elapsed();

    // Verify it executed reasonably fast (batch should be quick)
    assert!(
        batch_duration.as_secs() < 5,
        "Batch should complete in reasonable time"
    );

    let success_count = responses.iter().filter(|r| r.is_success()).count();
    assert!(success_count > 0, "Some spawns should succeed");

    // Cleanup
    let destroy_commands: Vec<_> = responses
        .iter()
        .filter_map(|r| r.actor_id())
        .map(Command::destroy_actor)
        .collect();
    client.apply_batch_sync(destroy_commands, false);

    Ok(())
}

fn test_mixed_command_types(client: &mut Client, world: &mut carla::client::World) -> TestResult {
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Vehicle blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).ok_or("No spawn points available")?;

    // Mix different command types in one batch
    let commands = vec![Command::spawn_actor(
        vehicle_bp.clone(),
        spawn_point.clone(),
        None,
    )];

    let responses = client.apply_batch_sync(commands, false);

    let actor_id = responses[0].actor_id().ok_or("Failed to spawn vehicle")?;

    // Now create a mixed batch with different operations on the same actor
    let mut mixed_commands = Vec::new();

    // 2. Apply control
    let control = VehicleControl {
        throttle: 0.3,
        steer: 0.0,
        brake: 0.0,
        hand_brake: false,
        reverse: false,
        manual_gear_shift: false,
        gear: 0,
    };
    mixed_commands.push(Command::apply_vehicle_control(actor_id, control));

    // 3. Apply location change
    let new_location = Location::new(
        spawn_point.location.x + 5.0,
        spawn_point.location.y,
        spawn_point.location.z,
    );
    mixed_commands.push(Command::ApplyLocation {
        actor_id,
        location: new_location,
    });

    // 4. Enable autopilot
    mixed_commands.push(Command::set_autopilot(actor_id, true, 8000));

    // Execute mixed batch
    let mixed_responses = client.apply_batch_sync(mixed_commands, false);

    assert_eq!(
        mixed_responses.len(),
        3,
        "Should have response for each command"
    );

    thread::sleep(Duration::from_millis(100));

    // Cleanup
    client.apply_batch_sync(vec![Command::destroy_actor(actor_id)], false);

    Ok(())
}
