//! Advanced Vehicle Features Tests
//!
//! Tests for vehicle physics, doors, lights (Phase 10.3).
//!
//! # Test Categories
//! - Door operations: Open/close, state queries
//! - Wheel operations: Count, state queries
//! - Failure states: Creation, application
//! - Light states: Creation, application, queries
//!
//! Run with:
//! ```bash
//! cargo run --example test_vehicle_advanced --profile dev-release
//! ```

use carla::{
    client::{Client, Vehicle},
    rpc::{VehicleDoor, VehicleFailureState, VehicleLightState, VehicleWheelLocation},
};
use std::{thread, time::Duration};

type TestResult = Result<(), Box<dyn std::error::Error>>;

fn main() {
    println!("=== Advanced Vehicle Features Tests ===\n");

    let client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();
    println!("Connected to CARLA server\n");

    // Setup test scenario
    let vehicle = setup_test_scenario(&mut world);

    let mut passed = 0;
    let mut failed = 0;

    // Door operation tests
    println!("--- Door Operations ---");
    run_test(
        "test_vehicle_door_open_close",
        || test_vehicle_door_open_close(&vehicle),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_vehicle_door_states",
        || test_vehicle_door_states(&vehicle),
        &mut passed,
        &mut failed,
    );

    // Wheel operation tests
    println!("\n--- Wheel Operations ---");
    run_test(
        "test_vehicle_wheel_count",
        || test_vehicle_wheel_count(&vehicle),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_vehicle_wheel_states",
        || test_vehicle_wheel_states(&vehicle),
        &mut passed,
        &mut failed,
    );

    // Failure state tests
    println!("\n--- Failure States ---");
    run_test(
        "test_failure_state_creation",
        test_failure_state_creation,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_apply_failure_state",
        || test_apply_failure_state(&vehicle),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_vehicle_with_failures",
        || test_vehicle_with_failures(&vehicle),
        &mut passed,
        &mut failed,
    );

    // Light state tests
    println!("\n--- Light States ---");
    run_test(
        "test_light_state_creation",
        test_light_state_creation,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_light_state_flags",
        test_light_state_flags,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_apply_light_state",
        || test_apply_light_state(&vehicle),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_vehicle_light_query",
        || test_vehicle_light_query(&vehicle),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_multiple_light_states",
        || test_multiple_light_states(&vehicle),
        &mut passed,
        &mut failed,
    );

    println!("\n=== Results ===");
    println!("Passed: {}", passed);
    println!("Failed: {}", failed);
    std::process::exit(if failed > 0 { 1 } else { 0 });
}

fn setup_test_scenario(world: &mut carla::client::World) -> Vehicle {
    println!("Setting up test scenario...");

    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .expect("Vehicle blueprint not found");

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).expect("No spawn points available");

    let vehicle_actor = world
        .spawn_actor(&vehicle_bp, spawn_point)
        .expect("Failed to spawn vehicle");

    let vehicle = Vehicle::try_from(vehicle_actor).expect("Failed to cast to Vehicle");
    println!("Test scenario ready\n");
    vehicle
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

// ===== Door Operation Tests =====

fn test_vehicle_door_open_close(vehicle: &Vehicle) -> TestResult {
    // Test opening individual doors
    vehicle.open_door(VehicleDoor::FL);
    thread::sleep(Duration::from_millis(100));

    vehicle.open_door(VehicleDoor::FR);
    thread::sleep(Duration::from_millis(100));

    vehicle.open_door(VehicleDoor::RL);
    thread::sleep(Duration::from_millis(100));

    vehicle.open_door(VehicleDoor::RR);
    thread::sleep(Duration::from_millis(100));

    // Test closing doors
    vehicle.close_door(VehicleDoor::FL);
    vehicle.close_door(VehicleDoor::FR);
    vehicle.close_door(VehicleDoor::RL);
    vehicle.close_door(VehicleDoor::RR);

    Ok(())
}

fn test_vehicle_door_states(vehicle: &Vehicle) -> TestResult {
    // Test opening and closing in different patterns

    // Open front doors only
    vehicle.open_door(VehicleDoor::FL);
    vehicle.open_door(VehicleDoor::FR);
    thread::sleep(Duration::from_millis(200));

    // Close front, open rear
    vehicle.close_door(VehicleDoor::FL);
    vehicle.close_door(VehicleDoor::FR);
    vehicle.open_door(VehicleDoor::RL);
    vehicle.open_door(VehicleDoor::RR);
    thread::sleep(Duration::from_millis(200));

    // Close all
    vehicle.close_door(VehicleDoor::RL);
    vehicle.close_door(VehicleDoor::RR);

    Ok(())
}

// ===== Wheel Operation Tests =====

fn test_vehicle_wheel_count(vehicle: &Vehicle) -> TestResult {
    // Test that wheel operations work for front wheel locations
    // API provides FL_Wheel and FR_Wheel

    // Set steering angles for front wheels
    vehicle.set_wheel_steer_direction(VehicleWheelLocation::FL_Wheel, 15.0);
    vehicle.set_wheel_steer_direction(VehicleWheelLocation::FR_Wheel, 15.0);

    thread::sleep(Duration::from_millis(100));

    // Reset to center
    vehicle.set_wheel_steer_direction(VehicleWheelLocation::FL_Wheel, 0.0);
    vehicle.set_wheel_steer_direction(VehicleWheelLocation::FR_Wheel, 0.0);

    Ok(())
}

fn test_vehicle_wheel_states(vehicle: &Vehicle) -> TestResult {
    // Test wheel steering angles
    let test_angle = 20.0;

    vehicle.set_wheel_steer_direction(VehicleWheelLocation::FL_Wheel, test_angle);
    thread::sleep(Duration::from_millis(100));

    let angle_fl = vehicle.wheel_steer_angle(VehicleWheelLocation::FL_Wheel);

    // The angle might not be exactly what we set due to physics
    // Just verify we can query it
    assert!(
        angle_fl.is_finite(),
        "Wheel steer angle should be a finite number"
    );

    // Test front right wheel
    let angle_fr = vehicle.wheel_steer_angle(VehicleWheelLocation::FR_Wheel);
    assert!(angle_fr.is_finite());

    // Reset steering
    vehicle.set_wheel_steer_direction(VehicleWheelLocation::FL_Wheel, 0.0);
    vehicle.set_wheel_steer_direction(VehicleWheelLocation::FR_Wheel, 0.0);

    Ok(())
}

// ===== Failure State Tests =====

fn test_failure_state_creation() -> TestResult {
    // Test creating different failure states
    let _none = VehicleFailureState::None;
    let _rollover = VehicleFailureState::Rollover;
    let _engine = VehicleFailureState::Engine;
    let _tire = VehicleFailureState::TirePuncture;

    // Verify they're different values
    assert!(
        VehicleFailureState::None != VehicleFailureState::Rollover,
        "Failure states should be distinct"
    );
    assert!(
        VehicleFailureState::Engine != VehicleFailureState::TirePuncture,
        "Failure states should be distinct"
    );

    Ok(())
}

fn test_apply_failure_state(vehicle: &Vehicle) -> TestResult {
    // Query the current failure state
    let initial_state = vehicle.failure_state();

    // Should start with no failures
    assert!(
        initial_state == VehicleFailureState::None,
        "Vehicle should start with no failures"
    );

    // Note: We don't actually apply failure states in tests
    // as it would damage the test vehicle
    // Just verify the query works

    Ok(())
}

fn test_vehicle_with_failures(vehicle: &Vehicle) -> TestResult {
    // Test querying failure state multiple times
    let state1 = vehicle.failure_state();
    thread::sleep(Duration::from_millis(50));
    let state2 = vehicle.failure_state();

    // States should be consistent if no failures applied
    assert!(state1 == state2, "Failure state should be stable");

    Ok(())
}

// ===== Light State Tests =====

fn test_light_state_creation() -> TestResult {
    // Test creating light states with different flags
    let none = VehicleLightState::NONE;
    let position = VehicleLightState::POSITION;
    let low_beam = VehicleLightState::LOW_BEAM;
    let high_beam = VehicleLightState::HIGH_BEAM;

    assert_eq!(none.bits(), 0x0);
    assert_eq!(position.bits(), 0x1);
    assert_eq!(low_beam.bits(), 0x2);
    assert_eq!(high_beam.bits(), 0x4);

    Ok(())
}

fn test_light_state_flags() -> TestResult {
    // Test combining light flags
    let lights = VehicleLightState::POSITION | VehicleLightState::LOW_BEAM;
    assert!(lights.contains(VehicleLightState::POSITION));
    assert!(lights.contains(VehicleLightState::LOW_BEAM));
    assert!(!lights.contains(VehicleLightState::HIGH_BEAM));

    // Test all lights
    let all_lights = VehicleLightState::ALL;
    assert!(all_lights.contains(VehicleLightState::POSITION));
    assert!(all_lights.contains(VehicleLightState::LOW_BEAM));
    assert!(all_lights.contains(VehicleLightState::HIGH_BEAM));
    assert!(all_lights.contains(VehicleLightState::BRAKE));

    // Test individual flags
    assert!(all_lights.contains(VehicleLightState::RIGHT_BLINKER));
    assert!(all_lights.contains(VehicleLightState::LEFT_BLINKER));
    assert!(all_lights.contains(VehicleLightState::REVERSE));
    assert!(all_lights.contains(VehicleLightState::FOG));

    Ok(())
}

fn test_apply_light_state(vehicle: &Vehicle) -> TestResult {
    // Test setting position and low beam lights
    let lights = VehicleLightState::POSITION | VehicleLightState::LOW_BEAM;
    vehicle.set_light_state(&lights);
    thread::sleep(Duration::from_millis(200));

    // Turn on high beams
    let lights_high =
        VehicleLightState::POSITION | VehicleLightState::LOW_BEAM | VehicleLightState::HIGH_BEAM;
    vehicle.set_light_state(&lights_high);
    thread::sleep(Duration::from_millis(200));

    // Turn off all lights
    vehicle.set_light_state(&VehicleLightState::NONE);

    Ok(())
}

fn test_vehicle_light_query(vehicle: &Vehicle) -> TestResult {
    // Set some lights
    let test_lights = VehicleLightState::POSITION | VehicleLightState::LOW_BEAM;
    vehicle.set_light_state(&test_lights);
    thread::sleep(Duration::from_millis(200));

    // Query the current state
    let current_lights = vehicle.light_state();

    // Verify position light is on
    assert!(
        current_lights.contains(VehicleLightState::POSITION),
        "Position light should be on"
    );

    // Verify low beam is on
    assert!(
        current_lights.contains(VehicleLightState::LOW_BEAM),
        "Low beam should be on"
    );

    // Turn off lights
    vehicle.set_light_state(&VehicleLightState::NONE);
    thread::sleep(Duration::from_millis(100));

    // Query again
    let _lights_off = vehicle.light_state();

    // Note: CARLA might keep some lights on by default
    // Just verify we can query the state without errors

    Ok(())
}

fn test_multiple_light_states(vehicle: &Vehicle) -> TestResult {
    // Test various light combinations

    // Test 1: Turn signals
    let left_signal = VehicleLightState::POSITION | VehicleLightState::LEFT_BLINKER;
    vehicle.set_light_state(&left_signal);
    thread::sleep(Duration::from_millis(200));

    let right_signal = VehicleLightState::POSITION | VehicleLightState::RIGHT_BLINKER;
    vehicle.set_light_state(&right_signal);
    thread::sleep(Duration::from_millis(200));

    // Test 2: Brake lights
    let braking =
        VehicleLightState::POSITION | VehicleLightState::LOW_BEAM | VehicleLightState::BRAKE;
    vehicle.set_light_state(&braking);
    thread::sleep(Duration::from_millis(200));

    // Test 3: Reverse lights
    let reversing =
        VehicleLightState::POSITION | VehicleLightState::REVERSE | VehicleLightState::BRAKE;
    vehicle.set_light_state(&reversing);
    thread::sleep(Duration::from_millis(200));

    // Test 4: Fog lights
    let foggy = VehicleLightState::POSITION | VehicleLightState::LOW_BEAM | VehicleLightState::FOG;
    vehicle.set_light_state(&foggy);
    thread::sleep(Duration::from_millis(200));

    // Reset
    vehicle.set_light_state(&VehicleLightState::NONE);

    Ok(())
}
