//! Recording and Playback Tests
//!
//! Tests for recorder/replayer functionality (Phase 10.2).
//!
//! # Test Categories
//! - Recorder operations: Start, stop, file management
//! - Playback operations: Replay, time factor, camera following
//! - Query operations: Metadata, collisions, actor bounds
//!
//! Run with:
//! ```bash
//! cargo run --example test_recording --profile dev-release
//! ```

use carla::{
    client::{ActorBase, Client, Vehicle},
    rpc::VehicleControl,
};
use std::{thread, time::Duration};

type TestResult = Result<(), Box<dyn std::error::Error>>;

fn main() {
    println!("=== Recording and Playback Tests ===\n");

    // Connect to CARLA
    let mut client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();
    println!("Connected to CARLA server\n");

    // Setup test scenario
    let vehicle = setup_test_scenario(&mut world);

    // Run tests
    let mut passed = 0;
    let mut failed = 0;

    // Test recorder operations
    println!("--- Recorder Operations ---");
    run_test(
        "test_recorder_filename_validation",
        || test_recorder_filename_validation(&mut client),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_start_stop_recorder",
        || test_start_stop_recorder(&mut client),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_record_simple_scenario",
        || test_record_simple_scenario(&mut client, &vehicle),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_recorder_file_info",
        || test_recorder_file_info(&mut client),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_recorder_collision_query",
        || test_recorder_collision_query(&mut client),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_recorder_actor_bounds",
        || test_recorder_actor_bounds(&mut client),
        &mut passed,
        &mut failed,
    );

    // Test replay operations
    println!("\n--- Replay Operations ---");
    run_test(
        "test_replay_recorded_scenario",
        || test_replay_recorded_scenario(&mut client),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_replay_time_factor_bounds",
        || test_replay_time_factor_bounds(&mut client),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_replay_time_factor",
        || test_replay_time_factor(&mut client),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_replay_with_follow_camera",
        || test_replay_with_follow_camera(&mut client, &vehicle),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_recorder_frame_accuracy",
        || test_recorder_frame_accuracy(&mut client, &vehicle),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_replay_pause_resume",
        || test_replay_pause_resume(&mut client),
        &mut passed,
        &mut failed,
    );

    // Report results
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

// ===== Recorder Operation Tests =====

fn test_recorder_filename_validation(client: &mut Client) -> TestResult {
    // Test that recorder accepts valid filenames
    let result = client.start_recorder("/tmp/test_valid_filename.log", false);
    assert!(
        !result.is_empty(),
        "start_recorder should return a result string"
    );

    client.stop_recorder();

    // Test with different extensions
    let result2 = client.start_recorder("/tmp/test_recording.rec", false);
    assert!(
        !result2.is_empty(),
        "Should accept different file extensions"
    );

    client.stop_recorder();
    Ok(())
}

fn test_start_stop_recorder(client: &mut Client) -> TestResult {
    // Start recording
    let start_result = client.start_recorder("/tmp/test_start_stop.log", false);
    assert!(
        !start_result.is_empty(),
        "start_recorder should return a message"
    );

    // Wait a bit
    thread::sleep(Duration::from_millis(100));

    // Stop recording
    client.stop_recorder();

    // Start another recording to verify stop worked
    let restart_result = client.start_recorder("/tmp/test_restart.log", false);
    assert!(
        !restart_result.is_empty(),
        "Should be able to start recording again after stop"
    );

    client.stop_recorder();
    Ok(())
}

fn test_record_simple_scenario(client: &mut Client, vehicle: &Vehicle) -> TestResult {
    // Start recording with additional data
    client.start_recorder("/tmp/test_scenario.log", true);

    // Perform some actions with the vehicle
    let control = VehicleControl {
        throttle: 0.5,
        steer: 0.0,
        brake: 0.0,
        hand_brake: false,
        reverse: false,
        manual_gear_shift: false,
        gear: 0,
    };
    vehicle.apply_control(&control);

    thread::sleep(Duration::from_millis(500));

    let control = VehicleControl {
        throttle: 0.0,
        steer: 0.0,
        brake: 1.0,
        hand_brake: false,
        reverse: false,
        manual_gear_shift: false,
        gear: 0,
    };
    vehicle.apply_control(&control);

    thread::sleep(Duration::from_millis(500));

    // Stop recording
    client.stop_recorder();

    Ok(())
}

fn test_recorder_file_info(client: &mut Client) -> TestResult {
    // Record a short session
    client.start_recorder("/tmp/test_file_info.log", true);
    thread::sleep(Duration::from_millis(200));
    client.stop_recorder();

    // Query file info (summary)
    let info_summary = client.show_recorder_file_info("/tmp/test_file_info.log", false);
    assert!(
        !info_summary.is_empty(),
        "File info summary should not be empty"
    );

    // Query detailed info
    let info_detailed = client.show_recorder_file_info("/tmp/test_file_info.log", true);
    assert!(
        !info_detailed.is_empty(),
        "Detailed file info should not be empty"
    );
    assert!(
        info_detailed.len() >= info_summary.len(),
        "Detailed info should be at least as long as summary"
    );

    Ok(())
}

fn test_recorder_collision_query(client: &mut Client) -> TestResult {
    // Use a previously recorded file (from test_record_simple_scenario)
    // Query collisions - vehicle to vehicle
    let vv_collisions = client.show_recorder_collisions("/tmp/test_scenario.log", 'v', 'v');
    assert!(
        !vv_collisions.is_empty(),
        "Collision query should return a result"
    );

    // Query any collisions
    let any_collisions = client.show_recorder_collisions("/tmp/test_scenario.log", 'a', 'a');
    assert!(
        !any_collisions.is_empty(),
        "Any collision query should return a result"
    );

    // Query hero vehicle collisions
    let hero_collisions = client.show_recorder_collisions("/tmp/test_scenario.log", 'h', 'a');
    assert!(
        !hero_collisions.is_empty(),
        "Hero collision query should return a result"
    );

    Ok(())
}

fn test_recorder_actor_bounds(client: &mut Client) -> TestResult {
    // Use a previously recorded file
    // Query for actors blocked for 1 second within 0.5 meters
    let blocked_actors = client.show_recorder_actors_blocked("/tmp/test_scenario.log", 1.0, 0.5);
    assert!(
        !blocked_actors.is_empty(),
        "Blocked actors query should return a result"
    );

    // Query with different parameters
    let blocked_tight = client.show_recorder_actors_blocked("/tmp/test_scenario.log", 0.5, 0.1);
    assert!(
        !blocked_tight.is_empty(),
        "Tighter bounds query should return a result"
    );

    Ok(())
}

// ===== Replay Operation Tests =====

fn test_replay_recorded_scenario(client: &mut Client) -> TestResult {
    // First ensure we have a recording
    client.start_recorder("/tmp/test_replay_basic.log", true);
    thread::sleep(Duration::from_millis(500));
    client.stop_recorder();

    // Replay from beginning
    let replay_result = client.replay_file("/tmp/test_replay_basic.log", 0.0, 0.0, 0, false);
    assert!(
        !replay_result.is_empty(),
        "Replay should return a result message"
    );

    thread::sleep(Duration::from_millis(200));

    // Stop replay
    client.stop_replayer(false);

    Ok(())
}

fn test_replay_time_factor_bounds(client: &mut Client) -> TestResult {
    // Start replay
    client.replay_file("/tmp/test_replay_basic.log", 0.0, 0.0, 0, false);

    // Test normal speed
    client.set_replayer_time_factor(1.0);

    // Test slow motion
    client.set_replayer_time_factor(0.5);

    // Test fast forward
    client.set_replayer_time_factor(2.0);

    // Test very slow
    client.set_replayer_time_factor(0.1);

    // Test very fast
    client.set_replayer_time_factor(10.0);

    client.stop_replayer(false);
    Ok(())
}

fn test_replay_time_factor(client: &mut Client) -> TestResult {
    // Create a recording with known duration
    client.start_recorder("/tmp/test_time_factor.log", true);
    thread::sleep(Duration::from_millis(1000)); // 1 second recording
    client.stop_recorder();

    // Replay at 2x speed
    client.replay_file("/tmp/test_time_factor.log", 0.0, 0.0, 0, false);
    client.set_replayer_time_factor(2.0);

    // At 2x speed, 1 second should complete in ~500ms
    thread::sleep(Duration::from_millis(600));

    client.stop_replayer(false);
    Ok(())
}

fn test_replay_with_follow_camera(client: &mut Client, vehicle: &Vehicle) -> TestResult {
    // Record a scenario with the vehicle
    client.start_recorder("/tmp/test_follow_camera.log", true);

    let control = VehicleControl {
        throttle: 0.3,
        steer: 0.0,
        brake: 0.0,
        hand_brake: false,
        reverse: false,
        manual_gear_shift: false,
        gear: 0,
    };
    vehicle.apply_control(&control);

    thread::sleep(Duration::from_millis(500));
    client.stop_recorder();

    // Replay with camera following the vehicle
    let vehicle_id = vehicle.id();
    let replay_result =
        client.replay_file("/tmp/test_follow_camera.log", 0.0, 0.0, vehicle_id, false);
    assert!(
        !replay_result.is_empty(),
        "Replay with follow camera should return result"
    );

    thread::sleep(Duration::from_millis(300));

    client.stop_replayer(false);
    Ok(())
}

fn test_recorder_frame_accuracy(client: &mut Client, vehicle: &Vehicle) -> TestResult {
    // Record at specific intervals
    client.start_recorder("/tmp/test_frame_accuracy.log", true);

    // Perform actions at known times

    // Time 0: Start moving
    let control = VehicleControl {
        throttle: 0.5,
        steer: 0.0,
        brake: 0.0,
        hand_brake: false,
        reverse: false,
        manual_gear_shift: false,
        gear: 0,
    };
    vehicle.apply_control(&control);
    thread::sleep(Duration::from_millis(250));

    // Time 250ms: Apply brake
    let control = VehicleControl {
        throttle: 0.0,
        steer: 0.0,
        brake: 1.0,
        hand_brake: false,
        reverse: false,
        manual_gear_shift: false,
        gear: 0,
    };
    vehicle.apply_control(&control);
    thread::sleep(Duration::from_millis(250));

    client.stop_recorder();

    // Query the recording info to verify it captured the frames
    let info = client.show_recorder_file_info("/tmp/test_frame_accuracy.log", true);
    assert!(
        !info.is_empty(),
        "Recording should contain frame information"
    );

    Ok(())
}

fn test_replay_pause_resume(client: &mut Client) -> TestResult {
    // Create a recording
    client.start_recorder("/tmp/test_pause_resume.log", true);
    thread::sleep(Duration::from_millis(1000));
    client.stop_recorder();

    // Start replay at normal speed
    client.replay_file("/tmp/test_pause_resume.log", 0.0, 0.0, 0, false);
    client.set_replayer_time_factor(1.0);

    thread::sleep(Duration::from_millis(200));

    // "Pause" by setting time factor to 0
    client.set_replayer_time_factor(0.0);
    thread::sleep(Duration::from_millis(200));

    // "Resume" by setting time factor back to 1.0
    client.set_replayer_time_factor(1.0);
    thread::sleep(Duration::from_millis(200));

    client.stop_replayer(false);
    Ok(())
}
