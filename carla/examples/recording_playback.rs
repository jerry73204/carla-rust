//! Recording and playback example
//!
//! This example demonstrates the CARLA recording and replay API for capturing
//! and replaying simulation sessions.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example recording_playback
//! ```

use carla::{
    client::{ActorBase, Client, Vehicle},
    rpc::VehicleControl,
};
use std::{thread, time::Duration};

fn main() {
    println!("=== CARLA Recording and Playback Example ===\n");

    // Connect to CARLA
    println!("Connecting to CARLA simulator...");
    let mut client = Client::connect("localhost", 2000, None);
    println!("Connected!\n");

    // Get world and spawn a vehicle
    println!("Loading map: Town10HD_Opt...");
    let mut world = client.load_world("Town10HD_Opt");
    println!("Map loaded!\n");

    // Spawn a vehicle
    println!("Spawning vehicle...");
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .expect("Tesla Model 3 not found");

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).expect("No spawn points available");

    let vehicle = world
        .spawn_actor(&vehicle_bp, spawn_point)
        .expect("Failed to spawn vehicle");
    let vehicle = Vehicle::try_from(vehicle).expect("Actor is not a vehicle");

    println!("Spawned vehicle: {}", vehicle.type_id());
    println!("Vehicle ID: {}\n", vehicle.id());

    // Start recording
    let recording_filename = "test_recording.log";
    println!("Starting recording to '{}'...", recording_filename);
    let result = client.start_recorder(recording_filename, false);
    println!("Recording started: {}\n", result);

    // Drive the vehicle forward for a few seconds
    println!("Driving vehicle forward for 5 seconds...");
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

    for i in 1..=5 {
        thread::sleep(Duration::from_secs(1));
        let location = vehicle.location();
        println!(
            "  [{}s] Vehicle position: ({:.2}, {:.2}, {:.2})",
            i, location.x, location.y, location.z
        );
    }

    // Stop the vehicle
    println!("\nStopping vehicle...");
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
    println!("Stopping recording...");
    client.stop_recorder();
    println!("Recording stopped!\n");

    // Show recording file info
    println!("=== Recording File Info ===");
    let file_info = client.show_recorder_file_info(recording_filename, false);
    println!("{}\n", file_info);

    // Query collisions (there shouldn't be any in this simple example)
    println!("=== Collisions (vehicle-to-vehicle) ===");
    let collisions = client.show_recorder_collisions(recording_filename, 'v', 'v');
    if collisions.trim().is_empty() {
        println!("No collisions detected\n");
    } else {
        println!("{}\n", collisions);
    }

    // Query blocked actors
    println!("=== Blocked Actors ===");
    let blocked = client.show_recorder_actors_blocked(recording_filename, 30.0, 10.0);
    if blocked.trim().is_empty() {
        println!("No blocked actors\n");
    } else {
        println!("{}\n", blocked);
    }

    // Replay the recording
    println!("=== Starting Replay ===");
    println!("Replaying recording at 2x speed...");
    let replay_result = client.replay_file(recording_filename, 0.0, 0.0, 0, false);
    println!("Replay started: {}\n", replay_result);

    // Set replay speed to 2x
    client.set_replayer_time_factor(2.0);
    println!("Replay speed set to 2x");

    // Let the replay run for a bit
    println!("Watching replay for 3 seconds...");
    thread::sleep(Duration::from_secs(3));

    // Stop replay
    println!("\nStopping replay...");
    client.stop_replayer(false);
    println!("Replay stopped!");

    println!("\n=== Example Complete ===");
    println!("Recording saved to: {}", recording_filename);
    println!("Note: The recording file is saved on the CARLA server, not the client.");
}
