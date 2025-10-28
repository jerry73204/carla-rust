//! Start Replaying Example
//!
//! This example demonstrates how to replay a previously recorded CARLA session.
//! Make sure to run start_recording.rs first to create a recording file.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//! - A recording file (recording01.log) must exist from start_recording.rs
//!
//! Run with:
//! ```bash
//! cargo run --example start_replaying
//! ```

use carla::client::Client;
use std::{thread, time::Duration};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CARLA Start Replaying Example ===\n");

    // Connect to CARLA
    println!("Connecting to CARLA simulator...");
    let mut client = Client::connect("localhost", 2000, None);
    println!("✓ Connected!\n");

    // Replay configuration
    let recording_filename = "recording01.log";
    let start_time = 0.0; // Start from beginning
    let duration = 0.0; // 0.0 means replay entire recording
    let follow_id = 0; // 0 means don't follow specific actor
    let replay_sensors = true; // Replay sensor data

    println!("Starting replay...");
    println!("  Filename: {}", recording_filename);
    println!("  Start time: {:.1}s", start_time);
    let duration_str = if duration == 0.0 {
        "∞ (entire recording)".to_string()
    } else {
        format!("{:.1}s", duration)
    };
    println!("  Duration: {}", duration_str);
    let follow_str = if follow_id == 0 {
        "none".to_string()
    } else {
        follow_id.to_string()
    };
    println!("  Follow actor: {}", follow_str);
    println!("  Replay sensors: {}", replay_sensors);

    let result = client.replay_file(
        recording_filename,
        start_time,
        duration,
        follow_id,
        replay_sensors,
    );

    println!("✓ Replay started: {}\n", result);

    // Let replay run for a while
    println!("Replaying for 30 seconds...");
    for i in 1..=30 {
        thread::sleep(Duration::from_secs(1));
        println!("  Replaying... {} seconds", i);
    }

    // Stop replay
    println!("\nStopping replayer...");
    client.stop_replayer(false); // false = destroy actors after replay stops
    println!("✓ Replay stopped");

    println!("\n=== Replay Complete ===");
    println!("\nReplay controls:");
    println!("  • client.replay_file(...) - Start replay");
    println!("  • client.stop_replayer(keep_actors) - Stop replay");
    println!("  • client.set_replayer_time_factor(speed) - Adjust playback speed");
    println!("  • client.set_replayer_ignore_hero(true) - Ignore hero vehicle");

    Ok(())
}
