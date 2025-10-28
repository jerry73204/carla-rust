//! Show Recorder Info Example
//!
//! This example demonstrates how to query metadata from CARLA recording files:
//! - File information (duration, frames, actors)
//! - Collision data
//! - Blocked actor data
//!
//! Prerequisites:
//! - CARLA simulator must be running
//! - A recording file (recording01.log) must exist from start_recording.rs
//!
//! Run with:
//! ```bash
//! cargo run --example show_recorder_info
//! ```

use carla::client::Client;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CARLA Show Recorder Info Example ===\n");

    // Connect to CARLA
    println!("Connecting to CARLA simulator...");
    let mut client = Client::connect("localhost", 2000, None);
    println!("✓ Connected!\n");

    let recording_filename = "recording01.log";

    // Query 1: File Information
    println!("=== Query 1: File Information ===");
    println!("Requesting file info (summary)...\n");

    let file_info = client.show_recorder_file_info(recording_filename, false);
    println!("{}", file_info);
    println!();

    // Query 2: Detailed File Information
    println!("=== Query 2: Detailed File Information ===");
    println!("Requesting file info (detailed)...\n");

    let detailed_info = client.show_recorder_file_info(recording_filename, true);
    println!("{}", detailed_info);
    println!();

    // Query 3: Collision Data
    println!("=== Query 3: Collision Data ===");
    println!("Querying collisions (vehicle vs any)...\n");

    let collision_info = client.show_recorder_collisions(
        recording_filename,
        'v', // vehicles
        'a', // any type
    );
    println!("{}", collision_info);
    println!();

    println!("Querying collisions (any vs any)...\n");
    let all_collisions = client.show_recorder_collisions(
        recording_filename,
        'a', // any
        'a', // any
    );
    println!("{}", all_collisions);
    println!();

    // Query 4: Blocked Actors
    println!("=== Query 4: Blocked Actors ===");
    println!("Querying actors blocked for >10s (within 10m)...\n");

    let blocked_info = client.show_recorder_actors_blocked(
        recording_filename,
        10.0, // minimum time blocked (seconds)
        10.0, // minimum distance (meters)
    );
    println!("{}", blocked_info);
    println!();

    println!("=== Recorder Queries Complete ===");
    println!("\nAvailable query types:");
    println!("  • show_recorder_file_info(file, show_all)");
    println!("    - Summary or detailed info about recording");
    println!("    - Shows duration, frames, actors, map");
    println!();
    println!("  • show_recorder_collisions(file, type1, type2)");
    println!("    - Query collision events");
    println!(
        "    - Types: 'h'=hero, 'v'=vehicle, 'w'=walker, 't'=trafficlight, 'o'=other, 'a'=any"
    );
    println!();
    println!("  • show_recorder_actors_blocked(file, min_time, min_distance)");
    println!("    - Find actors stuck in place");
    println!("    - Useful for detecting traffic jams or physics issues");

    Ok(())
}
