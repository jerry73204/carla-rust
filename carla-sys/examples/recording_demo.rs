//! Recording and playback demo for carla-sys
//!
//! This example demonstrates recording simulation data, analyzing recordings,
//! and playing back recorded sessions in CARLA simulator.

use carla_sys::{
    recording::{actor_types, defaults},
    ClientWrapper,
};
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    println!("🎬 Recording and Playback Demo for carla-sys");
    println!("=============================================");

    // Try to connect to CARLA server
    match ClientWrapper::new("localhost", 2000) {
        Ok(client) => {
            println!("✅ Connected to CARLA server");

            let world = client.get_world();

            // Demo recording filename
            let recording_filename = "demo_recording.log";

            // 1. Start Recording
            println!("\n🔴 Starting recording...");
            let start_result = client.start_recorder(recording_filename, true);
            println!("Start recording result: {}", start_result);

            // Let the simulation run for a few ticks to record some data
            println!("📊 Recording simulation data for 5 seconds...");
            for i in 1..=5 {
                world.tick(Duration::from_secs(2));
                println!("   Tick {}/5 completed", i);
            }

            // 2. Stop Recording
            println!("\n⏹️  Stopping recording...");
            client.stop_recorder();
            println!("✅ Recording stopped");

            // 3. Analyze Recording
            println!("\n📋 Analyzing recording file...");

            // Show file information
            println!("\n📄 File Information:");
            let file_info = client.show_recorder_file_info(recording_filename, true);
            println!("{}", file_info);

            // Show collision analysis (all actor types)
            println!("\n💥 Collision Analysis:");
            let collision_analysis = client.show_recorder_collisions(
                recording_filename,
                actor_types::ALL,
                actor_types::ALL,
            );
            println!("{}", collision_analysis);

            // Show blocked actors analysis
            println!("\n🚫 Blocked Actors Analysis:");
            let blocked_analysis = client.show_recorder_actors_blocked(
                recording_filename,
                defaults::MIN_TIME_BLOCKED,
                defaults::MIN_DISTANCE_BLOCKED,
            );
            println!("{}", blocked_analysis);

            // 4. Playback Demo
            println!("\n▶️  Starting playback demo...");

            // Set replay parameters
            client.set_replayer_time_factor(2.0); // 2x speed
            client.set_replayer_ignore_hero(false);
            client.set_replayer_ignore_spectator(false);

            // Start replay from beginning for 10 seconds, no specific actor to follow
            let replay_result = client.replay_file(
                recording_filename,
                0.0,  // start time
                10.0, // duration (0.0 for entire file)
                0,    // follow_id (0 for no follow)
                true, // replay_sensors
            );
            println!("Replay result: {}", replay_result);

            // Let it play for a moment
            std::thread::sleep(Duration::from_secs(3));

            // Change playback speed during replay
            println!("🚀 Increasing playback speed to 3x...");
            client.set_replayer_time_factor(3.0);

            std::thread::sleep(Duration::from_secs(2));

            // Stop replay
            println!("\n⏹️  Stopping replay...");
            client.stop_replayer(false); // don't keep actors

            // 5. Advanced Analysis Examples
            println!("\n🔍 Advanced Analysis Examples:");

            // Analyze vehicle-only collisions
            println!("\n🚗 Vehicle Collision Analysis:");
            let vehicle_collisions = client.show_recorder_collisions(
                recording_filename,
                actor_types::VEHICLE,
                actor_types::VEHICLE,
            );
            println!("{}", vehicle_collisions);

            // Custom blocked actor analysis with different thresholds
            println!("\n⏱️  Custom Blocked Actor Analysis (stricter thresholds):");
            let strict_blocked = client.show_recorder_actors_blocked(
                recording_filename,
                10.0, // 10 second threshold
                5.0,  // 5 meter threshold
            );
            println!("{}", strict_blocked);

            println!("\n✨ Recording and playback demo completed successfully!");
            println!("💡 Recording saved as: {}", recording_filename);
            println!("💡 You can replay this file later or analyze it further");
        }
        Err(e) => {
            println!("❌ Failed to connect to CARLA server: {}", e);
            println!("💡 Make sure CARLA simulator is running on localhost:2000");
            println!("   Example: ./CarlaUE4.sh -windowed -ResX=800 -ResY=600");
            return Err(e);
        }
    }

    Ok(())
}
