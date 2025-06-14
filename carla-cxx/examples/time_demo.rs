//! Example demonstrating time and timestamp functionality with carla-cxx

use anyhow::Result;
use carla_cxx::{ClientWrapper, SimulationClock, TimeDuration};
use std::time::Duration;

fn main() -> Result<()> {
    println!("CARLA Time and Timestamp Example");
    println!("================================\n");

    // Connect to CARLA server
    println!("Connecting to CARLA server...");
    let mut client = ClientWrapper::new("localhost", 2000)?;
    println!("Connected to CARLA server");

    // Set client timeout
    client.set_timeout(Duration::from_secs(10));
    println!("Client timeout set to: {:?}", client.get_timeout());

    // Get the world
    let world = client.get_world();
    println!("Got world with ID: {}", world.get_id());

    // Create a simulation clock to track time
    let mut sim_clock = SimulationClock::new();
    println!("\nSimulation clock created");

    // Run simulation for a while
    println!("\nStarting simulation loop...");
    let start_time = std::time::Instant::now();
    let mut last_report_time = start_time;

    for i in 0..300 {
        // Tick the world
        let frame_id = world.tick(Duration::from_secs(1));

        // Get world snapshot with timestamp
        let timestamp = world.get_snapshot();

        // Update simulation clock
        sim_clock.update(timestamp);

        // Print timestamp info every 30 frames (roughly 1 second at 30 FPS)
        if i % 30 == 0 {
            println!("\n--- Frame {} ---", frame_id);
            println!("Timestamp: {}", timestamp);
            println!("  Frame: {}", timestamp.frame());
            println!("  Elapsed: {:.3}s", timestamp.elapsed_seconds());
            println!("  Delta: {:.3}s", timestamp.delta_seconds());
            println!("  Platform: {:.3}s", timestamp.platform_timestamp());

            // Show simulation clock stats
            println!("\nSimulation Clock:");
            println!("  Total elapsed: {:.3}s", sim_clock.total_elapsed());
            println!("  Total frames: {}", sim_clock.total_frames());
            println!("  Average FPS: {:.2}", sim_clock.average_fps());
            println!("  Current FPS: {:.2}", sim_clock.current_fps());

            // Show real-time performance
            let real_elapsed = start_time.elapsed();
            let report_delta = last_report_time.elapsed();
            println!("\nReal-time Performance:");
            println!("  Real elapsed: {:.3}s", real_elapsed.as_secs_f64());
            println!("  Report interval: {:.3}s", report_delta.as_secs_f64());
            println!(
                "  Simulation speed: {:.2}x",
                sim_clock.total_elapsed() / real_elapsed.as_secs_f64()
            );

            last_report_time = std::time::Instant::now();
        }

        // Demonstrate frame timing utilities
        if i == 150 {
            println!("\n=== Frame Rate Control Demo ===");

            // Show how to limit frame rate
            let target_fps = 30.0;
            println!("Target FPS: {}", target_fps);

            let frame_start = std::time::Instant::now();
            world.tick(Duration::from_secs(1));

            if let Some(sleep_duration) =
                carla_cxx::time::utils::sleep_for_target_fps(frame_start, target_fps)
            {
                println!(
                    "Slept for {:?} to maintain {} FPS",
                    sleep_duration, target_fps
                );
            } else {
                println!("Frame took too long, couldn't maintain {} FPS", target_fps);
            }
        }

        // Demonstrate time scale (slow motion)
        if i == 200 {
            println!("\n=== Time Scale Demo ===");
            sim_clock.set_time_scale(0.5);
            println!("Set time scale to 0.5x (slow motion)");
        }

        // Small delay to prevent overwhelming the server
        std::thread::sleep(Duration::from_millis(10));
    }

    // Final statistics
    println!("\n=== Final Statistics ===");
    let final_timestamp = world.get_snapshot();
    sim_clock.update(final_timestamp);

    println!("Total simulation time: {:.3}s", sim_clock.total_elapsed());
    println!("Total frames: {}", sim_clock.total_frames());
    println!("Average FPS: {:.2}", sim_clock.average_fps());

    let total_real_time = start_time.elapsed();
    println!("\nTotal real time: {:.3}s", total_real_time.as_secs_f64());
    println!(
        "Simulation/Real time ratio: {:.2}x",
        sim_clock.total_elapsed() / total_real_time.as_secs_f64()
    );

    // Demonstrate duration formatting
    println!("\n=== Duration Formatting ===");
    use carla_cxx::time::utils::format_duration;
    println!("500ms: {}", format_duration(0.5));
    println!("1.5s: {}", format_duration(1.5));
    println!("65s: {}", format_duration(65.0));
    println!("3665s: {}", format_duration(3665.0));

    // Demonstrate TimeDuration
    println!("\n=== TimeDuration Demo ===");
    let duration1 = TimeDuration::from_seconds(2.5);
    println!(
        "Duration 1: {}ms = {}s",
        duration1.as_milliseconds(),
        duration1.as_seconds()
    );

    let std_duration = Duration::from_millis(1500);
    let duration2 = TimeDuration::from(std_duration);
    println!("Duration 2 (from std): {}ms", duration2.as_milliseconds());

    println!("\nDemo complete!");
    Ok(())
}
