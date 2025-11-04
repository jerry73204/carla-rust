//! Sensor Synchronization Example
//!
//! This example demonstrates spawning and synchronizing multiple sensors
//! of different types (RGB, Depth, Semantic Segmentation, Lidar).
//! Shows proper sensor setup, data collection patterns, and synchronization.
//!
//! Prerequisites:
//! - CARLA simulator must be running
//!
//! Run with:
//! ```bash
//! cargo run --example sensor_synchronization
//! ```

use carla::{
    client::{ActorBase, Client, Sensor, Vehicle},
    geom::{Location, Rotation},
    rpc::AttachmentType,
    sensor::data::{Image, LidarMeasurement},
};
use std::{
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

#[derive(Debug, Default)]
struct SensorStats {
    rgb_frames: usize,
    depth_frames: usize,
    semantic_frames: usize,
    lidar_frames: usize,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== CARLA Sensor Synchronization Example ===\n");

    // Connect to CARLA
    println!("Connecting to CARLA simulator...");
    let client = Client::connect("localhost", 2000, None);
    let mut world = client.world();
    println!("✓ Connected! Current map: {}\n", world.map().name());

    // Spawn a vehicle
    println!("Spawning vehicle...");
    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or("Tesla Model 3 blueprint not found")?;

    let spawn_points = world.map().recommended_spawn_points();
    let spawn_point = spawn_points.get(0).ok_or("No spawn points available")?;

    let actor = world.spawn_actor(&vehicle_bp, spawn_point)?;
    let vehicle = Vehicle::try_from(actor).map_err(|_| "Failed to convert to vehicle")?;
    vehicle.set_autopilot(true);
    println!("✓ Vehicle spawned (ID: {})\n", vehicle.id());

    // Define sensor transforms
    let camera_transform = carla::geom::Transform {
        location: Location::new(2.0, 0.0, 1.5), // Front hood position
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };

    let lidar_transform = carla::geom::Transform {
        location: Location::new(0.0, 0.0, 2.5), // Roof position
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };

    println!("Spawning sensors...");

    // 1. RGB Camera
    let mut camera_rgb_bp = blueprint_library
        .find("sensor.camera.rgb")
        .ok_or("RGB camera not found")?;
    if !camera_rgb_bp.set_attribute("image_size_x", "800") {
        return Err("Failed to set RGB camera image_size_x".into());
    }
    if !camera_rgb_bp.set_attribute("image_size_y", "600") {
        return Err("Failed to set RGB camera image_size_y".into());
    }
    if !camera_rgb_bp.set_attribute("fov", "90.0") {
        return Err("Failed to set RGB camera fov".into());
    }

    let camera_rgb_actor = world.spawn_actor_opt(
        &camera_rgb_bp,
        &camera_transform,
        Some(&vehicle),
        AttachmentType::Rigid,
    )?;
    let camera_rgb =
        Sensor::try_from(camera_rgb_actor).map_err(|_| "Failed to convert RGB camera")?;
    println!("  ✓ RGB Camera (ID: {})", camera_rgb.id());

    // 2. Depth Camera
    let mut camera_depth_bp = blueprint_library
        .find("sensor.camera.depth")
        .ok_or("Depth camera not found")?;
    if !camera_depth_bp.set_attribute("image_size_x", "800") {
        return Err("Failed to set depth camera image_size_x".into());
    }
    if !camera_depth_bp.set_attribute("image_size_y", "600") {
        return Err("Failed to set depth camera image_size_y".into());
    }
    if !camera_depth_bp.set_attribute("fov", "90.0") {
        return Err("Failed to set depth camera fov".into());
    }

    let camera_depth_actor = world.spawn_actor_opt(
        &camera_depth_bp,
        &camera_transform,
        Some(&vehicle),
        AttachmentType::Rigid,
    )?;
    let camera_depth =
        Sensor::try_from(camera_depth_actor).map_err(|_| "Failed to convert depth camera")?;
    println!("  ✓ Depth Camera (ID: {})", camera_depth.id());

    // 3. Semantic Segmentation Camera
    let mut camera_seg_bp = blueprint_library
        .find("sensor.camera.semantic_segmentation")
        .ok_or("Semantic camera not found")?;
    if !camera_seg_bp.set_attribute("image_size_x", "800") {
        return Err("Failed to set semantic camera image_size_x".into());
    }
    if !camera_seg_bp.set_attribute("image_size_y", "600") {
        return Err("Failed to set semantic camera image_size_y".into());
    }
    if !camera_seg_bp.set_attribute("fov", "90.0") {
        return Err("Failed to set semantic camera fov".into());
    }

    let camera_seg_actor = world.spawn_actor_opt(
        &camera_seg_bp,
        &camera_transform,
        Some(&vehicle),
        AttachmentType::Rigid,
    )?;
    let camera_seg =
        Sensor::try_from(camera_seg_actor).map_err(|_| "Failed to convert semantic camera")?;
    println!("  ✓ Semantic Camera (ID: {})", camera_seg.id());

    // 4. Lidar
    let mut lidar_bp = blueprint_library
        .find("sensor.lidar.ray_cast")
        .ok_or("Lidar not found")?;
    if !lidar_bp.set_attribute("range", "50.0") {
        return Err("Failed to set lidar range".into());
    }
    if !lidar_bp.set_attribute("rotation_frequency", "20.0") {
        return Err("Failed to set lidar rotation_frequency".into());
    }
    if !lidar_bp.set_attribute("channels", "32") {
        return Err("Failed to set lidar channels".into());
    }
    if !lidar_bp.set_attribute("points_per_second", "100000") {
        return Err("Failed to set lidar points_per_second".into());
    }

    let lidar_actor = world.spawn_actor_opt(
        &lidar_bp,
        &lidar_transform,
        Some(&vehicle),
        AttachmentType::Rigid,
    )?;
    let lidar = Sensor::try_from(lidar_actor).map_err(|_| "Failed to convert lidar")?;
    println!("  ✓ Lidar (ID: {})\n", lidar.id());

    // Setup synchronized data collection
    let stats = Arc::new(Mutex::new(SensorStats::default()));

    // RGB listener
    let stats_rgb = Arc::clone(&stats);
    camera_rgb.listen(move |sensor_data| {
        if let Ok(image) = Image::try_from(sensor_data) {
            let mut s = stats_rgb.lock().unwrap();
            s.rgb_frames += 1;

            // Log every 10th frame
            if s.rgb_frames % 10 == 0 {
                println!(
                    "  RGB: Frame {} ({}x{} pixels)",
                    s.rgb_frames,
                    image.width(),
                    image.height()
                );
            }
        }
    });

    // Depth listener
    let stats_depth = Arc::clone(&stats);
    camera_depth.listen(move |sensor_data| {
        if let Ok(_image) = Image::try_from(sensor_data) {
            let mut s = stats_depth.lock().unwrap();
            s.depth_frames += 1;
        }
    });

    // Semantic listener
    let stats_seg = Arc::clone(&stats);
    camera_seg.listen(move |sensor_data| {
        if let Ok(_image) = Image::try_from(sensor_data) {
            let mut s = stats_seg.lock().unwrap();
            s.semantic_frames += 1;
        }
    });

    // Lidar listener
    let stats_lidar = Arc::clone(&stats);
    lidar.listen(move |sensor_data| {
        if let Ok(measurement) = LidarMeasurement::try_from(sensor_data) {
            let mut s = stats_lidar.lock().unwrap();
            s.lidar_frames += 1;

            // Log every 5th scan
            if s.lidar_frames % 5 == 0 {
                println!(
                    "  Lidar: Scan {} ({} points)",
                    s.lidar_frames,
                    measurement.len()
                );
            }
        }
    });

    println!("✓ All sensors listening\n");

    // Monitor sensor synchronization for 30 seconds
    println!("Monitoring sensor data collection (30 seconds)...\n");
    println!("┌──────────┬─────────────────────────────────────────────────────┐");
    println!("│   Time   │            Sensor Frame Counts                      │");
    println!("├──────────┼─────────────────────────────────────────────────────┤");

    let start_time = Instant::now();
    for i in 0..30 {
        thread::sleep(Duration::from_secs(1));

        if i % 5 == 0 {
            let s = stats.lock().unwrap();
            println!(
                "│ {:>6}s  │ RGB:{:>4}  Depth:{:>4}  Semantic:{:>4}  Lidar:{:>4}     │",
                i + 1,
                s.rgb_frames,
                s.depth_frames,
                s.semantic_frames,
                s.lidar_frames
            );
        }
    }

    println!("└──────────┴─────────────────────────────────────────────────────┘");

    let elapsed = start_time.elapsed();

    // Final statistics
    let final_stats = stats.lock().unwrap();

    println!("\n=== Sensor Synchronization Statistics ===");
    println!("Total time: {:.2} seconds", elapsed.as_secs_f64());
    println!("\nFrame counts:");
    println!(
        "  RGB Camera:        {} frames ({:.1} FPS)",
        final_stats.rgb_frames,
        final_stats.rgb_frames as f64 / elapsed.as_secs_f64()
    );
    println!(
        "  Depth Camera:      {} frames ({:.1} FPS)",
        final_stats.depth_frames,
        final_stats.depth_frames as f64 / elapsed.as_secs_f64()
    );
    println!(
        "  Semantic Camera:   {} frames ({:.1} FPS)",
        final_stats.semantic_frames,
        final_stats.semantic_frames as f64 / elapsed.as_secs_f64()
    );
    println!(
        "  Lidar:             {} scans ({:.1} Hz)",
        final_stats.lidar_frames,
        final_stats.lidar_frames as f64 / elapsed.as_secs_f64()
    );

    // Check synchronization
    let camera_frames_match = final_stats.rgb_frames == final_stats.depth_frames
        && final_stats.depth_frames == final_stats.semantic_frames;

    println!("\nSynchronization status:");
    println!(
        "  Camera sync: {}",
        if camera_frames_match {
            "✓ Perfect"
        } else {
            "⚠ Drift detected"
        }
    );

    let avg_camera_frames =
        (final_stats.rgb_frames + final_stats.depth_frames + final_stats.semantic_frames) as f64
            / 3.0;
    let sync_percentage = (final_stats
        .rgb_frames
        .min(final_stats.depth_frames)
        .min(final_stats.semantic_frames) as f64
        / avg_camera_frames)
        * 100.0;
    println!("  Sync quality: {:.1}%", sync_percentage);

    // Cleanup
    println!("\nCleaning up...");
    camera_rgb.stop();
    camera_depth.stop();
    camera_seg.stop();
    lidar.stop();

    camera_rgb.destroy();
    camera_depth.destroy();
    camera_seg.destroy();
    lidar.destroy();
    vehicle.destroy();
    println!("✓ All actors destroyed");

    println!("\n=== Sensor Synchronization Demo Complete ===");
    println!("\nKey Features Demonstrated:");
    println!("  • Multiple sensor types (RGB, Depth, Semantic, Lidar)");
    println!("  • Sensor attachment to moving vehicle");
    println!("  • Synchronized data collection");
    println!("  • Frame rate monitoring");
    println!("  • Synchronization quality metrics");
    println!("  • Proper sensor cleanup");

    Ok(())
}
