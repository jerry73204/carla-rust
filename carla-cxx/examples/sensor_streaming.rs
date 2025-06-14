//! Example demonstrating sensor data streaming with carla-cxx

use anyhow::Result;
use carla_cxx::{
    ClientWrapper, SensorWrapper, StreamConfig, StreamPriority, StreamingManager, SyncGroup,
};
use std::{sync::Arc, thread, time::Duration};

fn main() -> Result<()> {
    println!("CARLA Sensor Streaming Example");
    println!("==============================\n");

    // Connect to CARLA server
    println!("Connecting to CARLA server...");
    let mut client = ClientWrapper::new("localhost", 2000)?;
    let world = client.get_world();
    println!("Connected to CARLA server");

    // Get blueprint library
    let blueprint_library = world.get_blueprint_library();

    // Spawn a vehicle
    println!("\nSpawning vehicle...");
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .ok_or_else(|| anyhow::anyhow!("Vehicle blueprint not found"))?;

    let spawn_points = world.get_spawn_points();
    if spawn_points.is_empty() {
        return Err(anyhow::anyhow!("No spawn points available"));
    }

    let vehicle = world.spawn_actor(&vehicle_bp, &spawn_points[0], None)?;
    println!("Vehicle spawned with ID: {}", vehicle.get_id());

    // Create streaming manager
    let streaming_manager = Arc::new(StreamingManager::new());
    let mut streaming_manager_mut = StreamingManager::new();
    streaming_manager_mut.start()?;

    // Spawn RGB camera sensor
    println!("\nSpawning RGB camera sensor...");
    let camera_bp = blueprint_library
        .find("sensor.camera.rgb")
        .ok_or_else(|| anyhow::anyhow!("Camera blueprint not found"))?;

    // Set camera attributes
    let mut camera_bp_mut = camera_bp.clone();
    camera_bp_mut.set_attribute("image_size_x", "1920");
    camera_bp_mut.set_attribute("image_size_y", "1080");
    camera_bp_mut.set_attribute("fov", "90");

    // Camera transform (relative to vehicle)
    let camera_transform = carla_cxx::SimpleTransform::new(
        carla_cxx::SimpleLocation::new(2.0, 0.0, 1.5),
        carla_cxx::SimpleRotation::ZERO,
    );

    let camera_actor = world.spawn_actor(&camera_bp_mut, &camera_transform, Some(&vehicle))?;
    let mut camera_sensor = SensorWrapper::from_actor(&camera_actor)
        .ok_or_else(|| anyhow::anyhow!("Failed to cast to sensor"))?;

    // Configure streaming for camera
    let camera_config = StreamConfig {
        buffer_size: 30,
        drop_on_overflow: true,
        priority: StreamPriority::High,
        max_fps: Some(30.0),
        enable_sync: true,
    };

    let camera_stream_id = {
        let manager_clone = Arc::clone(&streaming_manager);
        camera_sensor.listen_streaming(manager_clone, camera_config, |data| match &data {
            carla_cxx::SensorData::Image(img) => {
                println!(
                    "Camera: Received image {}x{} ({} bytes)",
                    img.width,
                    img.height,
                    img.data.len()
                );
            }
            _ => println!("Camera: Unexpected data type"),
        })?;
        1 // Placeholder stream ID
    };

    // Spawn LiDAR sensor
    println!("\nSpawning LiDAR sensor...");
    let lidar_bp = blueprint_library
        .find("sensor.lidar.ray_cast")
        .ok_or_else(|| anyhow::anyhow!("LiDAR blueprint not found"))?;

    // Set LiDAR attributes
    let mut lidar_bp_mut = lidar_bp.clone();
    lidar_bp_mut.set_attribute("channels", "32");
    lidar_bp_mut.set_attribute("range", "100");
    lidar_bp_mut.set_attribute("rotation_frequency", "10");
    lidar_bp_mut.set_attribute("points_per_second", "100000");

    // LiDAR transform (on vehicle roof)
    let lidar_transform = carla_cxx::SimpleTransform::new(
        carla_cxx::SimpleLocation::new(0.0, 0.0, 2.5),
        carla_cxx::SimpleRotation::ZERO,
    );

    let lidar_actor = world.spawn_actor(&lidar_bp_mut, &lidar_transform, Some(&vehicle))?;
    let mut lidar_sensor = SensorWrapper::from_actor(&lidar_actor)
        .ok_or_else(|| anyhow::anyhow!("Failed to cast to sensor"))?;

    // Configure streaming for LiDAR
    let lidar_config = StreamConfig {
        buffer_size: 20,
        drop_on_overflow: true,
        priority: StreamPriority::Normal,
        max_fps: Some(10.0),
        enable_sync: true,
    };

    let lidar_stream_id = {
        let manager_clone = Arc::clone(&streaming_manager);
        lidar_sensor.listen_streaming(manager_clone, lidar_config, |data| match &data {
            carla_cxx::SensorData::LiDAR(lidar) => {
                println!("LiDAR: Received {} points", lidar.point_count);
            }
            _ => println!("LiDAR: Unexpected data type"),
        })?;
        2 // Placeholder stream ID
    };

    // Create synchronized sensor group
    println!("\nCreating synchronized sensor group...");
    let sync_group =
        streaming_manager.create_sync_group(vec![camera_stream_id, lidar_stream_id])?;

    // Spawn collision detector
    println!("\nSpawning collision detector...");
    let collision_bp = blueprint_library
        .find("sensor.other.collision")
        .ok_or_else(|| anyhow::anyhow!("Collision blueprint not found"))?;

    let collision_actor = world.spawn_actor(&collision_bp, &spawn_points[0], Some(&vehicle))?;
    let mut collision_sensor = SensorWrapper::from_actor(&collision_actor)
        .ok_or_else(|| anyhow::anyhow!("Failed to cast to sensor"))?;

    // Configure streaming for collision detector
    let collision_config = StreamConfig {
        buffer_size: 100,
        drop_on_overflow: false, // Don't drop collision events
        priority: StreamPriority::High,
        max_fps: None, // No rate limiting for collision events
        enable_sync: false,
    };

    collision_sensor.listen_streaming(
        Arc::clone(&streaming_manager),
        collision_config,
        |data| match &data {
            carla_cxx::SensorData::Collision(collision) => {
                println!(
                    "COLLISION! With actor {} (impulse: {:.2} N)",
                    collision.other_actor_id,
                    collision.impulse_magnitude()
                );
            }
            _ => println!("Collision: Unexpected data type"),
        },
    )?;

    // Main simulation loop
    println!("\nStarting simulation loop...");
    println!("Press Ctrl+C to stop\n");

    let start_time = std::time::Instant::now();
    let mut frame_count = 0;

    loop {
        // Tick the world
        world.tick(Duration::from_secs(1))?;

        // Poll sensors
        camera_sensor.poll_streaming()?;
        lidar_sensor.poll_streaming()?;
        collision_sensor.poll_streaming()?;

        frame_count += 1;

        // Print statistics every 100 frames
        if frame_count % 100 == 0 {
            let elapsed = start_time.elapsed();
            let fps = frame_count as f32 / elapsed.as_secs_f32();

            println!("\n--- Statistics ---");
            println!("Frames: {}", frame_count);
            println!("Average FPS: {:.2}", fps);

            // Get stream statistics
            if let Ok(camera_stats) = streaming_manager.get_stats(camera_stream_id) {
                println!("\nCamera Stream:");
                println!("  Frames received: {}", camera_stats.frames_received);
                println!("  Frames dropped: {}", camera_stats.frames_dropped);
                println!(
                    "  Buffer utilization: {:.1}%",
                    camera_stats.buffer_utilization * 100.0
                );
                println!("  Current FPS: {:.2}", camera_stats.current_fps);
                println!(
                    "  Avg processing time: {:.2}ms",
                    camera_stats.avg_processing_time_ms
                );
            }

            if let Ok(lidar_stats) = streaming_manager.get_stats(lidar_stream_id) {
                println!("\nLiDAR Stream:");
                println!("  Frames received: {}", lidar_stats.frames_received);
                println!("  Frames dropped: {}", lidar_stats.frames_dropped);
                println!(
                    "  Buffer utilization: {:.1}%",
                    lidar_stats.buffer_utilization * 100.0
                );
                println!("  Current FPS: {:.2}", lidar_stats.current_fps);
                println!(
                    "  Avg processing time: {:.2}ms",
                    lidar_stats.avg_processing_time_ms
                );
            }
            println!("------------------\n");
        }

        // Small delay to prevent busy waiting
        thread::sleep(Duration::from_millis(10));

        // Stop after 30 seconds for demo
        if start_time.elapsed() > Duration::from_secs(30) {
            println!("\nDemo complete!");
            break;
        }
    }

    // Cleanup
    println!("\nCleaning up...");
    vehicle.destroy()?;
    println!("Done!");

    Ok(())
}
