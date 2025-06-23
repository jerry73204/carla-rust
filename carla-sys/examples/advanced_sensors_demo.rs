//! Advanced Sensor Features Demo for CARLA CXX
//!
//! This example demonstrates how to use CARLA's advanced sensor features through carla-sys,
//! including Dynamic Vision Sensor (DVS), Obstacle Detection, Semantic LiDAR, and RSS sensors.
//!
//! ## Features Demonstrated
//!
//! 1. **Dynamic Vision Sensor (DVS)** - Event-based camera simulation
//! 2. **Obstacle Detection Sensor** - Real-time obstacle detection events  
//! 3. **Semantic LiDAR** - Enhanced LiDAR with semantic information
//! 4. **RSS (Road Safety) Sensor** - Intel RSS safety validation
//!
//! ## Usage
//!
//! ```bash
//! # Start CARLA server first
//! ./CarlaUE4.sh
//!
//! # Run the demo
//! cargo run --example advanced_sensors_demo
//! ```

use anyhow::Result;
use carla_sys::{
    ClientWrapper, DVSEventArray, ObstacleDetectionEvent, RssResponse, SemanticLidarData,
    SensorWrapper, SimpleLocation, SimpleRotation, SimpleTransform,
};
use std::time::Duration;

fn main() -> Result<()> {
    println!("🚗 CARLA Advanced Sensors Demo");
    println!("=============================");

    // Connect to CARLA server
    println!("\n📡 Connecting to CARLA server...");
    let mut client = ClientWrapper::new("localhost", 2000)?;
    client.set_timeout(Duration::from_secs(10));
    let world = client.get_world()?;

    println!(
        "✅ Connected to CARLA server version: {}",
        client.get_server_version()
    );
    println!("🌍 World ID: {}", world.get_id());

    // Get blueprint library
    let blueprint_library = world.get_blueprint_library()?;
    println!(
        "📚 Blueprint library loaded with {} blueprints",
        blueprint_library.size()
    );

    // Define spawn location
    let spawn_transform = SimpleTransform::new(
        SimpleLocation::new(0.0, 0.0, 2.0), // Higher spawn point for sensors
        SimpleRotation::ZERO,
    );

    println!("\n🔧 Setting up advanced sensors...");

    // Demonstrate DVS (Dynamic Vision Sensor)
    demonstrate_dvs_sensor(&world, &blueprint_library, &spawn_transform)?;

    // Demonstrate Obstacle Detection Sensor
    demonstrate_obstacle_detection_sensor(&world, &blueprint_library, &spawn_transform)?;

    // Demonstrate Semantic LiDAR
    demonstrate_semantic_lidar_sensor(&world, &blueprint_library, &spawn_transform)?;

    // Demonstrate RSS (Road Safety) Sensor
    demonstrate_rss_sensor(&world, &blueprint_library, &spawn_transform)?;

    println!("\n✅ Advanced sensors demo completed successfully!");
    println!("📊 Summary of advanced sensor capabilities:");
    println!("   • DVS: Event-based vision with neuromorphic processing");
    println!("   • Obstacle Detection: Real-time collision avoidance");
    println!("   • Semantic LiDAR: Object classification with 3D data");
    println!("   • RSS: Intel's safety validation framework");

    Ok(())
}

/// Demonstrate Dynamic Vision Sensor (DVS) functionality
fn demonstrate_dvs_sensor(
    world: &carla_sys::WorldWrapper,
    blueprint_library: &carla_sys::BlueprintLibraryWrapper,
    spawn_transform: &SimpleTransform,
) -> Result<()> {
    println!("\n🎥 Dynamic Vision Sensor (DVS) Demo");
    println!("====================================");

    // Try to find DVS camera blueprint
    if let Some(dvs_bp) = blueprint_library.find("sensor.camera.dvs") {
        println!("📋 Found DVS camera blueprint: {}", dvs_bp.get_id());

        // Note: DVS configuration would typically include:
        // - Image resolution (width x height)
        // - Field of view (FOV)
        // - Positive/negative thresholds for event generation
        // - Noise parameters and refractory period
        println!("⚙️  DVS sensor features:");
        println!("   • Event-based vision processing");
        println!("   • Neuromorphic camera simulation");
        println!("   • Configurable event thresholds");
        println!("   • Logarithmic intensity calculation");

        // Spawn DVS sensor
        if let Ok(dvs_actor) = world.spawn_actor(&dvs_bp, spawn_transform, None) {
            if let Some(dvs_sensor) = SensorWrapper::from_actor(dvs_actor.get_shared_ptr()) {
                println!("🎯 DVS sensor spawned with ID: {}", dvs_actor.get_id());

                // Start listening
                dvs_sensor.listen();
                println!("👂 DVS sensor listening for events...");

                // Simulate some time for event generation
                std::thread::sleep(Duration::from_millis(500));

                // Get DVS data
                let dvs_data: DVSEventArray = dvs_sensor.get_last_dvs_data();
                println!("📊 DVS Data received:");
                println!("   • Image size: {}x{}", dvs_data.width, dvs_data.height);
                println!("   • Field of view: {:.1}°", dvs_data.fov_angle);
                println!("   • Events captured: {}", dvs_data.events.len());

                if !dvs_data.events.is_empty() {
                    let event = &dvs_data.events[0];
                    println!(
                        "   • First event: x={}, y={}, t={}, pol={}",
                        event.x, event.y, event.t, event.pol
                    );
                }

                dvs_sensor.stop();
                dvs_actor.destroy();
                println!("✅ DVS sensor demo completed");
            }
        } else {
            println!("❌ Failed to spawn DVS sensor");
        }
    } else {
        println!("⚠️  DVS camera blueprint not found (requires CARLA 0.10.0+)");
    }

    Ok(())
}

/// Demonstrate Obstacle Detection Sensor functionality
fn demonstrate_obstacle_detection_sensor(
    world: &carla_sys::WorldWrapper,
    blueprint_library: &carla_sys::BlueprintLibraryWrapper,
    spawn_transform: &SimpleTransform,
) -> Result<()> {
    println!("\n🚧 Obstacle Detection Sensor Demo");
    println!("==================================");

    // Try to find obstacle detection sensor blueprint
    if let Some(obstacle_bp) = blueprint_library.find("sensor.other.obstacle") {
        println!(
            "📋 Found obstacle detection blueprint: {}",
            obstacle_bp.get_id()
        );

        // Note: Obstacle detection configuration would typically include:
        // - Detection distance (range)
        // - Hit radius for collision detection
        // - Filter for dynamic vs static objects
        // - Debug visualization options
        println!("⚙️  Obstacle detection features:");
        println!("   • Configurable detection range");
        println!("   • Ray-casting based detection");
        println!("   • Static and dynamic object filtering");
        println!("   • Real-time collision avoidance data");

        // Spawn obstacle detection sensor
        if let Ok(obstacle_actor) = world.spawn_actor(&obstacle_bp, spawn_transform, None) {
            if let Some(obstacle_sensor) =
                SensorWrapper::from_actor(obstacle_actor.get_shared_ptr())
            {
                println!(
                    "🎯 Obstacle detection sensor spawned with ID: {}",
                    obstacle_actor.get_id()
                );

                // Start listening
                obstacle_sensor.listen();
                println!("👂 Obstacle sensor listening for detections...");

                // Simulate some time for detection
                std::thread::sleep(Duration::from_millis(500));

                // Get obstacle detection data
                let obstacle_data: ObstacleDetectionEvent =
                    obstacle_sensor.get_last_obstacle_detection_data();
                println!("📊 Obstacle Detection Data:");
                println!("   • Self actor ID: {}", obstacle_data.self_actor_id);
                println!("   • Other actor ID: {}", obstacle_data.other_actor_id);
                println!("   • Distance: {:.2} meters", obstacle_data.distance);

                if obstacle_data.distance > 0.0 {
                    println!("🚨 Obstacle detected at {:.2}m!", obstacle_data.distance);
                } else {
                    println!("✅ No obstacles detected in range");
                }

                obstacle_sensor.stop();
                obstacle_actor.destroy();
                println!("✅ Obstacle detection sensor demo completed");
            }
        } else {
            println!("❌ Failed to spawn obstacle detection sensor");
        }
    } else {
        println!("⚠️  Obstacle detection sensor blueprint not found");
    }

    Ok(())
}

/// Demonstrate Semantic LiDAR functionality
fn demonstrate_semantic_lidar_sensor(
    world: &carla_sys::WorldWrapper,
    blueprint_library: &carla_sys::BlueprintLibraryWrapper,
    spawn_transform: &SimpleTransform,
) -> Result<()> {
    println!("\n🔍 Semantic LiDAR Sensor Demo");
    println!("=============================");

    // Try to find semantic LiDAR blueprint
    if let Some(semantic_lidar_bp) = blueprint_library.find("sensor.lidar.ray_cast_semantic") {
        println!(
            "📋 Found semantic LiDAR blueprint: {}",
            semantic_lidar_bp.get_id()
        );

        // Note: Semantic LiDAR configuration would typically include:
        // - Number of channels (laser beams)
        // - Detection range and rotation frequency
        // - Points per second (resolution)
        // - Vertical field of view (upper/lower FOV)
        println!("⚙️  Semantic LiDAR features:");
        println!("   • Multi-channel laser simulation");
        println!("   • Semantic object classification");
        println!("   • Configurable range and resolution");
        println!("   • Object instance indexing");

        // Spawn semantic LiDAR sensor
        if let Ok(lidar_actor) = world.spawn_actor(&semantic_lidar_bp, spawn_transform, None) {
            if let Some(lidar_sensor) = SensorWrapper::from_actor(lidar_actor.get_shared_ptr()) {
                println!(
                    "🎯 Semantic LiDAR sensor spawned with ID: {}",
                    lidar_actor.get_id()
                );

                // Start listening
                lidar_sensor.listen();
                println!("👂 Semantic LiDAR sensor listening...");

                // Simulate some time for data collection
                std::thread::sleep(Duration::from_millis(500));

                // Get semantic LiDAR data
                let semantic_data: SemanticLidarData = lidar_sensor.get_last_semantic_lidar_data();
                println!("📊 Semantic LiDAR Data:");
                println!(
                    "   • Horizontal angle: {:.2}°",
                    semantic_data.horizontal_angle
                );
                println!("   • Channel count: {}", semantic_data.channel_count);
                println!("   • Detections: {}", semantic_data.detections.len());

                if !semantic_data.detections.is_empty() {
                    let detection = &semantic_data.detections[0];
                    println!("   • First detection:");
                    println!(
                        "     - Position: ({:.2}, {:.2}, {:.2})",
                        detection.point.x, detection.point.y, detection.point.z
                    );
                    println!("     - Cosine incidence: {:.3}", detection.cos_inc_angle);
                    println!("     - Object index: {}", detection.object_idx);
                    println!("     - Semantic tag: {}", detection.object_tag);
                }

                lidar_sensor.stop();
                lidar_actor.destroy();
                println!("✅ Semantic LiDAR sensor demo completed");
            }
        } else {
            println!("❌ Failed to spawn semantic LiDAR sensor");
        }
    } else {
        println!("⚠️  Semantic LiDAR blueprint not found");
    }

    Ok(())
}

/// Demonstrate RSS (Road Safety) Sensor functionality
fn demonstrate_rss_sensor(
    world: &carla_sys::WorldWrapper,
    blueprint_library: &carla_sys::BlueprintLibraryWrapper,
    spawn_transform: &SimpleTransform,
) -> Result<()> {
    println!("\n🛡️  RSS (Road Safety) Sensor Demo");
    println!("==================================");

    // Try to find RSS sensor blueprint
    if let Some(rss_bp) = blueprint_library.find("sensor.other.rss") {
        println!("📋 Found RSS sensor blueprint: {}", rss_bp.get_id());

        // RSS sensors typically don't have many configurable attributes
        // They work with the vehicle's current dynamics and road situation
        println!("⚙️  RSS sensor uses Intel's Responsibility-Sensitive Safety framework");
        println!("   • Ego vehicle dynamics monitoring");
        println!("   • Other vehicle behavior analysis");
        println!("   • Road boundaries and traffic rules");
        println!("   • Real-time safety validation");

        // Spawn RSS sensor
        if let Ok(rss_actor) = world.spawn_actor(&rss_bp, spawn_transform, None) {
            if let Some(rss_sensor) = SensorWrapper::from_actor(rss_actor.get_shared_ptr()) {
                println!("🎯 RSS sensor spawned with ID: {}", rss_actor.get_id());

                // Start listening
                rss_sensor.listen();
                println!("👂 RSS sensor monitoring safety...");

                // Simulate some time for RSS analysis
                std::thread::sleep(Duration::from_millis(500));

                // Get RSS data
                let rss_data: RssResponse = rss_sensor.get_last_rss_data();
                println!("📊 RSS Safety Analysis:");
                println!("   • Response valid: {}", rss_data.response_valid);

                if rss_data.response_valid {
                    println!(
                        "   • Proper response: {} chars",
                        rss_data.proper_response.len()
                    );
                    println!(
                        "   • RSS state snapshot: {} chars",
                        rss_data.rss_state_snapshot.len()
                    );
                    println!(
                        "   • Situation snapshot: {} chars",
                        rss_data.situation_snapshot.len()
                    );
                    println!("   • World model: {} chars", rss_data.world_model.len());
                    println!(
                        "   • Ego dynamics: {} chars",
                        rss_data.ego_dynamics_on_route.len()
                    );
                    println!("✅ RSS safety validation active");
                } else {
                    println!("⚠️  RSS analysis not available (requires proper scenario setup)");
                }

                rss_sensor.stop();
                rss_actor.destroy();
                println!("✅ RSS sensor demo completed");
            }
        } else {
            println!("❌ Failed to spawn RSS sensor");
        }
    } else {
        println!("⚠️  RSS sensor blueprint not found (may be deprecated in CARLA 0.10.0)");
    }

    Ok(())
}
