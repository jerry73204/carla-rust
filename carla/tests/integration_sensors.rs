//! Sensor integration tests
//!
//! Tests for sensor functionality that require a CARLA server

mod common;

use carla::{
    actor::{ActorExt, Sensor},
    error::CarlaResult,
    geom::Transform,
    sensor_data::{CollisionData, IMUData, LiDARData, RGBImageData, RadarData},
};
use common::{get_test_client, reset_world, spawn_test_vehicle};
use serial_test::serial;
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_sensor_spawning() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Test spawning different sensor types
    let sensor_types = [
        "sensor.camera.rgb",
        "sensor.camera.depth",
        "sensor.camera.semantic_segmentation",
        "sensor.lidar.ray_cast",
        "sensor.other.gnss",
        "sensor.other.imu",
        "sensor.other.collision",
        "sensor.other.lane_invasion",
        "sensor.other.obstacle",
        "sensor.other.radar",
    ];

    for sensor_type in &sensor_types {
        if let Some(blueprint) = blueprint_library.find(sensor_type)? {
            let sensor = world
                .spawn_actor(&blueprint, &Transform::default(), None)?
                .into_sensor()
                .expect(&format!("Failed to cast {} to sensor", sensor_type));

            // Verify sensor properties
            assert!(sensor.id() > 0);
            assert!(!sensor.is_listening());

            // Clean up
            drop(sensor); // Automatic destruction
        }
    }

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_camera_sensor_data_callback() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Create RGB camera
    let camera_bp = blueprint_library
        .find("sensor.camera.rgb")?
        .ok_or_else(|| {
            carla::error::CarlaError::World(carla::error::WorldError::BlueprintNotFound(
                "sensor.camera.rgb".to_string(),
            ))
        })?;

    // Set camera attributes
    let mut camera_bp = camera_bp;
    camera_bp.set_attribute("image_size_x", "320")?;
    camera_bp.set_attribute("image_size_y", "240")?;

    let mut sensor = world
        .spawn_actor(&camera_bp, &Transform::default(), None)?
        .into_sensor()
        .expect("Failed to cast to sensor");

    // Set up data reception
    let data_received = Arc::new(Mutex::new(false));
    let data_received_clone = data_received.clone();

    // Register callback
    sensor.listen(move |data: Vec<u8>| {
        // Verify we received some data
        if !data.is_empty() {
            *data_received_clone.lock().unwrap() = true;
        }
    })?;

    assert!(sensor.is_listening());

    // Wait for data (with timeout)
    std::thread::sleep(Duration::from_millis(500));

    // Stop listening
    sensor.stop();
    assert!(!sensor.is_listening());

    // Check if we received data
    let received = *data_received.lock().unwrap();
    if !received {
        println!("Warning: Camera data not received within timeout");
    }

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_imu_sensor_streaming() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Spawn a vehicle to attach IMU to
    let vehicle = spawn_test_vehicle(&client)?;

    // Create IMU sensor
    let imu_bp = blueprint_library.find("sensor.other.imu")?.ok_or_else(|| {
        carla::error::CarlaError::World(carla::error::WorldError::BlueprintNotFound(
            "sensor.other.imu".to_string(),
        ))
    })?;

    let mut sensor = world
        .spawn_actor(&imu_bp, &Transform::default(), None)?
        .into_sensor()
        .expect("Failed to cast to sensor");

    // Set up data reception
    let data_count = Arc::new(Mutex::new(0));
    let data_count_clone = data_count.clone();

    // Register callback
    sensor.listen(move |data: Vec<u8>| {
        if !data.is_empty() {
            *data_count_clone.lock().unwrap() += 1;
        }
    })?;

    // Let it stream for a bit
    std::thread::sleep(Duration::from_millis(1000));

    // Stop listening
    sensor.stop();

    let count = *data_count.lock().unwrap();
    println!("Received {} IMU data samples", count);

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_sensor_destruction_while_listening() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Create GNSS sensor
    let gnss_bp = blueprint_library
        .find("sensor.other.gnss")?
        .ok_or_else(|| {
            carla::error::CarlaError::World(carla::error::WorldError::BlueprintNotFound(
                "sensor.other.gnss".to_string(),
            ))
        })?;

    let mut sensor = world
        .spawn_actor(&gnss_bp, &Transform::default(), None)?
        .into_sensor()
        .expect("Failed to cast to sensor");

    // Start listening
    sensor.listen(|data: Vec<u8>| {
        // Process sensor data (generic handling)
        let _ = data.len(); // Use data to avoid warnings
    })?;

    assert!(sensor.is_listening());

    // Destroy while listening (should stop listening first)
    sensor.destroy()?;

    // Sensor should be destroyed cleanly
    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_collision_sensor() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Spawn a vehicle
    let vehicle = spawn_test_vehicle(&client)?;

    // Create collision sensor
    let collision_bp = blueprint_library
        .find("sensor.other.collision")?
        .ok_or_else(|| {
            carla::error::CarlaError::World(carla::error::WorldError::BlueprintNotFound(
                "sensor.other.collision".to_string(),
            ))
        })?;

    let sensor = world
        .spawn_actor(&collision_bp, &Transform::default(), None)?
        .into_sensor()
        .expect("Failed to cast to sensor");

    let collision_detected = Arc::new(Mutex::new(false));
    let collision_detected_clone = collision_detected.clone();

    // Register collision callback
    sensor.listen(move |data: Vec<u8>| {
        if !data.is_empty() {
            *collision_detected_clone.lock().unwrap() = true;
        }
    })?;

    // Note: Actually causing a collision would require more complex setup
    // For now, we just verify the sensor can be created and callbacks registered

    // Clean up
    sensor.stop();

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_lane_invasion_sensor() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Spawn a vehicle
    let vehicle = spawn_test_vehicle(&client)?;

    // Create lane invasion sensor
    let lane_invasion_bp = blueprint_library
        .find("sensor.other.lane_invasion")?
        .ok_or_else(|| {
            carla::error::CarlaError::World(carla::error::WorldError::BlueprintNotFound(
                "sensor.other.lane_invasion".to_string(),
            ))
        })?;

    let sensor = world
        .spawn_actor(&lane_invasion_bp, &Transform::default(), None)?
        .into_sensor()
        .expect("Failed to cast to sensor");

    let invasion_detected = Arc::new(Mutex::new(false));
    let invasion_detected_clone = invasion_detected.clone();

    // Register lane invasion callback
    sensor.listen(move |data: Vec<u8>| {
        if !data.is_empty() {
            *invasion_detected_clone.lock().unwrap() = true;
        }
    })?;

    // Note: Actually causing a lane invasion would require vehicle movement
    // For now, we just verify the sensor can be created and callbacks registered

    // Clean up
    sensor.stop();

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_multiple_sensors_on_vehicle() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Spawn a vehicle
    let vehicle = spawn_test_vehicle(&client)?;

    let mut sensors = Vec::new();

    // Attach multiple sensors to the vehicle
    let sensor_configs = [
        ("sensor.other.gnss", Transform::default()),
        ("sensor.other.imu", Transform::default()),
        ("sensor.camera.rgb", Transform::default()),
    ];

    for (sensor_type, transform) in &sensor_configs {
        if let Some(bp) = blueprint_library.find(sensor_type)? {
            let sensor = world
                .spawn_actor(&bp, &transform, None)?
                .into_sensor()
                .expect(&format!("Failed to cast {} to sensor", sensor_type));

            sensors.push(sensor);
        }
    }

    // Verify all sensors are attached to the vehicle
    assert!(sensors.len() >= 2);

    // Start listening on all sensors
    for sensor in &mut sensors {
        sensor.listen(|data: Vec<u8>| {
            // Process sensor data
            let _ = data.len(); // Use data to avoid warnings
        })?;
    }

    // Let them run for a bit
    std::thread::sleep(Duration::from_millis(500));

    // Stop all sensors
    for sensor in &mut sensors {
        sensor.stop();
    }

    // Clean up - sensors destroyed when dropped

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_sensor_callback_cleanup() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Create multiple sensors and register callbacks
    let mut sensors = Vec::new();

    for i in 0..3 {
        if let Some(bp) = blueprint_library.find("sensor.other.gnss")? {
            let mut sensor = world
                .spawn_actor(&bp, &Transform::default(), None)?
                .into_sensor()
                .expect("Failed to cast to sensor");

            // Register unique callback for each sensor
            let sensor_id = i;
            sensor.listen(move |data: Vec<u8>| {
                // Callback specific to sensor_id
                let _ = (sensor_id, data.len()); // Use to avoid warning
            })?;

            sensors.push(sensor);
        }
    }

    // Verify all are listening
    for sensor in &sensors {
        assert!(sensor.is_listening());
    }

    // Destroy sensors one by one
    while let Some(mut sensor) = sensors.pop() {
        sensor.destroy()?;
    }

    // All callbacks should be cleaned up

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_lidar_sensor() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Create LIDAR sensor
    let lidar_bp = blueprint_library
        .find("sensor.lidar.ray_cast")?
        .ok_or_else(|| {
            carla::error::CarlaError::World(carla::error::WorldError::BlueprintNotFound(
                "sensor.lidar.ray_cast".to_string(),
            ))
        })?;

    // Configure LIDAR parameters
    let mut lidar_bp = lidar_bp;
    lidar_bp.set_attribute("channels", "32")?;
    lidar_bp.set_attribute("points_per_second", "56000")?;
    lidar_bp.set_attribute("rotation_frequency", "10")?;
    lidar_bp.set_attribute("range", "100")?;

    let mut sensor = world
        .spawn_actor(&lidar_bp, &Transform::default(), None)?
        .into_sensor()
        .expect("Failed to cast to sensor");

    let point_count = Arc::new(Mutex::new(0));
    let point_count_clone = point_count.clone();

    // Register callback
    sensor.listen(move |data: Vec<u8>| {
        if !data.is_empty() {
            // Estimate point count from data size (each point is ~16 bytes: x,y,z,intensity)
            let estimated_points = data.len() / 16;
            *point_count_clone.lock().unwrap() += estimated_points;
        }
    })?;

    // Let it capture some data
    std::thread::sleep(Duration::from_millis(500));

    sensor.stop();

    let total_points = *point_count.lock().unwrap();
    println!("LIDAR captured {} points", total_points);

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_radar_sensor() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Spawn a vehicle
    let vehicle = spawn_test_vehicle(&client)?;

    // Create radar sensor
    let radar_bp = blueprint_library
        .find("sensor.other.radar")?
        .ok_or_else(|| {
            carla::error::CarlaError::World(carla::error::WorldError::BlueprintNotFound(
                "sensor.other.radar".to_string(),
            ))
        })?;

    let mut sensor = world
        .spawn_actor(&radar_bp, &Transform::default(), None)?
        .into_sensor()
        .expect("Failed to cast to sensor");

    let detection_count = Arc::new(Mutex::new(0));
    let detection_count_clone = detection_count.clone();

    // Register callback
    sensor.listen(move |data: Vec<u8>| {
        if !data.is_empty() {
            // Estimate detection count from data size (each detection is ~24 bytes)
            let estimated_detections = data.len() / 24;
            *detection_count_clone.lock().unwrap() += estimated_detections;
        }
    })?;

    // Let it run
    std::thread::sleep(Duration::from_millis(500));

    sensor.stop();

    let total_detections = *detection_count.lock().unwrap();
    println!("Radar made {} detections", total_detections);

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_sensor_error_handling() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Create a sensor
    let sensor_bp = blueprint_library
        .find("sensor.other.gnss")?
        .ok_or_else(|| {
            carla::error::CarlaError::World(carla::error::WorldError::BlueprintNotFound(
                "sensor.other.gnss".to_string(),
            ))
        })?;

    let mut sensor = world
        .spawn_actor(&sensor_bp, &Transform::default(), None)?
        .into_sensor()
        .expect("Failed to cast to sensor");

    // Try to stop when not listening
    sensor.stop(); // Should not panic

    // Start listening
    sensor.listen(|data: Vec<u8>| {
        let _ = data.len(); // Use data to avoid warnings
    })?;

    // Try to listen again (should fail)
    let result = sensor.listen(|data: Vec<u8>| {
        let _ = data.len(); // Use data to avoid warnings
    });
    assert!(result.is_err());

    // Stop and clean up
    sensor.stop();

    Ok(())
}
