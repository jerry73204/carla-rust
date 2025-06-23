//! Streaming integration tests
//!
//! Tests for sensor data streaming that require a CARLA server

mod common;

#[cfg(feature = "test-carla-server")]
use carla::{
    actor::ActorExt,
    error::CarlaResult,
    geom::Transform,
    streaming::{SensorStream, StreamConfig},
};

#[cfg(feature = "test-carla-server")]
use common::{get_test_client, reset_world};

#[cfg(feature = "test-carla-server")]
use serial_test::serial;

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_sensor_stream_creation() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    // Create a sensor stream with default config
    let stream = SensorStream::new(StreamConfig::default());

    // Verify initial state
    assert_eq!(stream.subscription_count(), 0);

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_sensor_subscription() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Create a camera sensor
    let camera_bp = blueprint_library
        .find("sensor.camera.rgb")?
        .ok_or_else(|| {
            carla::error::CarlaError::World(carla::error::WorldError::BlueprintNotFound(
                "sensor.camera.rgb".to_string(),
            ))
        })?;

    let sensor = world
        .spawn_actor(&camera_bp, &Transform::default(), None)?
        .into_sensor()
        .expect("Failed to cast to sensor");

    let sensor_id = sensor.id();

    // Create stream and subscribe
    let stream = SensorStream::new(StreamConfig::default());
    stream.subscribe(sensor_id)?;

    // Verify subscription
    assert_eq!(stream.subscription_count(), 1);
    assert!(stream.is_subscribed(sensor_id));

    // Clean up
    drop(sensor);

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_sensor_unsubscription() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Create a sensor
    let imu_bp = blueprint_library.find("sensor.other.imu")?.ok_or_else(|| {
        carla::error::CarlaError::World(carla::error::WorldError::BlueprintNotFound(
            "sensor.other.imu".to_string(),
        ))
    })?;

    let sensor = world
        .spawn_actor(&imu_bp, &Transform::default(), None)?
        .into_sensor()
        .expect("Failed to cast to sensor");

    let sensor_id = sensor.id();

    // Create stream, subscribe and then unsubscribe
    let stream = SensorStream::new(StreamConfig::default());
    stream.subscribe(sensor_id)?;
    assert!(stream.is_subscribed(sensor_id));

    stream.unsubscribe(sensor_id)?;
    assert!(!stream.is_subscribed(sensor_id));
    assert_eq!(stream.subscription_count(), 0);

    // Clean up
    drop(sensor);

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_multiple_sensor_streams() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Create multiple sensors
    let mut sensors = Vec::new();

    // Camera
    if let Some(camera_bp) = blueprint_library.find("sensor.camera.rgb")? {
        let sensor = world
            .spawn_actor(&camera_bp, &Transform::default(), None)?
            .into_sensor()
            .expect("Failed to cast to sensor");
        sensors.push(sensor);
    }

    // IMU
    if let Some(imu_bp) = blueprint_library.find("sensor.other.imu")? {
        let sensor = world
            .spawn_actor(&imu_bp, &Transform::default(), None)?
            .into_sensor()
            .expect("Failed to cast to sensor");
        sensors.push(sensor);
    }

    // GNSS
    if let Some(gnss_bp) = blueprint_library.find("sensor.other.gnss")? {
        let sensor = world
            .spawn_actor(&gnss_bp, &Transform::default(), None)?
            .into_sensor()
            .expect("Failed to cast to sensor");
        sensors.push(sensor);
    }

    // Create stream and subscribe all sensors
    let stream = SensorStream::new(StreamConfig::default());
    let sensor_ids: Vec<_> = sensors.iter().map(|s| s.id()).collect();

    for &id in &sensor_ids {
        stream.subscribe(id)?;
    }

    // Verify all subscriptions
    assert_eq!(stream.subscription_count(), sensors.len());
    for &id in &sensor_ids {
        assert!(stream.is_subscribed(id));
    }

    // Unsubscribe one sensor
    if let Some(&first_id) = sensor_ids.first() {
        stream.unsubscribe(first_id)?;
        assert!(!stream.is_subscribed(first_id));
        assert_eq!(stream.subscription_count(), sensors.len() - 1);
    }

    // Clean up - sensors destroyed when dropped

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_stream_buffer_management() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Create a sensor
    let sensor_bp = blueprint_library.find("sensor.other.imu")?.ok_or_else(|| {
        carla::error::CarlaError::World(carla::error::WorldError::BlueprintNotFound(
            "sensor.other.imu".to_string(),
        ))
    })?;

    let sensor = world
        .spawn_actor(&sensor_bp, &Transform::default(), None)?
        .into_sensor()
        .expect("Failed to cast to sensor");

    let sensor_id = sensor.id();

    // Create stream with custom config
    let config = StreamConfig {
        buffer_size: 5,
        drop_old_data: true,
        ..Default::default()
    };

    let stream = SensorStream::new(config);
    stream.subscribe(sensor_id)?;

    // Check buffer operations
    assert_eq!(stream.buffer_size(sensor_id), 0);
    assert!(!stream.has_latest_data(sensor_id));

    // Clear buffer (should work even if empty)
    stream.clear_buffer(sensor_id)?;

    // Clean up
    drop(sensor);

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_stream_error_handling() -> CarlaResult<()> {
    let stream = SensorStream::new(StreamConfig::default());

    // Try to unsubscribe non-existent sensor
    let fake_id = 99999;
    let result = stream.unsubscribe(fake_id);
    assert!(result.is_err());

    // Try to clear buffer for non-subscribed sensor
    let result = stream.clear_buffer(fake_id);
    assert!(result.is_err());

    // Test max sensors limit
    let config = StreamConfig {
        max_sensors: 2,
        ..Default::default()
    };
    let limited_stream = SensorStream::new(config);

    // Subscribe up to limit should work
    limited_stream.subscribe(1)?;
    limited_stream.subscribe(2)?;

    // Exceeding limit should fail
    let result = limited_stream.subscribe(3);
    assert!(result.is_err());

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_stream_lifecycle_with_sensor_destruction() -> CarlaResult<()> {
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

    let sensor_id = sensor.id();

    // Create stream and subscribe
    let stream = SensorStream::new(StreamConfig::default());
    stream.subscribe(sensor_id)?;
    assert!(stream.is_subscribed(sensor_id));

    // Destroy sensor
    sensor.destroy()?;

    // Stream should still show subscription (cleanup is manual)
    assert!(stream.is_subscribed(sensor_id));

    // Manual unsubscribe
    stream.unsubscribe(sensor_id)?;

    Ok(())
}

#[test]
#[serial]
#[cfg(feature = "test-carla-server")]
fn test_get_subscribed_sensors() -> CarlaResult<()> {
    let client = get_test_client()?;
    reset_world(&client)?;

    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Create sensors
    let mut sensors = Vec::new();
    let sensor_types = ["sensor.camera.rgb", "sensor.other.imu", "sensor.other.gnss"];

    for sensor_type in &sensor_types {
        if let Some(bp) = blueprint_library.find(sensor_type)? {
            let sensor = world
                .spawn_actor(&bp, &Transform::default(), None)?
                .into_sensor()
                .expect("Failed to cast to sensor");
            sensors.push(sensor);
        }
    }

    // Create stream and subscribe sensors
    let stream = SensorStream::new(StreamConfig::default());
    let sensor_ids: Vec<_> = sensors.iter().map(|s| s.id()).collect();

    for &id in &sensor_ids {
        stream.subscribe(id)?;
    }

    // Get subscribed sensors
    let subscribed_sensors = stream.subscribed_sensors();
    assert_eq!(subscribed_sensors.len(), sensors.len());

    // Verify all sensor IDs are in subscribed list
    for &id in &sensor_ids {
        assert!(subscribed_sensors.contains(&id));
    }

    Ok(())
}
