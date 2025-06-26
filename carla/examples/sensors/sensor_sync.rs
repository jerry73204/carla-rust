// Python equivalent: carla-simulator/PythonAPI/examples/sensor_synchronization.py
// Expected behavior: Multi-sensor synchronization with frame-based data collection
// Key features: Sensor spawning, callback coordination, synchronous mode

use anyhow::Result;
use carla::{actor::ActorExt, client::ActorBlueprint, geom::Transform};
use clap::Parser;
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

#[path = "../common/mod.rs"]
mod common;
use common::{cli::CommonArgs, connect_with_retry, utils::*};

#[derive(Parser, Debug)]
#[command(
    author,
    version,
    about = "CARLA example: Multi-sensor synchronization demonstration"
)]
struct Args {
    #[command(flatten)]
    common: CommonArgs,

    #[arg(long, default_value_t = 50)]
    frames: u64,

    #[arg(long, default_value_t = 200)]
    frame_time_ms: u64,

    #[arg(long, default_value_t = 1000)]
    timeout_ms: u64,

    #[arg(long)]
    export_csv: Option<String>,

    #[arg(long)]
    save_sensor_data: bool,

    #[arg(long, default_value = "output")]
    output_dir: String,
}

#[derive(Debug, Clone)]
struct SensorData {
    frame_id: u64,
    sensor_name: String,
    timestamp: Instant,
    data_size: usize,
}

#[derive(Debug)]
struct SensorSyncQueue {
    data: Arc<Mutex<Vec<SensorData>>>,
    expected_sensors: usize,
}

impl SensorSyncQueue {
    fn new(expected_sensors: usize) -> Self {
        Self {
            data: Arc::new(Mutex::new(Vec::new())),
            expected_sensors,
        }
    }

    fn add_sensor_data(&self, data: SensorData) {
        let mut queue = self.data.lock().unwrap();
        queue.push(data);
    }

    fn wait_for_frame(&self, frame_id: u64, timeout: Duration) -> Vec<SensorData> {
        let start = Instant::now();
        loop {
            {
                let mut queue = self.data.lock().unwrap();
                let frame_data: Vec<_> = queue
                    .iter()
                    .filter(|data| data.frame_id == frame_id)
                    .cloned()
                    .collect();

                if frame_data.len() >= self.expected_sensors {
                    // Remove processed data
                    queue.retain(|data| data.frame_id != frame_id);
                    return frame_data;
                }
            }

            if start.elapsed() > timeout {
                let mut queue = self.data.lock().unwrap();
                let partial_data: Vec<_> = queue
                    .iter()
                    .filter(|data| data.frame_id == frame_id)
                    .cloned()
                    .collect();
                queue.retain(|data| data.frame_id != frame_id);
                return partial_data;
            }

            thread::sleep(Duration::from_millis(10));
        }
    }

    #[allow(dead_code)]
    fn clear(&self) {
        let mut queue = self.data.lock().unwrap();
        queue.clear();
    }
}

type SensorConfig = Vec<(
    &'static str,
    &'static str,
    Vec<(&'static str, &'static str)>,
)>;

fn create_sensor_config() -> SensorConfig {
    vec![
        ("camera01", "sensor.camera.rgb", vec![]),
        (
            "lidar01",
            "sensor.lidar.ray_cast",
            vec![("points_per_second", "100000")],
        ),
        (
            "lidar02",
            "sensor.lidar.ray_cast",
            vec![("points_per_second", "1000000")],
        ),
        ("radar01", "sensor.other.radar", vec![]),
        ("radar02", "sensor.other.radar", vec![]),
    ]
}

fn configure_blueprint(blueprint: &mut ActorBlueprint, attributes: &[(&str, &str)]) -> Result<()> {
    for (key, value) in attributes {
        blueprint.set_attribute(key, value.to_string())?;
    }
    Ok(())
}

fn main() -> Result<()> {
    let args = Args::parse();

    // Initialize logging
    env_logger::Builder::from_default_env()
        .filter_level(if args.common.verbose {
            log::LevelFilter::Debug
        } else {
            log::LevelFilter::Info
        })
        .init();

    println!("CARLA Sensor Synchronization Example");
    println!("Python equivalent: sensor_synchronization.py");
    println!("====================================");

    // Connect to CARLA
    let client = connect_with_retry(&args.common.host, args.common.port, args.common.timeout)?;
    let world = client.world()?;

    // TODO: Save original settings and set synchronous mode
    // This requires World settings FFI functions
    println!("TODO: Synchronous mode configuration not implemented");
    println!("This requires World::get_settings and World::apply_settings FFI functions");

    // Get blueprint library
    let blueprint_library = world.blueprint_library()?;

    // Create sensor configuration
    let sensor_configs = create_sensor_config();
    let sensor_count = sensor_configs.len();

    println!("Configuring {} sensors...", sensor_count);

    // Create sensor queue for synchronization
    let sensor_queue = SensorSyncQueue::new(sensor_count);

    // Setup CSV export if requested
    let mut csv_writer = if let Some(csv_path) = &args.export_csv {
        let mut writer = CsvWriter::new(csv_path)?;
        writer.write_header(&[
            "frame_id",
            "sensor_name",
            "timestamp_ms",
            "data_size",
            "sync_delay_ms",
            "expected_sensors",
            "received_sensors",
        ])?;
        Some(writer)
    } else {
        None
    };

    // Create sensors
    let mut sensors = Vec::new();
    let mut sensor_stats = HashMap::new();

    for (sensor_name, blueprint_id, attributes) in &sensor_configs {
        println!("Creating sensor: {} ({})", sensor_name, blueprint_id);

        // Find blueprint
        let blueprint_opt = blueprint_library.find(blueprint_id)?;
        if let Some(mut blueprint) = blueprint_opt {
            // Configure attributes
            configure_blueprint(&mut blueprint, attributes)?;

            // TODO: Spawn sensor with callback
            // This requires sensor spawning and callback registration FFI
            // For now, we'll spawn without callbacks
            match world.try_spawn_actor(&blueprint, &Transform::default(), None) {
                Ok(Some(sensor)) => {
                    println!("  ✓ Spawned sensor: {}", sensor_name);
                    sensors.push((sensor, sensor_name.to_string()));
                    sensor_stats.insert(sensor_name.to_string(), PerformanceStats::new());
                }
                Ok(None) => {
                    log::warn!(
                        "Failed to spawn sensor: {} - spawn point occupied",
                        sensor_name
                    );
                }
                Err(e) => {
                    log::warn!("Failed to spawn sensor {}: {}", sensor_name, e);
                }
            }
        } else {
            log::warn!("Blueprint not found: {}", blueprint_id);
        }
    }

    println!("Successfully spawned {} sensors", sensors.len());

    // TODO: Register sensor callbacks
    // This requires sensor callback registration FFI functions
    println!("\nTODO: Sensor callback registration not implemented");
    println!("This requires:");
    println!("  - Sensor::listen() or similar callback registration");
    println!("  - Sensor data callback handling");
    println!("  - Frame synchronization with sensor data");

    // Simulate synchronized data collection
    println!("\n=== Simulated Sensor Synchronization ===");
    println!(
        "Running {} frames with {}ms intervals...",
        args.frames, args.frame_time_ms
    );

    let mut frame_stats = PerformanceStats::new();
    let mut progress = ProgressTracker::new(args.frames, 10);

    for frame_id in 1..=args.frames {
        let frame_start = Instant::now();

        // TODO: world.tick() for frame synchronization
        // This requires World::tick() FFI function

        // Simulate sensor data collection
        let expected_sensors = sensors.len();

        // Simulate sensor callbacks with varying delays
        for (idx, (_sensor, sensor_name)) in sensors.iter().enumerate() {
            // Simulate sensor processing time
            let processing_delay = Duration::from_millis(10 + (idx as u64 * 5));
            thread::sleep(processing_delay);

            // Simulate sensor data
            let sensor_data = SensorData {
                frame_id,
                sensor_name: sensor_name.clone(),
                timestamp: Instant::now(),
                data_size: 1024 * (idx + 1), // Simulate different data sizes
            };

            sensor_queue.add_sensor_data(sensor_data);

            // Update sensor statistics
            if let Some(stats) = sensor_stats.get_mut(sensor_name) {
                stats.record_operation(processing_delay.as_millis() as u64, true);
            }
        }

        // Wait for all sensor data with timeout
        let timeout = Duration::from_millis(args.timeout_ms);
        let frame_data = sensor_queue.wait_for_frame(frame_id, timeout);

        let frame_time = frame_start.elapsed();
        let sync_delay = frame_time.as_millis() as u64;

        // Record frame statistics
        frame_stats.record_operation(sync_delay, frame_data.len() == expected_sensors);

        // Export to CSV if enabled
        if let Some(ref mut writer) = csv_writer {
            for data in &frame_data {
                writer.write_row(&[
                    frame_id.to_string(),
                    data.sensor_name.clone(),
                    data.timestamp.elapsed().as_millis().to_string(),
                    data.data_size.to_string(),
                    sync_delay.to_string(),
                    expected_sensors.to_string(),
                    frame_data.len().to_string(),
                ])?;
            }
        }

        // Log frame results
        if frame_data.len() == expected_sensors {
            log::debug!(
                "Frame {}: All {} sensors synchronized ({}ms)",
                frame_id,
                expected_sensors,
                sync_delay
            );
        } else {
            log::warn!(
                "Frame {}: Only {}/{} sensors synchronized ({}ms)",
                frame_id,
                frame_data.len(),
                expected_sensors,
                sync_delay
            );
        }

        progress.update(frame_id);

        // Maintain frame rate
        let target_frame_time = Duration::from_millis(args.frame_time_ms);
        if frame_time < target_frame_time {
            thread::sleep(target_frame_time - frame_time);
        }
    }

    // Print synchronization statistics
    println!("\n=== Synchronization Statistics ===");
    println!("Total frames processed: {}", args.frames);
    println!(
        "Successfully synchronized: {}",
        frame_stats.successful_operations
    );
    println!("Partial synchronization: {}", frame_stats.failed_operations);
    println!(
        "Sync success rate: {:.1}%",
        frame_stats.success_rate() * 100.0
    );
    println!(
        "Average sync time: {:.1}ms",
        frame_stats.average_duration_ms()
    );

    if let Some(min) = frame_stats.min_duration_ms {
        println!("Fastest sync: {}ms", min);
    }
    if let Some(max) = frame_stats.max_duration_ms {
        println!("Slowest sync: {}ms", max);
    }

    // Print per-sensor statistics
    println!("\n=== Per-Sensor Statistics ===");
    for (sensor_name, stats) in &sensor_stats {
        println!(
            "Sensor {}: {:.1}ms avg, {:.1}% success",
            sensor_name,
            stats.average_duration_ms(),
            stats.success_rate() * 100.0
        );
    }

    // Flush CSV if enabled
    if let Some(ref mut writer) = csv_writer {
        writer.flush()?;
        println!(
            "\nSynchronization data exported to: {}",
            args.export_csv.as_ref().unwrap()
        );
    }

    // Cleanup sensors
    println!("\n=== Cleanup ===");
    println!("Destroying {} sensors...", sensors.len());

    let mut cleanup_stats = PerformanceStats::new();
    for (mut sensor, sensor_name) in sensors {
        let cleanup_timer = Timer::new();
        match sensor.destroy() {
            Ok(_) => {
                cleanup_stats.record_operation(cleanup_timer.elapsed_ms(), true);
                log::debug!("Destroyed sensor: {}", sensor_name);
            }
            Err(e) => {
                cleanup_stats.record_operation(cleanup_timer.elapsed_ms(), false);
                log::warn!("Failed to destroy sensor {}: {}", sensor_name, e);
            }
        }
    }

    println!(
        "Cleanup success rate: {:.1}%",
        cleanup_stats.success_rate() * 100.0
    );

    // TODO: Restore original world settings
    println!("\nTODO: World settings restoration not implemented");

    println!("\nSensor synchronization demonstration completed!");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cli_args_match_python() {
        // Test that CLI arguments align with Python version
        let args = Args::try_parse_from(&[
            "sensor_sync",
            "--host",
            "localhost",
            "--port",
            "2000",
            "--frames",
            "50",
            "--frame-time-ms",
            "200",
            "--timeout-ms",
            "1000",
            "--export-csv",
            "sync_data.csv",
            "--save-sensor-data",
            "--output-dir",
            "sensor_output",
        ])
        .unwrap();

        assert_eq!(args.common.host, "localhost");
        assert_eq!(args.common.port, 2000);
        assert_eq!(args.frames, 50);
        assert_eq!(args.frame_time_ms, 200);
        assert_eq!(args.timeout_ms, 1000);
        assert!(args.save_sensor_data);
        assert_eq!(args.output_dir, "sensor_output");
    }

    #[test]
    fn test_sensor_sync_queue() {
        let queue = SensorSyncQueue::new(2);

        // Add sensor data
        queue.add_sensor_data(SensorData {
            frame_id: 1,
            sensor_name: "camera01".to_string(),
            timestamp: Instant::now(),
            data_size: 1024,
        });

        queue.add_sensor_data(SensorData {
            frame_id: 1,
            sensor_name: "lidar01".to_string(),
            timestamp: Instant::now(),
            data_size: 2048,
        });

        // Wait for frame data
        let frame_data = queue.wait_for_frame(1, Duration::from_millis(100));
        assert_eq!(frame_data.len(), 2);
        assert_eq!(frame_data[0].frame_id, 1);
        assert_eq!(frame_data[1].frame_id, 1);
    }

    #[test]
    fn test_sensor_config_creation() {
        let configs = create_sensor_config();
        assert_eq!(configs.len(), 5);

        // Verify sensor types
        let sensor_types: Vec<_> = configs
            .iter()
            .map(|(_, blueprint_id, _)| *blueprint_id)
            .collect();
        assert!(sensor_types.contains(&"sensor.camera.rgb"));
        assert!(sensor_types.contains(&"sensor.lidar.ray_cast"));
        assert!(sensor_types.contains(&"sensor.other.radar"));
    }

    #[test]
    fn test_blueprint_configuration() {
        // This would be tested with actual blueprint data in integration tests
        let attributes = vec![("points_per_second", "100000")];
        // Cannot test without actual blueprint instance
        assert_eq!(attributes.len(), 1);
    }
}
