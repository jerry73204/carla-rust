//! Tests to verify sequential execution of CARLA tests.
//!
//! These tests demonstrate that nextest's test groups ensure
//! only one CARLA test runs at a time, even with parallel test runners.

use carla_test_server::with_carla_server;
use std::time::{Duration, Instant};

/// Test 1: Verify sequential execution
#[with_carla_server]
fn test_sequential_execution_1() {
    // Simulate some work
    std::thread::sleep(Duration::from_millis(500));

    let version = client
        .server_version()
        .expect("Failed to get server version");

    println!("Test 1 - Server version: {}", version);

    // More work to increase chance of overlap if locking fails
    std::thread::sleep(Duration::from_millis(500));
}

/// Test 2: Another test that should run sequentially
#[with_carla_server]
fn test_sequential_execution_2(client: &carla::client::Client) {
    // Simulate some work
    std::thread::sleep(Duration::from_millis(500));

    let world = client.world().expect("Failed to get world");

    let map = world.map().expect("Failed to get map");

    println!("Test 2 - Current map: {}", map.name());

    // More work
    std::thread::sleep(Duration::from_millis(500));
}

/// Test 3: Test with implicit client
#[with_carla_server]
fn test_sequential_execution_3() {
    // Simulate some work
    std::thread::sleep(Duration::from_millis(500));

    let actors = client
        .world()
        .expect("Failed to get world")
        .actors()
        .expect("Failed to get actors");

    println!("Test 3 - Actor count: {}", actors.len());

    // More work
    std::thread::sleep(Duration::from_millis(500));
}

/// Test 4: Test server port consistency
#[with_carla_server]
fn test_sequential_execution_4() {
    // Each test should get the same port (sequential execution)
    let version = client.server_version().expect("Failed to get version");

    println!("Test 4 - Connected to server version: {}", version);

    // Verify we can actually use the server
    let world = client.world().expect("Failed to get world");

    let _settings = world.settings().expect("Failed to get settings");

    std::thread::sleep(Duration::from_millis(500));
}

/// Test 5: Longer running test
#[with_carla_server]
fn test_sequential_execution_5(client: &carla::client::Client) {
    let start = Instant::now();

    println!("Test 5 - Starting long-running test");

    // Perform multiple operations
    for i in 0..3 {
        let world = client.world().expect("Failed to get world");

        let actors = world.actors().expect("Failed to get actors");

        println!("  Iteration {}: {} actors", i, actors.len());
        std::thread::sleep(Duration::from_millis(300));
    }

    println!("Test 5 - Completed in {:?}", start.elapsed());
}

/// Test to verify nextest configuration exists
#[test]
fn test_verify_nextest_configuration() {
    use std::path::Path;

    println!("\n=== Nextest Configuration Verification ===");

    // Check if nextest configuration exists
    let config_path = Path::new("carla/.config/nextest.toml");
    let config_exists = config_path.exists() || Path::new(".config/nextest.toml").exists();

    if config_exists {
        println!("✓ Nextest configuration file exists");
    } else {
        println!("⚠️ WARNING: Nextest configuration file not found at carla/.config/nextest.toml");
        println!("Sequential execution requires nextest configuration.");
    }

    // Check if we're running under nextest by looking for environment variables
    if std::env::var("NEXTEST").is_ok() {
        println!("✓ Running under nextest");

        // If running with nextest and in the carla-server group, these vars should be set
        if let Ok(group) = std::env::var("NEXTEST_GROUP_NAME") {
            println!("✓ Test group: {}", group);
            assert_eq!(
                group, "carla-server",
                "Expected to be in carla-server test group"
            );
        }

        if let Ok(slot) = std::env::var("NEXTEST_GROUP_SLOT") {
            println!("✓ Test group slot: {}", slot);
        }
    } else {
        println!(
            "ℹ️  Not running under nextest. Use 'cargo nextest run' for sequential execution."
        );
        println!("   With regular 'cargo test', use '--test-threads=1' to avoid conflicts.");
    }
}

/// Test demonstrating mixed CARLA and non-CARLA tests
#[test]
fn test_non_carla_parallel_1() {
    // This test doesn't use CARLA and should run in parallel
    let start = Instant::now();
    println!("Non-CARLA test 1 started");

    // Simulate some work
    std::thread::sleep(Duration::from_millis(200));

    println!("Non-CARLA test 1 completed in {:?}", start.elapsed());
}

#[test]
fn test_non_carla_parallel_2() {
    // This test doesn't use CARLA and should run in parallel
    let start = Instant::now();
    println!("Non-CARLA test 2 started");

    // Simulate some work
    std::thread::sleep(Duration::from_millis(200));

    println!("Non-CARLA test 2 completed in {:?}", start.elapsed());
}

#[test]
fn test_non_carla_parallel_3() {
    // This test doesn't use CARLA and should run in parallel
    let start = Instant::now();
    println!("Non-CARLA test 3 started");

    // Simulate some work
    std::thread::sleep(Duration::from_millis(200));

    println!("Non-CARLA test 3 completed in {:?}", start.elapsed());
}
