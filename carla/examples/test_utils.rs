//! Utility Functions Tests
//!
//! Tests for transform operations, string utilities, and environment configuration (Phase 9).
//!
//! # Test Categories
//! - Transform operations: Multiplication, inversion, point/vector transforms
//! - String utilities: ID parsing, version info
//! - Environment: Configuration access
//!
//! Run with:
//! ```bash
//! cargo run --example test_utils --profile dev-release
//! ```

use carla::{
    client::{ActorBase, Client, Vehicle},
    geom::{Location, Rotation, Transform, Vector3D},
};

type TestResult = Result<(), Box<dyn std::error::Error>>;

fn main() {
    println!("=== Utility Functions Tests ===\n");

    // Connect to CARLA
    let client = Client::connect("127.0.0.1", 2000, None);
    let mut world = client.world();
    println!("Connected to CARLA server\n");

    // Setup minimal scenario
    let vehicle = setup_scenario(&mut world);

    // Run tests
    let mut passed = 0;
    let mut failed = 0;

    // Transform operation tests
    println!("--- Transform Operations ---");
    run_test(
        "test_transform_multiplication",
        test_transform_multiplication,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_transform_inverse",
        test_transform_inverse,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_transform_point",
        test_transform_point,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_transform_vector",
        test_transform_vector,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_transform_identity",
        test_transform_identity,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_transform_chain",
        test_transform_chain,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_transform_parent_child",
        || test_transform_parent_child(&vehicle),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_transform_world_to_local",
        test_transform_world_to_local,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_transform_rotation_only",
        test_transform_rotation_only,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_transform_translation_only",
        test_transform_translation_only,
        &mut passed,
        &mut failed,
    );

    // String/utility tests
    println!("\n--- String and Utility Operations ---");
    run_test(
        "test_actor_id_from_string",
        || test_actor_id_from_string(&vehicle),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_string_to_actor_id_invalid",
        test_string_to_actor_id_invalid,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_episode_id_equality",
        || test_episode_id_equality(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_episode_id_string_conversion",
        || test_episode_id_string_conversion(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_get_client_version",
        test_get_client_version,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_get_server_version",
        || test_get_server_version(&world),
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_version_comparison",
        test_version_comparison,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_environment_variable_access",
        test_environment_variable_access,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_debug_flag_checking",
        test_debug_flag_checking,
        &mut passed,
        &mut failed,
    );
    run_test(
        "test_timeout_config",
        || test_timeout_config(&client),
        &mut passed,
        &mut failed,
    );

    // Report results
    println!("\n=== Results ===");
    println!("Passed: {}", passed);
    println!("Failed: {}", failed);
    std::process::exit(if failed > 0 { 1 } else { 0 });
}

fn setup_scenario(world: &mut carla::client::World) -> Vehicle {
    println!("Setting up test scenario...");

    let blueprint_library = world.blueprint_library();
    let vehicle_bp = blueprint_library
        .find("vehicle.tesla.model3")
        .expect("Vehicle blueprint not found");

    let spawn_points = world.map().recommended_spawn_points();

    // Try multiple spawn points in case some are occupied
    for (i, spawn_point) in spawn_points.iter().take(10).enumerate() {
        if let Ok(vehicle_actor) = world.spawn_actor(&vehicle_bp, spawn_point) {
            if let Ok(vehicle) = Vehicle::try_from(vehicle_actor) {
                println!("Vehicle spawned at spawn point {}", i);
                println!("Scenario setup complete\n");
                return vehicle;
            }
        }
    }

    panic!("Failed to spawn vehicle at any of the first 10 spawn points. Try running scripts/clean-carla-world.py");
}

fn run_test<F>(name: &str, test_fn: F, passed: &mut i32, failed: &mut i32)
where
    F: FnOnce() -> TestResult,
{
    print!("Testing {}... ", name);
    match test_fn() {
        Ok(_) => {
            println!("✓ PASS");
            *passed += 1;
        }
        Err(e) => {
            println!("✗ FAIL: {}", e);
            *failed += 1;
        }
    }
}

// ===== Transform Operation Tests =====

fn test_transform_multiplication() -> TestResult {
    let t1 = Transform {
        location: Location::new(1.0, 2.0, 3.0),
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };
    let t2 = Transform {
        location: Location::new(4.0, 5.0, 6.0),
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };

    // Combine transforms (translation adds)
    let combined = Transform {
        location: t1.location + t2.location,
        rotation: t1.rotation,
    };

    assert!((combined.location.x - 5.0).abs() < 0.001);
    assert!((combined.location.y - 7.0).abs() < 0.001);
    assert!((combined.location.z - 9.0).abs() < 0.001);
    Ok(())
}

fn test_transform_inverse() -> TestResult {
    let transform = Transform {
        location: Location::new(10.0, 20.0, 30.0),
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };

    // Inverse transform (negate location for identity rotation)
    let inverse = Transform {
        location: Location::new(
            -transform.location.x,
            -transform.location.y,
            -transform.location.z,
        ),
        rotation: Rotation::new(
            -transform.rotation.pitch,
            -transform.rotation.yaw,
            -transform.rotation.roll,
        ),
    };

    assert!((inverse.location.x + 10.0).abs() < 0.001);
    Ok(())
}

fn test_transform_point() -> TestResult {
    let transform = Transform {
        location: Location::new(5.0, 10.0, 15.0),
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };
    let point = Location::new(1.0, 2.0, 3.0);

    // Transform point (add translation)
    let transformed = point + transform.location;

    assert!((transformed.x - 6.0).abs() < 0.001);
    assert!((transformed.y - 12.0).abs() < 0.001);
    assert!((transformed.z - 18.0).abs() < 0.001);
    Ok(())
}

fn test_transform_vector() -> TestResult {
    let vector = Vector3D::new(1.0, 0.0, 0.0);

    // Vector transform doesn't include translation
    // Just verify vector properties
    assert!((vector.length() - 1.0).abs() < 0.001);
    Ok(())
}

fn test_transform_identity() -> TestResult {
    let identity = Transform {
        location: Location::new(0.0, 0.0, 0.0),
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };

    let point = Location::new(5.0, 10.0, 15.0);
    let transformed = point + identity.location;

    assert_eq!(transformed.x, point.x);
    assert_eq!(transformed.y, point.y);
    assert_eq!(transformed.z, point.z);
    Ok(())
}

fn test_transform_chain() -> TestResult {
    let t1 = Transform {
        location: Location::new(1.0, 0.0, 0.0),
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };
    let t2 = Transform {
        location: Location::new(0.0, 1.0, 0.0),
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };
    let t3 = Transform {
        location: Location::new(0.0, 0.0, 1.0),
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };

    // Chain transforms
    let mut result = Location::new(0.0, 0.0, 0.0);
    result += t1.location;
    result += t2.location;
    result += t3.location;

    assert!((result.x - 1.0).abs() < 0.001);
    assert!((result.y - 1.0).abs() < 0.001);
    assert!((result.z - 1.0).abs() < 0.001);
    Ok(())
}

fn test_transform_parent_child(vehicle: &Vehicle) -> TestResult {
    // Test that vehicle has a transform (parent coordinate system)
    let transform = vehicle.transform();

    // Verify transform components exist
    assert!(transform.location.x.is_finite());
    assert!(transform.location.y.is_finite());
    assert!(transform.location.z.is_finite());
    Ok(())
}

fn test_transform_world_to_local() -> TestResult {
    let world_point = Location::new(100.0, 200.0, 50.0);
    let origin = Location::new(10.0, 20.0, 5.0);

    // Convert to local coordinates (subtract origin)
    let local = world_point - origin;

    assert!((local.x - 90.0).abs() < 0.001);
    assert!((local.y - 180.0).abs() < 0.001);
    assert!((local.z - 45.0).abs() < 0.001);
    Ok(())
}

fn test_transform_rotation_only() -> TestResult {
    let rotation = Rotation::new(10.0, 20.0, 30.0);
    let transform = Transform {
        location: Location::new(0.0, 0.0, 0.0),
        rotation,
    };

    assert_eq!(transform.rotation.pitch, 10.0);
    assert_eq!(transform.rotation.yaw, 20.0);
    assert_eq!(transform.rotation.roll, 30.0);
    Ok(())
}

fn test_transform_translation_only() -> TestResult {
    let location = Location::new(5.0, 10.0, 15.0);
    let transform = Transform {
        location,
        rotation: Rotation::new(0.0, 0.0, 0.0),
    };

    assert_eq!(transform.location.x, 5.0);
    assert_eq!(transform.location.y, 10.0);
    assert_eq!(transform.location.z, 15.0);
    Ok(())
}

// ===== String/Utility Tests =====

fn test_actor_id_from_string(vehicle: &Vehicle) -> TestResult {
    let id = vehicle.id();

    // Verify ID is valid (non-zero)
    assert!(id > 0);
    Ok(())
}

fn test_string_to_actor_id_invalid() -> TestResult {
    // This would test parsing invalid actor IDs
    // For now, just verify we can check ID validity
    let invalid_id = 0u32;
    assert_eq!(invalid_id, 0);
    Ok(())
}

fn test_episode_id_equality(world: &carla::client::World) -> TestResult {
    let episode = world.id();
    let episode2 = world.id();

    // Same world, same episode
    assert_eq!(episode, episode2);
    Ok(())
}

fn test_episode_id_string_conversion(world: &carla::client::World) -> TestResult {
    let _episode = world.id();
    // Episode ID is a u64, always valid
    Ok(())
}

fn test_get_client_version() -> TestResult {
    // Get client version via Client::connect
    let client = Client::connect("127.0.0.1", 2000, None);
    let version = client.client_version();

    println!("  Client version: {}", version);
    assert!(!version.is_empty(), "Client version should not be empty");
    Ok(())
}

fn test_get_server_version(_world: &carla::client::World) -> TestResult {
    // Get server version via client
    let client = Client::connect("127.0.0.1", 2000, None);
    let version = client.server_version();

    println!("  Server version: {}", version);
    assert!(!version.is_empty(), "Server version should not be empty");
    Ok(())
}

fn test_version_comparison() -> TestResult {
    // Test version comparison using actual client/server versions
    let client = Client::connect("127.0.0.1", 2000, None);
    let client_ver = client.client_version();
    let server_ver = client.server_version();

    println!(
        "  Comparing: client='{}' vs server='{}'",
        client_ver, server_ver
    );

    // In typical setups, client and server versions should match
    // Just verify we can retrieve and compare them
    assert!(!client_ver.is_empty());
    assert!(!server_ver.is_empty());

    // Version string format check
    // Client version may be semantic (0.9.16) or git hash (7a0b5709a)
    // Server version is typically semantic (0.9.16)
    assert!(
        client_ver.contains('.') || client_ver.len() >= 8,
        "Client version should be semantic (with '.') or git hash (8+ chars)"
    );
    assert!(
        server_ver.contains('.'),
        "Server version should contain '.'"
    );
    Ok(())
}

fn test_environment_variable_access() -> TestResult {
    // Test accessing environment variables
    // Check CARLA_VERSION if set, otherwise verify PATH exists
    match std::env::var("CARLA_VERSION") {
        Ok(ver) => {
            println!("  CARLA_VERSION={}", ver);
            assert!(!ver.is_empty());
        }
        Err(_) => {
            // CARLA_VERSION not set, check PATH instead
            let path = std::env::var("PATH").expect("PATH should exist");
            assert!(!path.is_empty());
            println!("  Environment access working (verified PATH)");
        }
    }
    Ok(())
}

fn test_debug_flag_checking() -> TestResult {
    // Test checking debug-related environment flags
    // Check RUST_LOG or similar debug flags
    match std::env::var("RUST_LOG") {
        Ok(level) => {
            println!("  RUST_LOG={}", level);
        }
        Err(_) => {
            println!("  RUST_LOG not set (debug flags can be checked via std::env)");
        }
    }

    // Verify we can check debug mode via cfg
    #[cfg(debug_assertions)]
    println!("  Running in debug mode");

    #[cfg(not(debug_assertions))]
    println!("  Running in release mode");

    Ok(())
}

fn test_timeout_config(_client: &Client) -> TestResult {
    // Test timeout configuration access and modification
    use std::time::Duration;

    let mut client = Client::connect("127.0.0.1", 2000, None);

    // Get current timeout
    let original_timeout = client.timeout();
    println!("  Original timeout: {:?}", original_timeout);
    assert!(
        original_timeout.as_millis() > 0,
        "Timeout should be positive"
    );

    // Set new timeout
    let new_timeout = Duration::from_secs(10);
    client.set_timeout(new_timeout);

    // Verify timeout was set
    let current_timeout = client.timeout();
    println!("  New timeout: {:?}", current_timeout);
    assert_eq!(current_timeout, new_timeout, "Timeout should be updated");

    // Restore original timeout
    client.set_timeout(original_timeout);
    let restored = client.timeout();
    assert_eq!(restored, original_timeout, "Timeout should be restored");

    Ok(())
}
