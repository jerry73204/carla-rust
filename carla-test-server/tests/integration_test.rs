// Integration tests for carla-test-server.
//
// These tests demonstrate usage patterns but require a mock
// carla crate to run without an actual CARLA server.

// Note: These tests would normally use the real carla crate,
// but since we're testing the test infrastructure itself,
// we need to be careful about circular dependencies.

#[cfg(test)]
mod tests {
    use carla_test_server::CarlaTestConfig;

    #[test]
    fn test_config_loading() {
        // Test that default configuration works
        let config = CarlaTestConfig::default();
        assert_eq!(config.server.port, 2000);
        assert_eq!(config.server.quality_level, "Low");
        assert!(!config.server.windowed);
    }

    #[test]
    fn test_helpers() {
        use carla_test_server::helpers::*;
        use std::time::Duration;

        // Test wait_for_condition
        let mut counter = 0;
        let result = wait_for_condition(Duration::from_secs(1), Duration::from_millis(10), || {
            counter += 1;
            counter >= 5
        });
        assert!(result.is_ok());

        // Test retry_operation
        let mut attempts = 0;
        let result: Result<i32, &str> = retry_operation(3, Duration::from_millis(10), || {
            attempts += 1;
            if attempts < 3 {
                Err("not yet")
            } else {
                Ok(42)
            }
        });
        assert_eq!(result.unwrap(), 42);
    }

    #[test]
    fn test_port_availability() {
        use carla_test_server::helpers::ensure_port_available;

        // Test with a likely available high port
        let result = ensure_port_available(59999);
        assert!(result.is_ok());
    }

    #[test]
    fn test_artifact_guard() {
        use carla_test_server::helpers::TestArtifactGuard;

        // Create a guard - it should not save anything unless we panic
        let _guard = TestArtifactGuard::new("test_artifact_guard");

        // Test passes, so no artifacts should be saved
    }
}

// Example of how the macro would be used in real tests
// (commented out because it would require the actual carla crate)

/*
use carla_test_server::with_carla_server;

#[with_carla_server]
fn test_basic_connection(client: &carla::client::Client) {
    // This test gets a fresh CARLA server and connected client
    let version = client.server_version().unwrap();
    assert!(!version.is_empty());
    println!("Connected to CARLA version: {}", version);
}

#[with_carla_server]
fn test_implicit_client() {
    // Client is available in scope
    let world = client.world().unwrap();
    let map = world.map().unwrap();
    println!("Current map: {}", map.name());
}

#[with_carla_server]
fn test_with_helpers() {
    use carla_test_server::helpers::*;
    use std::time::Duration;

    let world = client.world().unwrap();

    // Use retry for potentially flaky operations
    let actors = retry_operation(
        3,
        Duration::from_millis(500),
        || world.get_actors()
    ).unwrap();

    println!("Found {} actors", actors.len());
}
*/
