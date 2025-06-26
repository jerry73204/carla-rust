//! Helper functions for common test patterns with CARLA server.

use crate::config::CarlaTestConfig;
use std::time::Duration;

/// Waits for a condition to become true, with timeout.
///
/// This is useful for waiting for asynchronous CARLA operations to complete.
///
/// # Arguments
///
/// * `timeout` - Maximum time to wait
/// * `interval` - How often to check the condition
/// * `condition` - Closure that returns true when condition is met
///
/// # Returns
///
/// * `Ok(())` if condition becomes true within timeout
/// * `Err` if timeout is exceeded
///
/// # Example
///
/// ```rust
/// use carla_test_server::helpers::wait_for_condition;
/// use std::time::Duration;
///
/// wait_for_condition(Duration::from_secs(5), Duration::from_millis(100), || {
///     actor.is_alive()
/// })?;
/// ```
pub fn wait_for_condition<F>(
    timeout: Duration,
    interval: Duration,
    mut condition: F,
) -> Result<(), Box<dyn std::error::Error>>
where
    F: FnMut() -> bool,
{
    let start = std::time::Instant::now();

    while start.elapsed() < timeout {
        if condition() {
            return Ok(());
        }
        std::thread::sleep(interval);
    }

    Err("Timeout waiting for condition".into())
}

/// Retry an operation that might fail temporarily.
///
/// Useful for operations that might fail due to timing issues or
/// temporary resource unavailability.
///
/// # Arguments
///
/// * `max_attempts` - Maximum number of attempts
/// * `delay` - Delay between attempts
/// * `operation` - The operation to retry
///
/// # Returns
///
/// * The result of the first successful operation
/// * The last error if all attempts fail
///
/// # Example
///
/// ```rust
/// use carla_test_server::helpers::retry_operation;
/// use std::time::Duration;
///
/// let actor = retry_operation(3, Duration::from_millis(500), || {
///     world.spawn_actor(&blueprint, &transform)
/// })?;
/// ```
pub fn retry_operation<F, T, E>(
    max_attempts: u32,
    delay: Duration,
    mut operation: F,
) -> Result<T, E>
where
    F: FnMut() -> Result<T, E>,
{
    let mut last_error = None;

    for attempt in 1..=max_attempts {
        match operation() {
            Ok(result) => return Ok(result),
            Err(e) => {
                log::debug!("Attempt {} failed, retrying...", attempt);
                last_error = Some(e);
                if attempt < max_attempts {
                    std::thread::sleep(delay);
                }
            }
        }
    }

    Err(last_error.expect("Should have at least one error"))
}

/// Configuration for test debugging.
#[derive(Debug, Clone)]
pub struct TestDebugConfig {
    /// Whether to save server logs on test failure
    pub save_logs_on_failure: bool,
    /// Whether to print server output to console
    pub print_server_output: bool,
    /// Whether to take screenshots on failure
    pub screenshot_on_failure: bool,
}

impl Default for TestDebugConfig {
    fn default() -> Self {
        Self {
            save_logs_on_failure: true,
            print_server_output: false,
            screenshot_on_failure: false,
        }
    }
}

/// Gets the default test timeout from environment or config.
///
/// Checks the `CARLA_TEST_TIMEOUT` environment variable first,
/// then falls back to the configured timeout.
pub fn get_test_timeout(config: &CarlaTestConfig) -> Duration {
    if let Ok(timeout_str) = std::env::var("CARLA_TEST_TIMEOUT") {
        if let Ok(seconds) = timeout_str.parse::<u64>() {
            return Duration::from_secs(seconds);
        }
    }

    Duration::from_secs(config.coordination.timeout_seconds)
}

/// Ensures a test port is available before starting.
///
/// This is useful when running tests that might conflict with
/// other services on the same port.
pub fn ensure_port_available(port: u16) -> Result<(), Box<dyn std::error::Error>> {
    use std::net::TcpListener;

    match TcpListener::bind(("127.0.0.1", port)) {
        Ok(_) => Ok(()), // Port is available
        Err(e) => Err(format!("Port {} is not available: {}", port, e).into()),
    }
}

/// Gets the path to save test artifacts (logs, screenshots, etc).
///
/// Creates the directory if it doesn't exist.
pub fn get_test_artifacts_dir(
    test_name: &str,
) -> Result<std::path::PathBuf, Box<dyn std::error::Error>> {
    let base_dir = std::env::var("CARLA_TEST_ARTIFACTS")
        .unwrap_or_else(|_| "/tmp/carla_test_artifacts".to_string());

    let dir = std::path::Path::new(&base_dir).join(test_name);
    std::fs::create_dir_all(&dir)?;

    Ok(dir)
}

/// A guard that captures test artifacts on drop if the test panics.
pub struct TestArtifactGuard {
    test_name: String,
    config: TestDebugConfig,
}

impl TestArtifactGuard {
    /// Creates a new test artifact guard.
    pub fn new(test_name: impl Into<String>) -> Self {
        Self {
            test_name: test_name.into(),
            config: TestDebugConfig::default(),
        }
    }

    /// Creates a guard with custom debug configuration.
    pub fn with_config(test_name: impl Into<String>, config: TestDebugConfig) -> Self {
        Self {
            test_name: test_name.into(),
            config,
        }
    }
}

impl Drop for TestArtifactGuard {
    fn drop(&mut self) {
        // Check if we're panicking
        if std::thread::panicking() && self.config.save_logs_on_failure {
            // Try to save artifacts
            if let Ok(dir) = get_test_artifacts_dir(&self.test_name) {
                log::error!(
                    "Test {} failed, saving artifacts to {:?}",
                    self.test_name,
                    dir
                );

                // Copy server logs if they exist
                if let Ok(entries) = std::fs::read_dir("/tmp/carla_test_logs") {
                    for entry in entries.flatten() {
                        let dest = dir.join(entry.file_name());
                        let _ = std::fs::copy(entry.path(), dest);
                    }
                }
            }
        }
    }
}
