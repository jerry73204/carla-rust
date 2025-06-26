/// Test infrastructure for CARLA Rust client with automatic server management.
///
/// This crate provides a clean attribute macro `#[with_carla_server]` that automatically
/// manages CARLA server lifecycle for integration tests.
///
/// Re-export the procedural macro
pub use carla_test_server_macros::with_carla_server;

// Configuration types
pub mod config;
pub use config::{CarlaTestConfig, LoggingConfig, ServerConfig};

// Server management
pub mod server;
pub use server::CarlaTestServer;

// Helper utilities for tests
pub mod helpers;

// Test context and scenario builders
pub mod test_context;
pub use test_context::{TestContext, TestPhase, TestScenarioBuilder};

use std::panic::UnwindSafe;

/// Main test runner function called by the procedural macro.
///
/// This function:
/// 1. Loads configuration from `carla_server.toml`
/// 2. Starts a fresh CARLA server instance
/// 3. Creates a client connection
/// 4. Runs the test function with panic safety
/// 5. Ensures proper cleanup on completion or failure
///
/// # Arguments
///
/// * `test_fn` - The test function that takes a CARLA client reference
///
/// # Returns
///
/// * `Ok(())` if the test passes
/// * `Err` if server startup or configuration fails
/// * Panics are propagated from the test function
pub fn run_test_with_server<F>(test_fn: F) -> Result<(), Box<dyn std::error::Error>>
where
    F: FnOnce(&carla::client::Client) + UnwindSafe,
{
    // Load configuration
    let config = config::load_config()?;

    // Start CARLA server directly
    let server = CarlaTestServer::start(&config)?;

    // Create client from server
    let client = carla::client::Client::new("localhost", server.port(), None)?;

    // Run test with panic safety
    let result = std::panic::catch_unwind(|| test_fn(&client));

    // Server automatically cleaned up on drop

    match result {
        Ok(_) => Ok(()),
        Err(e) => std::panic::resume_unwind(e),
    }
}

/// Extended test runner that provides test context.
///
/// This function is similar to `run_test_with_server` but provides
/// additional context information to the test function.
///
/// # Arguments
///
/// * `test_name` - Name of the test for logging and debugging
/// * `test_fn` - The test function that takes client and context
///
/// # Returns
///
/// * Same as `run_test_with_server`
pub fn run_test_with_context<F>(
    test_name: impl Into<String>,
    test_fn: F,
) -> Result<(), Box<dyn std::error::Error>>
where
    F: FnOnce(&carla::client::Client, &TestContext) + UnwindSafe,
{
    let test_name = test_name.into();

    // Load configuration
    let config = config::load_config()?;

    // Create test context
    let context = TestContext::new(&test_name, config.clone());
    context.log_milestone("Starting test");

    // Start CARLA server directly
    let server = CarlaTestServer::start(&config)?;
    context.log_milestone("Started server");

    // Create client from server
    let client = carla::client::Client::new("localhost", server.port(), None)?;
    context.log_milestone("Connected to server");

    // Run test with panic safety
    let result = std::panic::catch_unwind(|| test_fn(&client, &context));

    // Server automatically cleaned up on drop
    context.log_milestone("Test completed");

    match result {
        Ok(_) => Ok(()),
        Err(e) => std::panic::resume_unwind(e),
    }
}
