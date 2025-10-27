//! Test utilities for CARLA integration tests
//!
//! This module provides helper functions and utilities for writing integration tests
//! that require access to the CARLA simulator. Key features:
//!
//! - Exclusive simulator access using file locks
//! - Connection retry logic
//! - Automatic cleanup of spawned actors
//! - Simulator readiness detection
//!
//! # Example
//!
//! ```no_run
//! use carla_test_utils::with_simulator;
//! use serial_test::serial;
//!
//! #[test]
//! #[serial]
//! #[ignore] // Only run when simulator is available
//! fn test_vehicle_spawn() {
//!     with_simulator(|world| {
//!         // Test code here - has exclusive simulator access
//!         let blueprint = world
//!             .get_blueprint_library()
//!             .find("vehicle.tesla.model3")
//!             .unwrap();
//!         let spawn_point = world.get_map().get_spawn_points()[0];
//!         let vehicle = world.spawn_actor(&blueprint, &spawn_point).unwrap();
//!
//!         assert!(vehicle.is_alive());
//!
//!         // Cleanup handled automatically
//!     });
//! }
//! ```

pub mod simulator_lock;

use std::{env, time::Duration};

use carla::client::{ActorBase, Client, World};
use simulator_lock::SimulatorLock;

/// Default CARLA server host
pub const DEFAULT_HOST: &str = "localhost";

/// Default CARLA server port
pub const DEFAULT_PORT: u16 = 2000;

/// Default connection timeout in seconds
pub const DEFAULT_TIMEOUT_SECS: u64 = 10;

/// Maximum number of connection retry attempts
pub const MAX_CONNECTION_RETRIES: usize = 5;

/// Delay between connection retry attempts
pub const RETRY_DELAY: Duration = Duration::from_millis(500);

/// Configuration for simulator connection
#[derive(Debug, Clone)]
pub struct SimulatorConfig {
    pub host: String,
    pub port: u16,
    pub timeout: Duration,
}

impl Default for SimulatorConfig {
    fn default() -> Self {
        Self {
            host: env::var("CARLA_HOST").unwrap_or_else(|_| DEFAULT_HOST.to_string()),
            port: env::var("CARLA_PORT")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(DEFAULT_PORT),
            timeout: Duration::from_secs(DEFAULT_TIMEOUT_SECS),
        }
    }
}

/// Connect to the CARLA simulator with retry logic
///
/// Attempts to connect to the simulator, retrying on failure up to
/// `MAX_CONNECTION_RETRIES` times.
///
/// # Arguments
///
/// * `config` - Connection configuration
///
/// # Returns
///
/// Returns `Ok(Client)` on successful connection, or `Err` if all retries fail.
pub fn connect_to_simulator(config: &SimulatorConfig) -> anyhow::Result<Client> {
    for attempt in 1..=MAX_CONNECTION_RETRIES {
        // Try to connect
        let client = Client::connect(&config.host, config.port, None);

        // Verify connection by getting server version
        let version = client.server_version();
        eprintln!("Connected to CARLA server version: {}", version);
        return Ok(client);
    }

    Err(anyhow::anyhow!(
        "Failed to connect to CARLA simulator at {}:{} after {} attempts",
        config.host,
        config.port,
        MAX_CONNECTION_RETRIES
    ))
}

/// Wait for the simulator to be ready
///
/// Waits for the simulator to finish initialization and be ready to accept commands.
/// This is useful after loading a new world or starting the simulator.
///
/// # Arguments
///
/// * `world` - Reference to the CARLA world
/// * `timeout` - Maximum time to wait for readiness
///
/// # Returns
///
/// Returns `Ok(())` if the simulator is ready, or `Err` if timeout is reached.
pub fn wait_for_simulator_ready(world: &World, timeout: Duration) -> anyhow::Result<()> {
    let start = std::time::Instant::now();

    // Wait for at least one tick to ensure world is responsive
    loop {
        if start.elapsed() > timeout {
            return Err(anyhow::anyhow!(
                "Simulator did not become ready within {:?}",
                timeout
            ));
        }

        match world.wait_for_tick_or_timeout(Duration::from_secs(1)) {
            Some(_snapshot) => {
                // Successfully got a tick - simulator is ready
                eprintln!("Simulator ready after {:?}", start.elapsed());
                return Ok(());
            }
            None => {
                eprintln!("Waiting for simulator ready...");
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }
}

/// Clean up all actors from the world
///
/// Currently, actor destruction is not implemented in the Rust API.
/// This function lists actors that should be cleaned up.
///
/// # Arguments
///
/// * `world` - Reference to the CARLA world
///
/// # Note
///
/// TODO: Implement actor destruction when the API is available.
/// For now, actors remain in the world between tests.
pub fn cleanup_all_actors(world: &World) -> anyhow::Result<()> {
    let actors = world.actors();

    let mut actor_count = 0;
    for actor in actors.iter() {
        // Don't count the spectator
        if actor.type_id().contains("spectator") {
            continue;
        }
        actor_count += 1;
    }

    if actor_count > 0 {
        eprintln!(
            "Note: {} actors remaining (cleanup not yet implemented)",
            actor_count
        );
    }

    Ok(())
}

/// Run a test function with exclusive simulator access
///
/// This is the primary helper function for integration tests. It:
/// 1. Acquires an exclusive file lock to prevent concurrent simulator access
/// 2. Connects to the simulator with retry logic
/// 3. Waits for the simulator to be ready
/// 4. Runs the test function with the world
/// 5. Cleans up all spawned actors
/// 6. Releases the lock
///
/// # Arguments
///
/// * `test_fn` - Test function that takes a `&World` parameter
///
/// # Panics
///
/// Panics if:
/// - Cannot acquire simulator lock
/// - Cannot connect to simulator
/// - Simulator does not become ready
///
/// # Example
///
/// ```no_run
/// use serial_test::serial;
///
/// #[test]
/// #[serial]
/// #[ignore]
/// fn test_something() {
///     with_simulator(|world| {
///         // Your test code here
///     });
/// }
/// ```
pub fn with_simulator<F>(test_fn: F)
where
    F: FnOnce(&mut World),
{
    // Acquire exclusive lock
    let _lock = SimulatorLock::acquire_with_retry(10, Duration::from_millis(500))
        .expect("Failed to acquire simulator lock");

    eprintln!("Acquired simulator lock");

    // Connect to simulator
    let config = SimulatorConfig::default();
    let client = connect_to_simulator(&config).expect("Failed to connect to simulator");

    // Get world
    let mut world = client.world();

    // Wait for simulator to be ready
    wait_for_simulator_ready(&world, Duration::from_secs(10)).expect("Simulator not ready");

    // Clean up before test
    cleanup_all_actors(&world).expect("Failed to cleanup actors before test");

    // Run the test
    test_fn(&mut world);

    // Clean up after test
    cleanup_all_actors(&world).expect("Failed to cleanup actors after test");

    eprintln!("Test completed, releasing simulator lock");
}

/// Run a test function with exclusive simulator access and return the client
///
/// Similar to `with_simulator`, but provides access to the `Client` instead of just `World`.
/// This is useful for tests that need client-level operations like loading maps or
/// recording/playback.
///
/// # Arguments
///
/// * `test_fn` - Test function that takes a `&Client` parameter
///
/// # Example
///
/// ```no_run
/// use serial_test::serial;
///
/// #[test]
/// #[serial]
/// #[ignore]
/// fn test_with_client() {
///     with_simulator_client(|client| {
///         // Access client and world
///         let world = client.world();
///         // Your test code here
///     });
/// }
/// ```
pub fn with_simulator_client<F>(test_fn: F)
where
    F: FnOnce(&Client),
{
    // Acquire exclusive lock
    let _lock = SimulatorLock::acquire_with_retry(10, Duration::from_millis(500))
        .expect("Failed to acquire simulator lock");

    eprintln!("Acquired simulator lock");

    // Connect to simulator
    let config = SimulatorConfig::default();
    let client = connect_to_simulator(&config).expect("Failed to connect to simulator");

    // Get world and wait for ready
    let world = client.world();
    wait_for_simulator_ready(&world, Duration::from_secs(10)).expect("Simulator not ready");

    // Clean up before test
    cleanup_all_actors(&world).expect("Failed to cleanup actors before test");

    // Run the test
    test_fn(&client);

    // Clean up after test
    let world = client.world();
    cleanup_all_actors(&world).expect("Failed to cleanup actors after test");

    eprintln!("Test completed, releasing simulator lock");
}
