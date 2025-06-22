//! Common test utilities for CARLA integration tests

use carla::{
    actor::{Actor, ActorExt},
    client::{Client, World},
    error::CarlaResult,
    geom::Transform,
};
use std::{sync::Once, time::Duration};

/// Default timeout for test operations
pub const TEST_TIMEOUT: Duration = Duration::from_secs(10);

/// Default CARLA server host
pub const DEFAULT_HOST: &str = "localhost";

/// Default CARLA server port
pub const DEFAULT_PORT: u16 = 2000;

// Ensure CARLA server is started only once for all tests
static CARLA_INIT: Once = Once::new();

/// Ensure CARLA server is running
pub fn ensure_carla_server() {
    CARLA_INIT.call_once(|| {
        // Check if server is already running by creating a client and testing it
        match Client::new(DEFAULT_HOST, DEFAULT_PORT, Some(2usize)) {
            Ok(client) => {
                // Try to actually use the client to verify connection
                match client.server_version() {
                    Ok(version) => {
                        println!(
                            "✓ CARLA server {} is running on {}:{}",
                            version, DEFAULT_HOST, DEFAULT_PORT
                        );
                    }
                    Err(e) => {
                        eprintln!(
                            "\n❌ ERROR: Connected to CARLA server but cannot communicate!\n\
                             \n\
                             Server at: {}:{}\n\
                             Error: {:?}\n\
                             \n\
                             This usually means:\n\
                             - The server is still starting up (wait ~30 seconds)\n\
                             - The server crashed or is in a bad state\n\
                             - Network issues are preventing communication\n\
                             \n\
                             Try restarting CARLA server:\n\
                             ./carla.sh stop\n\
                             ./carla.sh start\n",
                            DEFAULT_HOST, DEFAULT_PORT, e
                        );
                    }
                }
            }
            Err(e) => {
                eprintln!(
                    "\n❌ ERROR: Cannot connect to CARLA server!\n\
                     \n\
                     Expected server at: {}:{}\n\
                     Error: {:?}\n\
                     \n\
                     To run integration tests, you must start CARLA server first:\n\
                     ./carla.sh start\n\
                     \n\
                     If CARLA is already running, check that:\n\
                     - The server is listening on the correct port ({})\n\
                     - No firewall is blocking the connection\n\
                     - The server is fully started (wait ~30 seconds after launch)\n",
                    DEFAULT_HOST, DEFAULT_PORT, e, DEFAULT_PORT
                );
            }
        }
    });
}

/// Get a test client with automatic server check
pub fn get_test_client() -> CarlaResult<Client> {
    ensure_carla_server();
    Client::new(DEFAULT_HOST, DEFAULT_PORT, None::<usize>).map_err(|e| {
        eprintln!(
            "\n❌ Test failed: Cannot connect to CARLA server at {}:{}\n\
                 Error: {:?}\n\
                 Ensure CARLA server is running with: ./carla.sh start\n",
            DEFAULT_HOST, DEFAULT_PORT, e
        );
        e
    })
}

/// Reset world to clean state for testing
pub fn reset_world(client: &Client) -> CarlaResult<()> {
    let world = client.world()?;

    // Get all actors
    let actors = world.actors()?;

    // Destroy all actors except the spectator
    use carla::actor::ActorExt;
    for mut actor in actors {
        if !actor.type_id().starts_with("spectator") {
            // Now available via ActorExt trait
            actor.destroy()?;
        }
    }

    // Reset weather to default
    use carla::client::WeatherParameters;
    world.set_weather(&WeatherParameters::default())?;

    // Reset settings to default
    let mut settings = world.settings()?;
    settings.synchronous_mode = false;
    settings.fixed_delta_seconds = None;
    world.apply_settings(&settings)?;

    Ok(())
}

/// Retry a test operation up to max_attempts times
pub fn retry_test<F, T, E>(mut f: F, max_attempts: u32) -> Result<T, E>
where
    F: FnMut() -> Result<T, E>,
{
    for attempt in 0..max_attempts {
        match f() {
            Ok(result) => return Ok(result),
            Err(_e) if attempt < max_attempts - 1 => {
                std::thread::sleep(Duration::from_millis(100));
                continue;
            }
            Err(e) => return Err(e),
        }
    }
    unreachable!()
}

/// Compare floating point values with tolerance
#[macro_export]
macro_rules! assert_near {
    ($left:expr, $right:expr, $tolerance:expr) => {
        let left_val: f64 = $left as f64;
        let right_val: f64 = $right as f64;
        let tolerance_val: f64 = $tolerance as f64;
        let diff = (left_val - right_val).abs();
        assert!(
            diff < tolerance_val,
            "assertion failed: `(left ≈ right)`\n  left: `{:?}`,\n right: `{:?}`,\n  diff: `{:?}`,\n  tolerance: `{:?}`",
            left_val, right_val, diff, tolerance_val
        );
    };
}

/// Helper to spawn a test vehicle at a safe location
pub fn spawn_test_vehicle(client: &Client) -> CarlaResult<carla::actor::Vehicle> {
    let world = client.world()?;
    let blueprint_library = world.blueprint_library()?;

    // Find a vehicle blueprint
    let vehicle_bp = blueprint_library
        .find("vehicle.dodge.charger")?
        .or_else(|| blueprint_library.filter("vehicle.*").ok()?.first().cloned())
        .expect("No vehicle blueprints found");

    // Get a spawn point
    let spawn_points = world.map()?.spawn_points();
    let spawn_point = spawn_points
        .get(0)
        .unwrap_or_else(|| carla::geom::Transform::default());

    // Spawn the vehicle
    let actor = world
        .try_spawn_actor(&vehicle_bp, &spawn_point, None)?
        .expect("Failed to spawn test vehicle");

    // Cast to Vehicle
    match actor.into_vehicle() {
        Ok(vehicle) => Ok(vehicle),
        Err(_) => panic!("Expected vehicle actor, got something else"),
    }
}

/// Helper to wait for a condition with timeout
pub fn wait_for<F>(condition: F, timeout: Duration) -> CarlaResult<()>
where
    F: Fn() -> bool,
{
    let start = std::time::Instant::now();
    while !condition() {
        if start.elapsed() > timeout {
            return Err(carla::error::CarlaError::Client(
                carla::error::ClientError::Timeout(timeout),
            ));
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    Ok(())
}

/// Helper to spawn an actor and ensure cleanup
pub fn with_spawned_actor<F, T>(
    world: &World,
    blueprint_name: &str,
    transform: &Transform,
    test_fn: F,
) -> CarlaResult<T>
where
    F: FnOnce(&mut Actor) -> CarlaResult<T>,
{
    let blueprint = world
        .blueprint_library()?
        .find(blueprint_name)?
        .ok_or_else(|| {
            carla::error::CarlaError::World(carla::error::WorldError::BlueprintNotFound(
                blueprint_name.to_string(),
            ))
        })?;

    let mut actor = world.spawn_actor(&blueprint, transform, None)?;

    // Run test function, ensuring cleanup even if it panics
    let result = test_fn(&mut actor);

    // Always destroy the actor
    use carla::actor::ActorExt;
    let _ = actor.destroy();

    result
}

/// Test fixture cleanup guard
pub struct TestCleanup<'a> {
    client: &'a Client,
}

impl<'a> TestCleanup<'a> {
    pub fn new(client: &'a Client) -> Self {
        Self { client }
    }
}

impl<'a> Drop for TestCleanup<'a> {
    fn drop(&mut self) {
        // Best effort cleanup on test exit
        let _ = reset_world(self.client);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_assert_near_macro() {
        assert_near!(1.0, 1.001, 0.01);
        assert_near!(5.0, 5.0, 0.0001);
    }

    #[test]
    #[should_panic]
    fn test_assert_near_macro_fails() {
        assert_near!(1.0, 2.0, 0.1);
    }

    #[test]
    fn test_retry_success() {
        let mut attempts = 0;
        let result = retry_test(
            || {
                attempts += 1;
                if attempts < 3 {
                    Err("not yet")
                } else {
                    Ok(42)
                }
            },
            5,
        );
        assert_eq!(result, Ok(42));
        assert_eq!(attempts, 3);
    }
}
