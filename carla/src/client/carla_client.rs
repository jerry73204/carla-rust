// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::World;
use crate::{
    rpc::{Command, CommandResponse, OpendriveGenerationParameters},
    traffic_manager::{constants::Networking::TM_DEFAULT_PORT, TrafficManager},
};
use autocxx::prelude::*;
use carla_sys::carla_rust::client::{FfiClient, FfiCommandBatch};
use cxx::{let_cxx_string, UniquePtr};
use derivative::Derivative;
use static_assertions::assert_impl_all;
use std::time::Duration;

/// The client maintains the connection to the CARLA simulator server.
///
/// The [`Client`] is your entry point for interacting with a CARLA simulation.
/// It manages the network connection to the server and provides methods to:
/// - Access the simulation [`World`]
/// - Load and manage maps
/// - Create [`TrafficManager`] instances
/// - Check version compatibility
///
/// Corresponds to [`carla.Client`] in the Python API.
#[cfg_attr(carla_version_0916, doc = "")]
#[cfg_attr(
    carla_version_0916,
    doc = " [`carla.Client`]: https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Client"
)]
#[cfg_attr(carla_version_0915, doc = "")]
#[cfg_attr(
    carla_version_0915,
    doc = " [`carla.Client`]: https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Client"
)]
#[cfg_attr(carla_version_0914, doc = "")]
#[cfg_attr(
    carla_version_0914,
    doc = " [`carla.Client`]: https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Client"
)]
///
/// # Thread Safety
///
/// [`Client`] implements [`Send`] and [`Sync`], but the underlying CARLA server
/// connection is not thread-safe. Coordinate access across threads carefully.
///
/// # Examples
///
/// ```no_run
/// use carla::client::Client;
///
/// // Connect to a local CARLA server
/// let client = Client::connect("localhost", 2000, None);
///
/// // Check version compatibility
/// println!("Client: {}", client.client_version());
/// println!("Server: {}", client.server_version());
///
/// // Access the simulation world
/// let world = client.world();
/// ```
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Client {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiClient>,
}

impl Client {
    /// Connects to a CARLA simulator server.
    ///
    /// # Arguments
    ///
    /// * `host` - Server hostname or IP address (e.g., "localhost", "192.168.1.100")
    /// * `port` - Server port (default is 2000)
    /// * `worker_threads` - Number of worker threads for async operations (0 = auto)
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// // Connect to local server with default settings
    /// let client = Client::connect("localhost", 2000, None);
    ///
    /// // Connect with 4 worker threads
    /// let client = Client::connect("localhost", 2000, Some(4));
    /// ```
    pub fn connect<W>(host: &str, port: u16, worker_threads: W) -> Self
    where
        W: Into<Option<usize>>,
    {
        let_cxx_string!(host_cxx = host);
        let worker_threads = worker_threads.into().unwrap_or(0);

        Self {
            inner: FfiClient::new(&host_cxx, port, worker_threads).within_unique_ptr(),
        }
    }

    /// Returns the client library version string.
    ///
    /// Use this with [`server_version()`](Self::server_version) to check
    /// version compatibility between client and server.
    pub fn client_version(&self) -> String {
        self.inner.GetClientVersion().to_string()
    }

    /// Returns the CARLA server version string.
    ///
    /// The client and server versions should match to ensure compatibility.
    pub fn server_version(&self) -> String {
        self.inner.GetServerVersion().to_string()
    }

    /// Returns the current network timeout for server operations.
    ///
    /// Operations that exceed this timeout will fail. Default is typically 5 seconds.
    pub fn timeout(&mut self) -> Duration {
        let millis = self.inner.pin_mut().GetTimeout();
        Duration::from_millis(millis as u64)
    }

    /// Sets the network timeout for server operations.
    ///
    /// # Arguments
    ///
    /// * `timeout` - Maximum time to wait for server responses
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    /// use std::time::Duration;
    ///
    /// let mut client = Client::default();
    /// client.set_timeout(Duration::from_secs(10));
    /// ```
    pub fn set_timeout(&mut self, timeout: Duration) {
        let millis = timeout.as_millis() as usize;
        self.inner.pin_mut().SetTimeout(millis);
    }

    /// Returns a list of available map names on the server.
    ///
    /// Map names can be used with [`load_world()`](Self::load_world) to switch maps.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// let maps = client.avaiable_maps();
    /// for map in maps {
    ///     println!("Available map: {}", map);
    /// }
    /// ```
    pub fn avaiable_maps(&self) -> Vec<String> {
        self.inner
            .GetAvailableMaps()
            .iter()
            .map(|s| s.to_string())
            .collect()
    }

    /// Loads a new map/world, resetting simulation settings to defaults.
    ///
    /// This is equivalent to calling [`load_world_opt()`](Self::load_world_opt)
    /// with `reset_settings = true`.
    ///
    /// # Arguments
    ///
    /// * `map_name` - Name of the map to load (e.g., "Town01", "Town02")
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// let world = client.load_world("Town03");
    /// ```
    pub fn load_world(&self, map_name: &str) -> World {
        self.load_world_opt(map_name, true)
    }

    /// Loads a new map/world with optional settings preservation.
    ///
    /// # Arguments
    ///
    /// * `map_name` - Name of the map to load
    /// * `reset_settings` - If true, resets simulation settings to defaults
    pub fn load_world_opt(&self, map_name: &str, reset_settings: bool) -> World {
        let world = self
            .inner
            .LoadWorld(map_name, reset_settings)
            .within_unique_ptr();
        unsafe { World::from_cxx(world).unwrap_unchecked() }
    }

    /// Loads a new map/world only if it's different from the current one.
    ///
    /// This method avoids unnecessary map reloads, which can save time when
    /// the requested map is already loaded. If the map is already loaded,
    /// returns the current world without reloading.
    ///
    /// **Available in CARLA 0.9.15+**
    ///
    /// **Note:** Currently only available when building with CARLA 0.9.16 due to
    /// build configuration. Support for 0.9.15 can be added if needed.
    ///
    /// # Arguments
    ///
    /// * `map_name` - Name of the map to load
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// // Only loads if different from current map
    /// let world = client.load_world_if_different("Town03");
    /// // This won't reload since Town03 is already loaded
    /// let world2 = client.load_world_if_different("Town03");
    /// ```
    #[cfg(carla_0916)]
    pub fn load_world_if_different(&self, map_name: &str) -> World {
        self.load_world_if_different_opt(map_name, true)
    }

    /// Loads a new map/world only if different, with optional settings preservation.
    ///
    /// **Available in CARLA 0.9.15+**
    ///
    /// # Arguments
    ///
    /// * `map_name` - Name of the map to load
    /// * `reset_settings` - If true, resets simulation settings to defaults
    #[cfg(carla_0916)]
    pub fn load_world_if_different_opt(&self, map_name: &str, reset_settings: bool) -> World {
        let world = self
            .inner
            .LoadWorldIfDifferent(map_name, reset_settings)
            .within_unique_ptr();
        unsafe { World::from_cxx(world).unwrap_unchecked() }
    }

    /// Reloads the current world, resetting simulation settings to defaults.
    ///
    /// This destroys all actors and reloads the same map.
    pub fn reload_world(&self) -> World {
        self.reload_world_opt(true)
    }

    /// Reloads the current world with optional settings preservation.
    ///
    /// # Arguments
    ///
    /// * `reset_settings` - If true, resets simulation settings to defaults
    pub fn reload_world_opt(&self, reset_settings: bool) -> World {
        let world = self.inner.ReloadWorld(reset_settings).within_unique_ptr();
        unsafe { World::from_cxx(world).unwrap_unchecked() }
    }

    /// Generates a world from an OpenDRIVE string.
    ///
    /// OpenDRIVE is an XML-based format for road network descriptions.
    ///
    /// # Arguments
    ///
    /// * `opendrive` - OpenDRIVE XML content as a string
    /// * `params` - Generation parameters for mesh and road properties
    /// * `reset_settings` - If true, resets simulation settings to defaults
    pub fn generate_open_drive_world(
        &self,
        opendrive: &str,
        params: &OpendriveGenerationParameters,
        reset_settings: bool,
    ) -> World {
        let world = self
            .inner
            .GenerateOpenDriveWorld(opendrive, params, reset_settings)
            .within_unique_ptr();
        unsafe { World::from_cxx(world).unwrap_unchecked() }
    }

    /// Returns the current simulation world.
    ///
    /// This is the primary way to access the simulation and interact with actors.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// let world = client.world();
    /// let actors = world.actors();
    /// ```
    pub fn world(&self) -> World {
        let world = self.inner.GetWorld().within_unique_ptr();
        unsafe { World::from_cxx(world).unwrap_unchecked() }
    }

    /// Creates or retrieves a Traffic Manager instance.
    ///
    /// The Traffic Manager controls autopilot behavior for vehicles. Multiple
    /// clients can share a Traffic Manager by using the same port.
    ///
    /// # Arguments
    ///
    /// * `port` - Traffic Manager port (default is 8000)
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// let mut tm = client.instance_tm(None); // Uses default port 8000
    /// ```
    pub fn instance_tm<P>(&self, port: P) -> TrafficManager
    where
        P: Into<Option<u16>>,
    {
        let port = port.into().unwrap_or(TM_DEFAULT_PORT);
        let ptr = self.inner.GetInstanceTM(port).within_unique_ptr();
        unsafe { TrafficManager::from_cxx(ptr).unwrap_unchecked() }
    }

    // ========================================================================
    // Recording Methods
    // ========================================================================

    /// Starts recording simulation data to a file.
    ///
    /// Recording captures actor movements, physics, and events for later replay.
    /// The recording file is saved in the CARLA server's directory.
    ///
    /// # Arguments
    ///
    /// * `filename` - Name of the recording file (e.g., "my_recording.log")
    /// * `additional_data` - If true, records non-essential data like sensor data
    ///
    /// # Returns
    ///
    /// A message string indicating success or failure
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let mut client = Client::default();
    /// let result = client.start_recorder("test_recording.log", false);
    /// println!("Recording started: {}", result);
    /// ```
    pub fn start_recorder(&mut self, filename: &str, additional_data: bool) -> String {
        self.inner
            .pin_mut()
            .StartRecorder(filename, additional_data)
            .to_string()
    }

    /// Stops the current recording session.
    ///
    /// The recording file is finalized and can be used for replay.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let mut client = Client::default();
    /// client.start_recorder("test.log", false);
    /// // ... perform actions ...
    /// client.stop_recorder();
    /// ```
    pub fn stop_recorder(&mut self) {
        self.inner.pin_mut().StopRecorder();
    }

    /// Retrieves information about a recorded simulation file.
    ///
    /// Returns detailed metadata about the recording including frames, time,
    /// events, and actor states.
    ///
    /// # Arguments
    ///
    /// * `filename` - Name of the recording file
    /// * `show_all` - If true, returns detailed info; if false, returns summary
    ///
    /// # Returns
    ///
    /// A formatted string containing the recording information
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// let info = client.show_recorder_file_info("test.log", false);
    /// println!("Recording info:\n{}", info);
    /// ```
    pub fn show_recorder_file_info(&mut self, filename: &str, show_all: bool) -> String {
        self.inner
            .pin_mut()
            .ShowRecorderFileInfo(filename, show_all)
            .to_string()
    }

    /// Queries collision events from a recorded simulation.
    ///
    /// Filters collisions by actor types using category characters:
    /// - 'h': Hero vehicle
    /// - 'v': Vehicle
    /// - 'w': Walker
    /// - 't': Traffic light
    /// - 'o': Other
    /// - 'a': Any
    ///
    /// # Arguments
    ///
    /// * `filename` - Name of the recording file
    /// * `category1` - First actor type filter
    /// * `category2` - Second actor type filter
    ///
    /// # Returns
    ///
    /// A formatted string listing collision events
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// // Show all vehicle-to-vehicle collisions
    /// let collisions = client.show_recorder_collisions("test.log", 'v', 'v');
    /// println!("Collisions:\n{}", collisions);
    /// ```
    pub fn show_recorder_collisions(
        &mut self,
        filename: &str,
        category1: char,
        category2: char,
    ) -> String {
        self.inner
            .pin_mut()
            .ShowRecorderCollisions(filename, category1 as i8, category2 as i8)
            .to_string()
    }

    /// Finds actors that were blocked (stuck) during the recording.
    ///
    /// An actor is considered blocked if it fails to move a minimum distance
    /// within a specified time period.
    ///
    /// # Arguments
    ///
    /// * `filename` - Name of the recording file
    /// * `min_time` - Minimum time in seconds to consider an actor blocked
    /// * `min_distance` - Minimum distance in meters the actor should have moved
    ///
    /// # Returns
    ///
    /// A formatted string listing blocked actors
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// // Find actors that didn't move 1m in 60 seconds
    /// let blocked = client.show_recorder_actors_blocked("test.log", 60.0, 1.0);
    /// println!("Blocked actors:\n{}", blocked);
    /// ```
    pub fn show_recorder_actors_blocked(
        &mut self,
        filename: &str,
        min_time: f32,
        min_distance: f32,
    ) -> String {
        self.inner
            .pin_mut()
            .ShowRecorderActorsBlocked(filename, min_time as f64, min_distance as f64)
            .to_string()
    }

    // ========================================================================
    // Replay Methods
    // ========================================================================

    /// Replays a previously recorded simulation.
    ///
    /// Loads and replays a recording file with optional time range and camera following.
    ///
    /// # Arguments
    ///
    /// * `filename` - Name of the recording file
    /// * `start_time` - Time in seconds to start playback (0.0 = from beginning)
    /// * `duration` - Duration in seconds to play (0.0 = until end)
    /// * `follow_id` - Actor ID to follow with camera (0 = no following)
    /// * `replay_sensors` - If true, replays sensor data
    ///
    /// # Returns
    ///
    /// A message string indicating success or failure
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let mut client = Client::default();
    /// // Replay from start, follow actor 42
    /// let result = client.replay_file("test.log", 0.0, 0.0, 42, false);
    /// println!("Replay started: {}", result);
    /// ```
    pub fn replay_file(
        &mut self,
        filename: &str,
        start_time: f32,
        duration: f32,
        follow_id: u32,
        replay_sensors: bool,
    ) -> String {
        self.inner
            .pin_mut()
            .ReplayFile(
                filename,
                start_time as f64,
                duration as f64,
                follow_id,
                replay_sensors,
            )
            .to_string()
    }

    /// Stops the current replay session.
    ///
    /// # Arguments
    ///
    /// * `keep_actors` - If true, actors remain in the world after stopping
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let mut client = Client::default();
    /// client.replay_file("test.log", 0.0, 0.0, 0, false);
    /// // ... wait for replay ...
    /// client.stop_replayer(false);
    /// ```
    pub fn stop_replayer(&mut self, keep_actors: bool) {
        self.inner.pin_mut().StopReplayer(keep_actors);
    }

    /// Sets the time factor for replay playback speed.
    ///
    /// # Arguments
    ///
    /// * `time_factor` - Speed multiplier (1.0 = normal, 2.0 = 2x speed, 0.5 = half speed)
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let mut client = Client::default();
    /// client.replay_file("test.log", 0.0, 0.0, 0, false);
    /// client.set_replayer_time_factor(2.0); // Play at 2x speed
    /// ```
    pub fn set_replayer_time_factor(&mut self, time_factor: f32) {
        self.inner
            .pin_mut()
            .SetReplayerTimeFactor(time_factor as f64);
    }

    /// Controls whether the hero vehicle is ignored during replay.
    ///
    /// # Arguments
    ///
    /// * `ignore_hero` - If true, the hero vehicle's recorded movements are not replayed
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let mut client = Client::default();
    /// client.replay_file("test.log", 0.0, 0.0, 0, false);
    /// client.set_replayer_ignore_hero(true); // Skip hero vehicle replay
    /// ```
    pub fn set_replayer_ignore_hero(&mut self, ignore_hero: bool) {
        self.inner.pin_mut().SetReplayerIgnoreHero(ignore_hero);
    }

    /// Controls whether the spectator is ignored during replay.
    ///
    /// Available in CARLA 0.9.15+
    ///
    /// # Arguments
    ///
    /// * `ignore_spectator` - If true, the spectator's recorded movements are not replayed
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let mut client = Client::default();
    /// client.replay_file("test.log", 0.0, 0.0, 0, false);
    /// client.set_replayer_ignore_spectator(true); // Skip spectator replay
    /// ```
    #[cfg(any(carla_version_0915, carla_version_0916))]
    pub fn set_replayer_ignore_spectator(&mut self, ignore_spectator: bool) {
        self.inner
            .pin_mut()
            .SetReplayerIgnoreSpectator(ignore_spectator);
    }

    // ========================================================================
    // Batch Operations
    // ========================================================================

    /// Executes a batch of commands without waiting for responses.
    ///
    /// This is the fire-and-forget version of batch execution. Commands are sent
    /// to the server but no results are returned. Use this when you don't need
    /// to check if commands succeeded.
    ///
    /// All commands in the batch are executed atomically in a single simulation step,
    /// which is significantly more efficient than individual calls.
    ///
    /// # Arguments
    ///
    /// * `commands` - Vector of commands to execute
    /// * `do_tick_cue` - If true, sends a tick cue after executing the batch
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::{client::Client, rpc::Command};
    ///
    /// let mut client = Client::default();
    /// let world = client.world();
    ///
    /// let commands = vec![
    ///     Command::console_command("weather.sun_altitude 45.0".to_string()),
    ///     Command::console_command("weather.cloudiness 50.0".to_string()),
    /// ];
    ///
    /// client.apply_batch(commands, false);
    /// ```
    pub fn apply_batch(&mut self, commands: Vec<Command>, do_tick_cue: bool) {
        let mut batch = FfiCommandBatch::new().within_unique_ptr();

        for command in &commands {
            command.add(&mut batch);
        }

        self.inner
            .pin_mut()
            .ApplyBatch(batch.pin_mut(), do_tick_cue);
    }

    /// Executes a batch of commands and returns responses for each.
    ///
    /// This is the synchronous version that waits for all commands to complete
    /// and returns their results. Use this when you need to check command success
    /// or retrieve actor IDs from spawn commands.
    ///
    /// All commands in the batch are executed atomically in a single simulation step,
    /// which is significantly more efficient than individual calls.
    ///
    /// # Arguments
    ///
    /// * `commands` - Vector of commands to execute
    /// * `do_tick_cue` - If true, sends a tick cue after executing the batch
    ///
    /// # Returns
    ///
    /// Vector of [`CommandResponse`] with results for each command
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::{client::Client, rpc::Command};
    ///
    /// let mut client = Client::default();
    /// let world = client.world();
    /// let blueprint_library = world.blueprint_library();
    ///
    /// // Spawn multiple vehicles at once
    /// let vehicle_bp = blueprint_library.find("vehicle.tesla.model3").unwrap();
    /// let spawn_points = world.map().recommended_spawn_points();
    ///
    /// let mut commands = Vec::new();
    /// for spawn_point in spawn_points.iter().take(10) {
    ///     commands.push(Command::spawn_actor(
    ///         vehicle_bp.clone(),
    ///         spawn_point.clone(),
    ///         None,
    ///     ));
    /// }
    ///
    /// let responses = client.apply_batch_sync(commands, true);
    /// for (i, response) in responses.iter().enumerate() {
    ///     if let Some(actor_id) = response.actor_id() {
    ///         println!("Spawned vehicle {}: actor ID {}", i, actor_id);
    ///     } else if let Some(error) = response.error() {
    ///         eprintln!("Failed to spawn vehicle {}: {}", i, error);
    ///     }
    /// }
    /// ```
    pub fn apply_batch_sync(
        &mut self,
        commands: Vec<Command>,
        do_tick_cue: bool,
    ) -> Vec<CommandResponse> {
        let mut batch = FfiCommandBatch::new().within_unique_ptr();

        for command in &commands {
            command.add(&mut batch);
        }

        let ffi_responses = self
            .inner
            .pin_mut()
            .ApplyBatchSync(batch.pin_mut(), do_tick_cue);

        ffi_responses
            .iter()
            .map(CommandResponse::from_ffi)
            .collect()
    }
}

impl Default for Client {
    /// Creates a client connected to localhost:2000 with default settings.
    ///
    /// This is equivalent to `Client::connect("localhost", 2000, None)`.
    fn default() -> Self {
        Self::connect("localhost", 2000, None)
    }
}

assert_impl_all!(Client: Sync, Send);
