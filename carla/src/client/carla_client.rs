// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::World;
use crate::{
    rpc::OpendriveGenerationParameters,
    traffic_manager::{constants::Networking::TM_DEFAULT_PORT, TrafficManager},
};
use autocxx::prelude::*;
use carla_sys::carla_rust::client::FfiClient;
use cxx::{let_cxx_string, UniquePtr};
use derivative::Derivative;
use static_assertions::assert_impl_all;
use std::time::Duration;

/// The client maintains the connection to the CARLA simulator server,
/// corresponding to `carla.Client` in Python API.
///
/// The [`Client`] is your entry point for interacting with a CARLA simulation.
/// It manages the network connection to the server and provides methods to:
/// - Access the simulation [`World`]
/// - Load and manage maps
/// - Create [`TrafficManager`] instances
/// - Check version compatibility
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
