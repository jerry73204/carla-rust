//! CARLA client for connecting to server.

use crate::{client::World, error::CarlaResult};
use carla_sys::ClientWrapper;
use std::time::Duration;

/// Main client for connecting to CARLA server.
#[derive(Debug)]
pub struct Client {
    /// Internal handle to carla-sys Client
    inner: ClientWrapper,
}

impl Client {
    /// Create a new client and connect to CARLA server.
    ///
    /// # Arguments
    /// * `host` - Server hostname or IP address
    /// * `port` - Server port number
    /// * `worker_threads` - Number of worker threads (None for default)
    pub fn new<W>(host: &str, port: u16, worker_threads: W) -> CarlaResult<Self>
    where
        W: Into<Option<usize>>,
    {
        let _worker_threads = worker_threads.into().unwrap_or(0);
        let inner = ClientWrapper::new(host, port).map_err(|e| {
            crate::error::CarlaError::Client(crate::error::ClientError::ConnectionFailed {
                host: host.to_string(),
                port,
                reason: e.to_string(),
            })
        })?;

        Ok(Self { inner })
    }

    /// Get the client version.
    pub fn client_version(&self) -> String {
        // Return the version from the carla crate
        env!("CARGO_PKG_VERSION").to_string()
    }

    /// Get the server version.
    pub fn server_version(&self) -> CarlaResult<String> {
        let version = self.inner.get_server_version();
        // Check if it's an error response from C++ exception handling
        if version.starts_with("ERROR: ") {
            Err(crate::error::CarlaError::Client(
                crate::error::ClientError::ServerCommunicationFailed {
                    reason: version
                        .strip_prefix("ERROR: ")
                        .unwrap_or(&version)
                        .to_string(),
                },
            ))
        } else {
            Ok(version)
        }
    }

    /// Set connection timeout.
    pub fn set_timeout(&mut self, timeout: Duration) -> CarlaResult<()> {
        self.inner.set_timeout(timeout);
        Ok(())
    }

    /// Get current connection timeout.
    pub fn timeout(&mut self) -> CarlaResult<Duration> {
        Ok(self.inner.get_timeout())
    }

    /// Get the current world.
    pub fn world(&self) -> CarlaResult<World> {
        let world_wrapper = self.inner.get_world().map_err(|e| {
            crate::error::CarlaError::Client(crate::error::ClientError::ServerCommunicationFailed {
                reason: e.to_string(),
            })
        })?;
        Ok(World::from_cxx(world_wrapper))
    }

    /// Get available maps.
    pub fn available_maps(&self) -> CarlaResult<Vec<String>> {
        Ok(self.inner.get_available_maps())
    }

    /// Load a new world/map.
    pub fn load_world(&self, map_name: &str) -> CarlaResult<World> {
        let world_wrapper = self.inner.load_world(map_name).map_err(|e| {
            crate::error::CarlaError::World(crate::error::WorldError::MapLoadFailed {
                map_name: map_name.to_string(),
                reason: e.to_string(),
            })
        })?;
        Ok(World::from_cxx(world_wrapper))
    }

    /// Reload the current world.
    pub fn reload_world(&self) -> CarlaResult<World> {
        let world_wrapper = self.inner.reload_world(true).map_err(|e| {
            crate::error::CarlaError::Client(crate::error::ClientError::ServerCommunicationFailed {
                reason: format!("Failed to reload world: {}", e),
            })
        })?; // reset_settings = true by default
        Ok(World::from_cxx(world_wrapper))
    }

    /// Generate OpenDRIVE map from file.
    pub fn generate_opendrive_world(&self, opendrive: &str) -> CarlaResult<World> {
        let world_wrapper =
            self.inner
                .generate_opendrive_world(opendrive)
                .map_err(|e| {
                    crate::error::CarlaError::Map(crate::error::MapError::OpenDriveParsing(
                        format!("Failed to generate OpenDRIVE world: {}", e),
                    ))
                })?;
        Ok(World::from_cxx(world_wrapper))
    }

    // Recording functionality

    /// Start recording simulation data to a file.
    ///
    /// # Arguments
    /// * `filename` - Output file path for the recording
    /// * `additional_data` - Whether to include additional sensor data
    ///
    /// # Returns
    /// String describing the recording status
    pub fn start_recorder(&self, filename: &str, additional_data: bool) -> CarlaResult<String> {
        Ok(self.inner.start_recorder(filename, additional_data))
    }

    /// Stop the current recording session.
    pub fn stop_recorder(&self) -> CarlaResult<()> {
        self.inner.stop_recorder();
        Ok(())
    }

    /// Show information about a recording file.
    ///
    /// # Arguments
    /// * `filename` - Path to the recording file
    /// * `show_all` - Whether to show detailed information
    ///
    /// # Returns
    /// String containing file information
    pub fn show_recorder_file_info(&self, filename: &str, show_all: bool) -> CarlaResult<String> {
        Ok(self.inner.show_recorder_file_info(filename, show_all))
    }

    /// Show collision analysis from a recording file.
    ///
    /// # Arguments
    /// * `filename` - Path to the recording file
    /// * `type1` - First actor type filter ('a' for all, 'v' for vehicles, 'w' for walkers)
    /// * `type2` - Second actor type filter
    ///
    /// # Returns
    /// String containing collision analysis
    pub fn show_recorder_collisions(
        &self,
        filename: &str,
        type1: u8,
        type2: u8,
    ) -> CarlaResult<String> {
        Ok(self.inner.show_recorder_collisions(filename, type1, type2))
    }

    /// Show analysis of blocked actors from a recording file.
    ///
    /// # Arguments
    /// * `filename` - Path to the recording file
    /// * `min_time` - Minimum time threshold for considering an actor blocked
    /// * `min_distance` - Minimum distance threshold for considering an actor blocked
    ///
    /// # Returns
    /// String containing blocked actors analysis
    pub fn show_recorder_actors_blocked(
        &self,
        filename: &str,
        min_time: f64,
        min_distance: f64,
    ) -> CarlaResult<String> {
        Ok(self
            .inner
            .show_recorder_actors_blocked(filename, min_time, min_distance))
    }

    // Playback functionality

    /// Replay a previously recorded simulation.
    ///
    /// # Arguments
    /// * `filename` - Path to the recording file
    /// * `start_time` - Starting time for playback (in seconds)
    /// * `duration` - Duration of playback (in seconds, 0.0 for full replay)
    /// * `follow_id` - Actor ID to follow during playback (0 for no following)
    /// * `replay_sensors` - Whether to replay sensor data
    ///
    /// # Returns
    /// String describing the replay status
    pub fn replay_file(
        &self,
        filename: &str,
        start_time: f64,
        duration: f64,
        follow_id: u32,
        replay_sensors: bool,
    ) -> CarlaResult<String> {
        Ok(self
            .inner
            .replay_file(filename, start_time, duration, follow_id, replay_sensors))
    }

    /// Stop the current replay session.
    ///
    /// # Arguments
    /// * `keep_actors` - Whether to keep actors from the replay in the world
    pub fn stop_replayer(&self, keep_actors: bool) -> CarlaResult<()> {
        self.inner.stop_replayer(keep_actors);
        Ok(())
    }

    /// Set the replay speed multiplier.
    ///
    /// # Arguments
    /// * `time_factor` - Speed multiplier (1.0 = normal speed, 2.0 = double speed, etc.)
    pub fn set_replayer_time_factor(&self, time_factor: f64) -> CarlaResult<()> {
        self.inner.set_replayer_time_factor(time_factor);
        Ok(())
    }

    /// Set whether to ignore hero vehicles during replay.
    ///
    /// # Arguments
    /// * `ignore_hero` - If true, hero vehicles from the recording will be ignored
    pub fn set_replayer_ignore_hero(&self, ignore_hero: bool) -> CarlaResult<()> {
        self.inner.set_replayer_ignore_hero(ignore_hero);
        Ok(())
    }

    /// Set whether to ignore spectator during replay.
    ///
    /// # Arguments
    /// * `ignore_spectator` - If true, spectator from the recording will be ignored
    pub fn set_replayer_ignore_spectator(&self, ignore_spectator: bool) -> CarlaResult<()> {
        self.inner.set_replayer_ignore_spectator(ignore_spectator);
        Ok(())
    }

    // Convenience methods for common operations

    /// Start a basic recording with default settings.
    ///
    /// # Arguments
    /// * `filename` - Output file path for the recording
    pub fn start_recording(&self, filename: &str) -> CarlaResult<String> {
        self.start_recorder(filename, false)
    }

    /// Replay a full recording from the beginning.
    ///
    /// # Arguments
    /// * `filename` - Path to the recording file
    pub fn replay_recording(&self, filename: &str) -> CarlaResult<String> {
        self.replay_file(filename, 0.0, 0.0, 0, false)
    }

    /// Get available maps as an alias for get_available_maps.
    pub fn list_available_maps(&self) -> CarlaResult<Vec<String>> {
        self.available_maps()
    }

    /// Load world with error handling for invalid map names.
    pub fn load_map(&self, map_name: &str) -> CarlaResult<World> {
        // First check if the map exists
        let available_maps = self.available_maps()?;
        if !available_maps.iter().any(|m| m == map_name) {
            return Err(crate::error::CarlaError::Client(
                crate::error::ClientError::MapNotFound {
                    map_name: map_name.to_string(),
                    available_maps,
                },
            ));
        }
        self.load_world(map_name)
    }

    /// Get reference to the internal ClientWrapper for advanced operations
    pub fn inner(&self) -> &ClientWrapper {
        &self.inner
    }
}
