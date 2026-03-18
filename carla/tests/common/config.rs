use std::{path::PathBuf, time::Duration};

use serde::Deserialize;

/// Top-level CARLA test configuration, loaded from `.config/carla-test.toml`.
#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct CarlaTestConfig {
    pub server: ServerConfig,
    pub timeouts: TimeoutsConfig,
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct ServerConfig {
    /// How many times to attempt starting CARLA before giving up.
    pub max_restart_attempts: u32,
}

#[derive(Debug, Deserialize)]
#[serde(default)]
pub struct TimeoutsConfig {
    /// Per-attempt timeout for the `world()` RPC probe (seconds).
    pub world_probe_secs: u64,
    /// Sleep between consecutive world readiness probe attempts (seconds).
    pub world_probe_interval_secs: u64,
    /// Total budget after TCP ready to wait for game world (seconds).
    pub world_ready_secs: u64,
}

impl Default for CarlaTestConfig {
    fn default() -> Self {
        Self {
            server: ServerConfig::default(),
            timeouts: TimeoutsConfig::default(),
        }
    }
}

impl Default for ServerConfig {
    fn default() -> Self {
        Self {
            max_restart_attempts: 3,
        }
    }
}

impl Default for TimeoutsConfig {
    fn default() -> Self {
        Self {
            world_probe_secs: 30,
            world_probe_interval_secs: 5,
            world_ready_secs: 240,
        }
    }
}

impl TimeoutsConfig {
    pub fn world_probe(&self) -> Duration {
        Duration::from_secs(self.world_probe_secs)
    }
    pub fn world_probe_interval(&self) -> Duration {
        Duration::from_secs(self.world_probe_interval_secs)
    }
    pub fn world_ready(&self) -> Duration {
        Duration::from_secs(self.world_ready_secs)
    }
}

/// Load the config from `.config/carla-test.toml` in the workspace root.
/// Falls back to compiled-in defaults if the file is absent or unreadable.
pub fn load() -> CarlaTestConfig {
    let path = workspace_root().join(".config").join("carla-test.toml");
    let Ok(text) = std::fs::read_to_string(&path) else {
        return CarlaTestConfig::default();
    };
    match toml::from_str::<CarlaTestConfig>(&text) {
        Ok(cfg) => cfg,
        Err(e) => {
            eprintln!(
                "Warning: failed to parse {}: {e}; using defaults",
                path.display()
            );
            CarlaTestConfig::default()
        }
    }
}

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf()
}
