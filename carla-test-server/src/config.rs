use serde::Deserialize;
use std::path::Path;

/// Main configuration structure for CARLA test server management.
#[derive(Debug, Deserialize, Clone, Default)]
pub struct CarlaTestConfig {
    /// Server configuration
    pub server: ServerConfig,
    /// Cross-process coordination configuration
    pub coordination: CoordinationConfig,
    /// Logging configuration
    pub logging: LoggingConfig,
}

/// Configuration for CARLA server process management.
#[derive(Debug, Deserialize, Clone)]
pub struct ServerConfig {
    /// Path to CARLA executable (CarlaUnreal.sh or similar)
    pub executable_path: String,
    /// Port for CARLA server to listen on
    pub port: u16,
    /// Timeout in seconds to wait for server startup
    pub startup_timeout_seconds: u64,
    /// Timeout in seconds to wait for server shutdown
    pub shutdown_timeout_seconds: u64,
    /// Graphics quality level (Low, Medium, High, Epic)
    pub quality_level: String,
    /// Whether to run in windowed mode (requires DISPLAY)
    pub windowed: bool,
    /// Additional command line arguments to pass to CARLA
    pub additional_args: Vec<String>,
}

/// Configuration for cross-process coordination.
#[derive(Debug, Deserialize, Clone)]
pub struct CoordinationConfig {
    /// Path to lock file for sequential test execution
    pub lock_file: String,
    /// Timeout in seconds to wait for lock acquisition
    pub timeout_seconds: u64,
}

/// Configuration for logging and output capture.
#[derive(Debug, Deserialize, Clone)]
pub struct LoggingConfig {
    /// Directory to store server logs
    pub log_dir: String,
    /// Whether to capture server stdout/stderr
    pub capture_output: bool,
}

impl Default for ServerConfig {
    fn default() -> Self {
        Self {
            executable_path: "/opt/carla-simulator/CarlaUnreal.sh".to_string(),
            port: 2000,
            startup_timeout_seconds: 30,
            shutdown_timeout_seconds: 10,
            quality_level: "Low".to_string(),
            windowed: false,
            additional_args: vec!["-nosound".to_string(), "-benchmark".to_string()],
        }
    }
}

impl Default for CoordinationConfig {
    fn default() -> Self {
        Self {
            lock_file: "/tmp/carla_test_server.lock".to_string(),
            timeout_seconds: 300,
        }
    }
}

impl Default for LoggingConfig {
    fn default() -> Self {
        Self {
            log_dir: "/tmp/carla_test_logs".to_string(),
            capture_output: true,
        }
    }
}

/// Loads configuration from `carla_server.toml` file.
///
/// The function searches for the configuration file in the following order:
/// 1. `carla/carla_server.toml` (relative to workspace root)
/// 2. `carla_server.toml` (in current directory)
/// 3. Falls back to default configuration if no file is found
///
/// # Returns
///
/// * `Ok(CarlaTestConfig)` - Successfully loaded configuration
/// * `Err` - If the configuration file exists but cannot be parsed
///
/// # Examples
///
/// ```rust
/// let config = carla_test_server::load_config()?;
/// println!("CARLA executable: {}", config.server.executable_path);
/// ```
pub fn load_config() -> Result<CarlaTestConfig, Box<dyn std::error::Error>> {
    // Try to find configuration file
    let config_paths = ["carla/carla_server.toml", "carla_server.toml"];

    for path in &config_paths {
        if Path::new(path).exists() {
            log::info!("Loading CARLA test configuration from: {}", path);
            let content = std::fs::read_to_string(path)?;
            let config: CarlaTestConfig = toml::from_str(&content)?;

            // Validate configuration
            validate_config(&config)?;

            return Ok(config);
        }
    }

    // No configuration file found - use defaults
    log::info!("No carla_server.toml found, using default configuration");
    let config = CarlaTestConfig::default();
    validate_config(&config)?;
    Ok(config)
}

/// Validates the configuration for common issues.
fn validate_config(config: &CarlaTestConfig) -> Result<(), Box<dyn std::error::Error>> {
    // Check if executable exists (if path is absolute)
    let exec_path = Path::new(&config.server.executable_path);
    if exec_path.is_absolute() && !exec_path.exists() {
        return Err(format!(
            "CARLA executable not found at: {}",
            config.server.executable_path
        )
        .into());
    }

    // Validate port range
    if config.server.port < 1024 {
        return Err("Server port should be >= 1024 for non-root users".into());
    }

    // Validate timeouts
    if config.server.startup_timeout_seconds == 0 {
        return Err("Server startup timeout must be > 0".into());
    }

    if config.coordination.timeout_seconds == 0 {
        return Err("Coordination timeout must be > 0".into());
    }

    // Validate quality level
    let valid_quality_levels = ["Low", "Medium", "High", "Epic"];
    if !valid_quality_levels.contains(&config.server.quality_level.as_str()) {
        return Err(format!(
            "Invalid quality level '{}'. Valid options: {:?}",
            config.server.quality_level, valid_quality_levels
        )
        .into());
    }

    Ok(())
}
