use crate::config::CarlaTestConfig;
use std::{
    fs,
    net::TcpStream,
    process::{Child, Command, Stdio},
    time::{Duration, Instant},
};

/// Represents a running CARLA server instance managed for testing.
pub struct CarlaTestServer {
    process: Child,
    port: u16,
    config: CarlaTestConfig,
}

impl CarlaTestServer {
    /// Starts a new CARLA server instance with the given configuration.
    pub fn start(config: &CarlaTestConfig) -> Result<Self, Box<dyn std::error::Error>> {
        log::info!("Starting CARLA server on port {}", config.server.port);

        // Ensure log directory exists
        if config.logging.capture_output {
            fs::create_dir_all(&config.logging.log_dir)?;
        }

        // Kill any existing CARLA processes first
        Self::cleanup_existing_processes(config.server.port)?;

        // Wait a moment for cleanup to complete
        std::thread::sleep(Duration::from_millis(500));

        // Validate executable exists before trying to start
        if !std::path::Path::new(&config.server.executable_path).exists() {
            return Err(format!(
                "CARLA executable not found at: {}",
                config.server.executable_path
            )
            .into());
        }

        // Build command line arguments
        let mut args = vec![
            format!("-carla-port={}", config.server.port),
            format!("-quality-level={}", config.server.quality_level),
        ];

        if !config.server.windowed {
            args.push("-RenderOffScreen".to_string());
        }

        // Add additional user-specified arguments
        args.extend(config.server.additional_args.clone());

        // Configure stdio based on logging settings
        let (stdout, stderr) = if config.logging.capture_output {
            let stdout_path = format!(
                "{}/carla_server_{}_stdout.log",
                config.logging.log_dir, config.server.port
            );
            let stderr_path = format!(
                "{}/carla_server_{}_stderr.log",
                config.logging.log_dir, config.server.port
            );

            let stdout_file = fs::File::create(&stdout_path)?;
            let stderr_file = fs::File::create(&stderr_path)?;

            log::info!(
                "Server logs: stdout={}, stderr={}",
                stdout_path,
                stderr_path
            );

            (Stdio::from(stdout_file), Stdio::from(stderr_file))
        } else {
            (Stdio::null(), Stdio::null())
        };

        // Start the server process
        log::info!(
            "Executing: {} {}",
            config.server.executable_path,
            args.join(" ")
        );

        let mut command = Command::new(&config.server.executable_path);
        command
            .args(&args)
            .stdout(stdout)
            .stderr(stderr)
            .stdin(Stdio::null());

        // Handle environment variables
        // Preserve DISPLAY for windowed mode
        if config.server.windowed {
            if let Ok(display) = std::env::var("DISPLAY") {
                command.env("DISPLAY", display);
                log::info!(
                    "Using DISPLAY={}",
                    std::env::var("DISPLAY").unwrap_or_default()
                );
            } else {
                log::warn!("DISPLAY environment variable not set, windowed mode may fail");
            }
        }

        // Preserve other important environment variables
        // LD_LIBRARY_PATH for shared libraries
        if let Ok(ld_path) = std::env::var("LD_LIBRARY_PATH") {
            command.env("LD_LIBRARY_PATH", ld_path);
        }

        // PATH for finding executables
        if let Ok(path) = std::env::var("PATH") {
            command.env("PATH", path);
        }

        // HOME for user-specific settings
        if let Ok(home) = std::env::var("HOME") {
            command.env("HOME", home);
        }

        let process = command
            .spawn()
            .map_err(|e| format!("Failed to start CARLA server: {}", e))?;

        let pid = process.id();
        log::info!("Started CARLA server process with PID: {}", pid);

        let mut server = Self {
            process,
            port: config.server.port,
            config: config.clone(),
        };

        // Wait for server to be ready
        server.wait_for_startup()?;

        log::info!(
            "CARLA server started successfully on port {}",
            config.server.port
        );
        Ok(server)
    }

    /// Waits for the server to become ready by checking port availability and API responsiveness.
    fn wait_for_startup(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let timeout = Duration::from_secs(self.config.server.startup_timeout_seconds);
        let start = Instant::now();

        log::info!("Waiting for CARLA server to start (timeout: {:?})", timeout);

        loop {
            // Check if process is still running
            match self.process.try_wait() {
                Ok(Some(status)) => {
                    return Err(
                        format!("CARLA server process exited with status: {}", status).into(),
                    );
                }
                Ok(None) => {
                    // Process is still running, continue
                }
                Err(e) => {
                    return Err(format!("Failed to check server process status: {}", e).into());
                }
            }

            // Check if we've timed out
            if start.elapsed() > timeout {
                // Kill the process since it didn't start properly
                let _ = self.process.kill();
                return Err(format!("CARLA server failed to start within {:?}", timeout).into());
            }

            // Try to connect to the port
            if TcpStream::connect(("127.0.0.1", self.port)).is_ok() {
                log::info!("CARLA server port {} is open", self.port);

                // Port is open, now try to create a test client
                match self.test_client_connection() {
                    Ok(_) => {
                        log::info!("CARLA server API is responsive");
                        return Ok(());
                    }
                    Err(e) => {
                        log::debug!("API not ready yet: {}", e);
                    }
                }
            }

            // Wait before retrying
            std::thread::sleep(Duration::from_millis(1000));
        }
    }

    /// Tests if the CARLA API is responsive by creating a test client connection.
    fn test_client_connection(&self) -> Result<(), Box<dyn std::error::Error>> {
        // Try to create a client and get server version
        let client = carla::client::Client::new("localhost", self.port, None)?;
        let _version = client.server_version()?;
        Ok(())
    }

    /// Kills any existing CARLA processes that might be running.
    fn cleanup_existing_processes(port: u16) -> Result<(), Box<dyn std::error::Error>> {
        log::info!("Cleaning up existing CARLA processes");

        // Try to kill CARLA processes by name
        let output = Command::new("pkill").arg("-f").arg("CarlaUnreal").output();

        match output {
            Ok(output) if output.status.success() => {
                log::info!("Killed existing CarlaUnreal processes");
            }
            Ok(_) => {
                log::debug!("No CarlaUnreal processes found");
            }
            Err(e) => {
                log::debug!("pkill command not available or failed: {}", e);
            }
        }

        // Also try with lowercase
        let _ = Command::new("pkill").arg("-f").arg("carla").output();

        // Additional cleanup for any hanging ports
        Self::cleanup_port(port)?;

        Ok(())
    }

    /// Attempts to clean up a specific port if it's in use.
    fn cleanup_port(port: u16) -> Result<(), Box<dyn std::error::Error>> {
        // Try to find process using the port
        let output = Command::new("lsof")
            .arg("-ti")
            .arg(format!(":{}", port))
            .output();

        if let Ok(output) = output {
            if output.status.success() && !output.stdout.is_empty() {
                let pids = String::from_utf8_lossy(&output.stdout);
                for pid in pids.lines() {
                    if let Ok(pid_num) = pid.trim().parse::<i32>() {
                        log::info!("Killing process {} using port {}", pid_num, port);
                        let _ = Command::new("kill")
                            .arg("-9")
                            .arg(pid_num.to_string())
                            .output();
                    }
                }
            }
        }

        Ok(())
    }

    /// Returns the port the server is listening on.
    pub fn port(&self) -> u16 {
        self.port
    }
}

impl Drop for CarlaTestServer {
    fn drop(&mut self) {
        log::info!("Shutting down CARLA server on port {}", self.port);

        // Try graceful termination first
        if let Err(e) = self.process.kill() {
            log::warn!("Failed to terminate CARLA server gracefully: {}", e);
        }

        // Wait for graceful shutdown
        let timeout = Duration::from_secs(self.config.server.shutdown_timeout_seconds);
        let start = Instant::now();

        while start.elapsed() < timeout {
            match self.process.try_wait() {
                Ok(Some(_)) => {
                    log::info!("CARLA server shut down gracefully");
                    return;
                }
                Ok(None) => {
                    // Still running, continue waiting
                    std::thread::sleep(Duration::from_millis(100));
                }
                Err(e) => {
                    log::warn!("Error checking server shutdown status: {}", e);
                    break;
                }
            }
        }

        // Force kill if graceful shutdown failed
        log::warn!("Force killing CARLA server");
        let _ = self.process.kill();
        let _ = self.process.wait();

        // Final cleanup of any remaining processes
        let _ = Self::cleanup_existing_processes(self.port);
    }
}
