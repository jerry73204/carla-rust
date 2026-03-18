use std::{
    env,
    fs::{self, File, OpenOptions},
    io::{self, Write},
    net::TcpStream,
    os::unix::process::CommandExt,
    path::PathBuf,
    process::Command,
    time::{Duration, Instant},
};

use carla::client::Client;

use super::config;

/// A lease on a CARLA server instance from the pool.
///
/// Holds a file lock to ensure exclusive access. When dropped, the lock is
/// released so other test processes can use the server. The CARLA process
/// is NOT killed — it persists across test processes for speed.
pub struct ServerLease {
    port: u16,
    _lock: File,
}

impl ServerLease {
    /// Acquire a server from the pool. Blocks until one is available.
    pub fn acquire() -> Self {
        let n = server_count();
        let base = base_port();

        loop {
            for i in 0..n {
                let port = base + (i as u16) * 10;
                if let Ok(lock) = try_lock(port) {
                    ensure_server_running(port);
                    return ServerLease { port, _lock: lock };
                }
            }
            std::thread::sleep(Duration::from_millis(500));
        }
    }

    /// The port number of the leased server.
    pub fn port(&self) -> u16 {
        self.port
    }
}

/// Number of concurrent CARLA instances in the pool.
fn server_count() -> usize {
    env::var("CARLA_TEST_SERVERS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(1)
}

/// Base port for the server pool.
fn base_port() -> u16 {
    env::var("CARLA_TEST_BASE_PORT")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(2000)
}

/// Directory for lock files.
fn lock_dir() -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("tmp")
        .join("test-servers");
    fs::create_dir_all(&dir).ok();
    dir
}

/// Try to acquire an exclusive file lock for the given port.
fn try_lock(port: u16) -> io::Result<File> {
    let path = lock_dir().join(format!("port-{port}.lock"));
    let file = OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(false)
        .open(&path)?;

    use std::os::unix::io::AsRawFd;
    let fd = file.as_raw_fd();
    let ret = unsafe { libc::flock(fd, libc::LOCK_EX | libc::LOCK_NB) };
    if ret != 0 {
        return Err(io::Error::last_os_error());
    }

    Ok(file)
}

/// Check if a server is accepting TCP connections on the given port.
fn tcp_is_ready(port: u16) -> bool {
    TcpStream::connect_timeout(
        &format!("127.0.0.1:{port}").parse().unwrap(),
        Duration::from_secs(2),
    )
    .is_ok()
}

/// Try to connect a CARLA client and call world() once with the configured probe timeout.
/// Returns true if the world is fully initialized and responding.
fn world_is_ready(port: u16, probe_timeout: Duration) -> bool {
    let Ok(mut client) = Client::connect("127.0.0.1", port, None) else {
        return false;
    };
    let _ = client.set_timeout(probe_timeout);
    client.world().is_ok()
}

/// Poll until world_is_ready returns true or the deadline is exceeded.
/// Returns false immediately if CARLA's TCP port goes down (process died).
fn wait_for_world_ready(port: u16, cfg: &config::TimeoutsConfig) -> bool {
    let deadline = Instant::now() + cfg.world_ready();
    while Instant::now() < deadline {
        if world_is_ready(port, cfg.world_probe()) {
            return true;
        }
        // If TCP is also down, CARLA died — no point waiting the full timeout
        if !tcp_is_ready(port) {
            eprintln!("CARLA on port {port} died while waiting for world to initialize");
            return false;
        }
        std::thread::sleep(cfg.world_probe_interval());
    }
    false
}

/// Kill any CARLA process running on the given port.
fn kill_server(port: u16) {
    // Kill via saved PID if available
    let pid_path = lock_dir().join(format!("port-{port}.pid"));
    if let Ok(content) = fs::read_to_string(&pid_path) {
        if let Ok(pid) = content.trim().parse::<libc::pid_t>() {
            unsafe {
                libc::kill(pid, libc::SIGKILL);
            }
        }
        let _ = fs::remove_file(&pid_path);
    }
    // Also kill by name pattern in case PID file is stale
    let _ = Command::new("pkill")
        .args(["-9", "-f", &format!("CarlaUE4.*carla-rpc-port={port}")])
        .status();
    std::thread::sleep(Duration::from_secs(2));
}

/// Ensure a CARLA server is running and the world is ready on the given port.
///
/// 1. If the world is already responding → done (fast path).
/// 2. If TCP is up but world is not responding → stale/hung CARLA → kill and restart.
/// 3. Start CARLA via the start script (blocks until CARLA is TCP-ready).
/// 4. Poll until the world is accessible (can take 60-120s after TCP ready when loading from swap).
/// 5. If CARLA dies during world loading, retry from step 2 (up to 3 total attempts).
fn ensure_server_running(port: u16) {
    let cfg = config::load();

    // Fast path: world is already responding (reusing a running instance)
    let t0 = Instant::now();
    if world_is_ready(port, cfg.timeouts.world_probe()) {
        eprintln!(
            "CARLA world already ready on port {port} (probe: {:.1}s)",
            t0.elapsed().as_secs_f32()
        );
        return;
    }

    let script = match env::var("CARLA_TEST_START_SCRIPT") {
        Ok(s) => s,
        Err(_) => {
            // No script: just require CARLA to already be running
            if tcp_is_ready(port) {
                panic!(
                    "CARLA is TCP-alive on port {port} but world is not ready and \
                     CARLA_TEST_START_SCRIPT is not set"
                );
            }
            panic!(
                "CARLA server not responding on port {port} and CARLA_TEST_START_SCRIPT is not set"
            );
        }
    };

    // Resolve relative paths against the workspace root
    let script = if std::path::Path::new(&script).is_relative() {
        let workspace_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .to_path_buf();
        workspace_root.join(&script).to_string_lossy().into_owned()
    } else {
        script
    };

    for attempt in 1..=cfg.server.max_restart_attempts {
        // Kill stale/hung CARLA before starting fresh
        if tcp_is_ready(port) {
            eprintln!(
                "CARLA on port {port} is TCP-alive but world unresponsive (attempt {attempt}); \
                 killing and restarting"
            );
            kill_server(port);
        }

        eprintln!(
            "Starting CARLA on port {port} via {script} \
             (attempt {attempt}/{})...",
            cfg.server.max_restart_attempts
        );
        let t_launch = Instant::now();

        // Spawn the start script. It blocks until CARLA's TCP port is ready, then exits.
        // CARLA itself keeps running as a detached background process.
        let child = unsafe {
            Command::new(&script)
                .arg(port.to_string())
                .pre_exec(|| {
                    libc::setsid(); // New session — fully detached from test process
                    Ok(())
                })
                .spawn()
        }
        .unwrap_or_else(|e| panic!("Failed to start CARLA via script {script}: {e}"));

        // Record start-script PID for cleanup
        let pid_path = lock_dir().join(format!("port-{port}.pid"));
        if let Ok(mut f) = File::create(&pid_path) {
            let _ = write!(f, "{}", child.id());
        }

        // Wait for the start script to exit (= CARLA TCP-ready)
        let mut child = child;
        match child.wait() {
            Ok(status) if status.success() => {}
            Ok(status) => {
                eprintln!("Start script failed (exit {status}) on attempt {attempt}; retrying...");
                continue;
            }
            Err(e) => {
                eprintln!("Failed to wait for start script on attempt {attempt}: {e}; retrying...");
                continue;
            }
        }
        let t_tcp = t_launch.elapsed();

        // Poll for full world readiness. The game map continues loading after TCP accepts
        // connections, and can take 60-120s when loading from swap on a busy server.
        eprintln!(
            "CARLA TCP ready in {:.1}s on port {port}; waiting for world to initialize...",
            t_tcp.as_secs_f32()
        );
        if wait_for_world_ready(port, &cfg.timeouts) {
            let t_world = t_launch.elapsed();
            eprintln!(
                "CARLA world ready on port {port} — total startup: {:.1}s \
                 (TCP: {:.1}s + world: {:.1}s)",
                t_world.as_secs_f32(),
                t_tcp.as_secs_f32(),
                (t_world - t_tcp).as_secs_f32(),
            );
            return;
        }

        eprintln!("CARLA world on port {port} not ready after attempt {attempt}; retrying...");
        kill_server(port);
    }

    panic!(
        "CARLA world on port {port} did not become ready after {} attempts",
        cfg.server.max_restart_attempts
    );
}
