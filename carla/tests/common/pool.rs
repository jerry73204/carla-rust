use std::{
    fs::{self, File, OpenOptions},
    io,
    net::TcpStream,
    path::PathBuf,
    time::{Duration, Instant},
};

use carla::client::Client;

use super::config;

/// A lease on a CARLA server instance from the pool.
///
/// Holds a file lock to ensure exclusive access. When dropped, the lock is
/// released so other test processes can use the server. Lifecycle management
/// (start/stop) is handled externally — by `just test-integration` or manually.
pub struct ServerLease {
    port: u16,
    _lock: File,
}

impl ServerLease {
    /// Acquire a server from the pool. Blocks until one is available and its
    /// world is responding. Panics if CARLA is not running on any pool port.
    pub fn acquire() -> Self {
        let cfg = config::load();
        let n = server_count();
        let base = base_port();

        loop {
            for i in 0..n {
                let port = base + (i as u16) * 10;
                if let Ok(lock) = try_lock(port) {
                    wait_for_world_ready(port, &cfg.timeouts);
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
    std::env::var("CARLA_TEST_SERVERS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(1)
}

/// Base port for the server pool.
fn base_port() -> u16 {
    std::env::var("CARLA_TEST_BASE_PORT")
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

/// Try to connect and call world() once with the configured probe timeout.
fn world_is_ready(port: u16, probe_timeout: Duration) -> bool {
    let Ok(mut client) = Client::connect("127.0.0.1", port, None) else {
        return false;
    };
    let _ = client.set_timeout(probe_timeout);
    client.world().is_ok()
}

/// Wait until world() responds or the configured deadline is reached.
/// Panics immediately if CARLA's TCP port is not open (process not running).
fn wait_for_world_ready(port: u16, cfg: &config::TimeoutsConfig) {
    if !tcp_is_ready(port) {
        panic!(
            "CARLA server is not running on port {port}.\n\
             Start it with:  just test-integration\n\
             Or manually:    ./scripts/start-carla-test.sh {port}"
        );
    }

    let t0 = Instant::now();
    let deadline = t0 + cfg.world_ready();

    while Instant::now() < deadline {
        if world_is_ready(port, cfg.world_probe()) {
            eprintln!(
                "CARLA world ready on port {port} (waited {:.1}s)",
                t0.elapsed().as_secs_f32()
            );
            return;
        }
        if !tcp_is_ready(port) {
            panic!(
                "CARLA server on port {port} went away while waiting for world readiness.\n\
                 Check logs: /tmp/carla-test-port-{port}.log"
            );
        }
        std::thread::sleep(cfg.world_probe_interval());
    }

    panic!(
        "CARLA world on port {port} did not become ready within {}s.\n\
         Check logs: /tmp/carla-test-port-{port}.log",
        cfg.world_ready_secs
    );
}
