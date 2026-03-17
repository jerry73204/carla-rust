use std::{
    env,
    fs::{self, File, OpenOptions},
    io,
    net::TcpStream,
    path::PathBuf,
    process::{Child, Command},
    time::Duration,
};

use crate::common::process::{graceful_kill_process_group, set_new_process_group};

/// A lease on a CARLA server instance from the pool.
///
/// Holds a file lock to ensure exclusive access. When dropped, the lock is
/// released and any managed CARLA process is killed.
pub struct ServerLease {
    port: u16,
    _lock: File,
    _process: Option<Child>,
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
                    let process = maybe_start_server(port);
                    return ServerLease {
                        port,
                        _lock: lock,
                        _process: process,
                    };
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

impl Drop for ServerLease {
    fn drop(&mut self) {
        if let Some(ref mut child) = self._process {
            graceful_kill_process_group(child);
        }
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

/// Start a CARLA server via the user-provided script, if configured.
/// Waits for the server to accept TCP connections before returning.
#[allow(clippy::zombie_processes)] // Child is wait()'d in ServerLease::drop via graceful_kill_process_group
fn maybe_start_server(port: u16) -> Option<Child> {
    let script = env::var("CARLA_TEST_START_SCRIPT").ok()?;

    // Check if server is already responding on this port
    if TcpStream::connect_timeout(
        &format!("127.0.0.1:{port}").parse().unwrap(),
        Duration::from_secs(1),
    )
    .is_ok()
    {
        return None;
    }

    let mut cmd = Command::new(&script);
    cmd.arg(port.to_string());
    set_new_process_group(&mut cmd);

    let child = cmd.spawn().unwrap_or_else(|e| {
        panic!("Failed to start CARLA via script {script}: {e}");
    });

    // Wait for server to become ready (up to 120s)
    let deadline = std::time::Instant::now() + Duration::from_secs(120);
    while std::time::Instant::now() < deadline {
        if TcpStream::connect_timeout(
            &format!("127.0.0.1:{port}").parse().unwrap(),
            Duration::from_secs(2),
        )
        .is_ok()
        {
            // Give the server a moment to fully initialize after accepting TCP
            std::thread::sleep(Duration::from_secs(2));
            return Some(child);
        }
        std::thread::sleep(Duration::from_secs(1));
    }

    panic!("CARLA server on port {port} did not become ready within 120s");
}
