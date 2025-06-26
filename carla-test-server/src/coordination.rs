use crate::{
    config::CarlaTestConfig,
    server::{CarlaTestServer, ServerResource},
};
use std::{
    fs::OpenOptions,
    os::unix::{fs::OpenOptionsExt, io::AsRawFd},
};

/// Trait for different server coordination strategies.
///
/// This trait allows for different implementations of server acquisition,
/// such as single-server with file locking or future server pooling.
pub trait ServerCoordinator {
    /// Acquires a server resource for testing.
    ///
    /// This method should block until a server is available and return
    /// a `ServerResource` that provides access to the server and ensures
    /// proper cleanup when dropped.
    fn acquire_server(&self) -> Result<ServerResource, Box<dyn std::error::Error>>;
}

/// File-based coordinator that ensures sequential execution of CARLA tests.
///
/// This coordinator uses Unix file locking to ensure that only one CARLA test
/// runs at a time across multiple processes. This is essential for nextest
/// which runs each test in its own process.
pub struct FileLockCoordinator {
    config: CarlaTestConfig,
}

/// RAII guard that automatically releases the file lock when dropped.
pub struct FileLockGuard {
    _file: std::fs::File,
    lock_file_path: String,
}

impl FileLockCoordinator {
    /// Creates a new file lock coordinator with the given configuration.
    pub fn new(config: &CarlaTestConfig) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self {
            config: config.clone(),
        })
    }
}

impl ServerCoordinator for FileLockCoordinator {
    fn acquire_server(&self) -> Result<ServerResource, Box<dyn std::error::Error>> {
        log::info!("Acquiring CARLA server lock");

        // Acquire file lock
        let lock_guard = self.acquire_file_lock()?;

        log::info!("Lock acquired, starting CARLA server");

        // Start single CARLA server
        let server = CarlaTestServer::start(&self.config)?;

        Ok(ServerResource::new(server, lock_guard))
    }
}

impl FileLockCoordinator {
    /// Acquires an exclusive file lock, blocking until available.
    fn acquire_file_lock(&self) -> Result<FileLockGuard, Box<dyn std::error::Error>> {
        use std::{fs, path::Path};

        let lock_file_path = &self.config.coordination.lock_file;

        // Ensure the parent directory exists
        if let Some(parent) = Path::new(lock_file_path).parent() {
            fs::create_dir_all(parent)?;
        }

        log::debug!("Acquiring file lock: {}", lock_file_path);

        let file = OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .mode(0o600) // rw-------
            .open(lock_file_path)?;

        let fd = file.as_raw_fd();

        // Use flock for exclusive locking (blocks until available)
        let result = unsafe { libc::flock(fd, libc::LOCK_EX) };

        if result == 0 {
            log::debug!("File lock acquired: {}", lock_file_path);
            Ok(FileLockGuard {
                _file: file,
                lock_file_path: lock_file_path.clone(),
            })
        } else {
            let error = std::io::Error::last_os_error();
            Err(format!("Failed to acquire CARLA test lock: {}", error).into())
        }
    }
}

impl Drop for FileLockGuard {
    fn drop(&mut self) {
        log::debug!("Releasing file lock: {}", self.lock_file_path);

        // Lock is automatically released when file is closed
        // We can optionally remove the lock file, but it's not strictly necessary
        // since other processes will be able to acquire the lock once the file descriptor is closed
        let _ = std::fs::remove_file(&self.lock_file_path);
    }
}
