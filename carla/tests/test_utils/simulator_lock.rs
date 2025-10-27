//! File-based locking mechanism for exclusive simulator access
//!
//! This module provides a file-based lock to ensure only one integration test
//! accesses the CARLA simulator at a time. This is necessary because CARLA can
//! only run one scenario at a time.

use std::{
    fs::{File, OpenOptions},
    io,
    path::PathBuf,
    time::Duration,
};

/// A file-based lock for exclusive simulator access
#[derive(Debug)]
pub struct SimulatorLock {
    _file: File,
    path: PathBuf,
}

impl SimulatorLock {
    /// Attempt to acquire the simulator lock
    ///
    /// Returns `Ok(SimulatorLock)` if the lock is acquired successfully.
    /// Returns `Err` if the lock cannot be acquired.
    pub fn try_acquire() -> io::Result<Self> {
        let lock_path = Self::lock_path();

        // Try to create the lock file exclusively
        let file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&lock_path)?;

        Ok(SimulatorLock {
            _file: file,
            path: lock_path,
        })
    }

    /// Attempt to acquire the lock with retry logic
    ///
    /// Tries to acquire the lock, retrying up to `max_retries` times with
    /// `retry_delay` between attempts.
    pub fn acquire_with_retry(max_retries: usize, retry_delay: Duration) -> io::Result<Self> {
        for attempt in 0..max_retries {
            match Self::try_acquire() {
                Ok(lock) => return Ok(lock),
                Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {
                    if attempt < max_retries - 1 {
                        std::thread::sleep(retry_delay);
                        continue;
                    }
                    return Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        format!(
                            "Failed to acquire simulator lock after {} attempts",
                            max_retries
                        ),
                    ));
                }
                Err(e) => return Err(e),
            }
        }
        unreachable!()
    }

    fn lock_path() -> PathBuf {
        let temp_dir = std::env::temp_dir();
        temp_dir.join("carla_simulator.lock")
    }
}

impl Drop for SimulatorLock {
    fn drop(&mut self) {
        // Clean up lock file when dropped
        let _ = std::fs::remove_file(&self.path);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulator_lock_acquisition() {
        // First lock should succeed
        let _lock1 = SimulatorLock::try_acquire().expect("Should acquire first lock");

        // Second lock should fail while first is held
        let result = SimulatorLock::try_acquire();
        assert!(result.is_err());
        assert_eq!(result.unwrap_err().kind(), io::ErrorKind::AlreadyExists);

        // Lock is released when _lock1 is dropped
    }

    #[test]
    fn test_simulator_lock_release() {
        {
            let _lock = SimulatorLock::try_acquire().expect("Should acquire lock");
            // Lock is held here
        }
        // Lock should be released after drop

        // Should be able to acquire again
        let _lock2 = SimulatorLock::try_acquire().expect("Should acquire lock after release");
    }

    #[test]
    fn test_simulator_lock_timeout() {
        let _lock = SimulatorLock::try_acquire().expect("Should acquire lock");

        // Try to acquire with retry - should timeout
        let result = SimulatorLock::acquire_with_retry(3, Duration::from_millis(10));
        assert!(result.is_err());
        assert_eq!(result.unwrap_err().kind(), io::ErrorKind::TimedOut);
    }

    #[test]
    fn test_simulator_lock_retry_success() {
        use std::{
            sync::{Arc, Barrier},
            thread,
        };

        let barrier = Arc::new(Barrier::new(2));
        let barrier_clone = barrier.clone();

        // Thread 1: Hold lock briefly then release
        let handle = thread::spawn(move || {
            let _lock = SimulatorLock::try_acquire().expect("Thread 1 should acquire lock");
            barrier_clone.wait(); // Signal that lock is held
            thread::sleep(Duration::from_millis(50));
            // Lock released when _lock is dropped
        });

        // Wait for thread 1 to acquire lock
        barrier.wait();

        // Give thread 1 a moment to ensure lock is held
        thread::sleep(Duration::from_millis(10));

        // Thread 2: Retry until lock is available
        let lock = SimulatorLock::acquire_with_retry(10, Duration::from_millis(20))
            .expect("Should acquire lock after retry");

        handle.join().unwrap();
        drop(lock);
    }
}
