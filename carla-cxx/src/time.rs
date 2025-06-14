//! Time and timestamp functionality for CARLA simulator FFI.

use crate::ffi::SimpleTimestamp;

/// Timestamp information from CARLA simulator
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Timestamp {
    /// Number of frames elapsed since the simulator was launched
    pub frame: u64,
    /// Simulated seconds elapsed since the beginning of the current episode
    pub elapsed_seconds: f64,
    /// Simulated seconds elapsed since previous frame
    pub delta_seconds: f64,
    /// Time-stamp of the frame at which this measurement was taken, in seconds as given by the OS
    pub platform_timestamp: f64,
}

impl Timestamp {
    /// Create a new timestamp
    pub fn new(
        frame: u64,
        elapsed_seconds: f64,
        delta_seconds: f64,
        platform_timestamp: f64,
    ) -> Self {
        Self {
            frame,
            elapsed_seconds,
            delta_seconds,
            platform_timestamp,
        }
    }

    /// Create a zero/default timestamp
    pub fn zero() -> Self {
        Self {
            frame: 0,
            elapsed_seconds: 0.0,
            delta_seconds: 0.0,
            platform_timestamp: 0.0,
        }
    }
}

impl From<SimpleTimestamp> for Timestamp {
    fn from(simple: SimpleTimestamp) -> Self {
        Self {
            frame: simple.frame,
            elapsed_seconds: simple.elapsed_seconds,
            delta_seconds: simple.delta_seconds,
            platform_timestamp: simple.platform_timestamp,
        }
    }
}

impl From<Timestamp> for SimpleTimestamp {
    fn from(timestamp: Timestamp) -> Self {
        Self {
            frame: timestamp.frame,
            elapsed_seconds: timestamp.elapsed_seconds,
            delta_seconds: timestamp.delta_seconds,
            platform_timestamp: timestamp.platform_timestamp,
        }
    }
}
