//! Timestamp utilities for CARLA simulation.

/// Represents a timestamp in the CARLA simulation.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Timestamp {
    /// Frame number
    pub frame: u64,
    /// Elapsed simulation time in seconds
    pub elapsed_seconds: f64,
    /// Delta time since last frame in seconds
    pub delta_seconds: f64,
    /// Platform timestamp when this frame was received
    pub platform_timestamp: f64,
}

impl Timestamp {
    /// Create a new timestamp.
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

    /// Get the frame number.
    pub fn frame(&self) -> u64 {
        self.frame
    }

    /// Get elapsed simulation time in seconds.
    pub fn elapsed_seconds(&self) -> f64 {
        self.elapsed_seconds
    }

    /// Get delta time since last frame.
    pub fn delta_seconds(&self) -> f64 {
        self.delta_seconds
    }

    /// Get platform timestamp.
    pub fn platform_timestamp(&self) -> f64 {
        self.platform_timestamp
    }

    /// Get frames per second based on delta time.
    pub fn fps(&self) -> f64 {
        if self.delta_seconds > 0.0 {
            1.0 / self.delta_seconds
        } else {
            0.0
        }
    }

    /// Check if this timestamp is after another.
    pub fn is_after(&self, other: &Timestamp) -> bool {
        self.frame > other.frame
    }

    /// Check if this timestamp is before another.
    pub fn is_before(&self, other: &Timestamp) -> bool {
        self.frame < other.frame
    }
}

impl Default for Timestamp {
    fn default() -> Self {
        Self::new(0, 0.0, 0.0, 0.0)
    }
}

impl std::fmt::Display for Timestamp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Timestamp(frame={}, elapsed={:.3}s, delta={:.3}s)",
            self.frame, self.elapsed_seconds, self.delta_seconds
        )
    }
}

// Conversion to/from carla-sys types
impl From<carla_sys::Timestamp> for Timestamp {
    fn from(cxx_timestamp: carla_sys::Timestamp) -> Self {
        Self::new(
            cxx_timestamp.frame,
            cxx_timestamp.elapsed_seconds,
            cxx_timestamp.delta_seconds,
            cxx_timestamp.platform_timestamp,
        )
    }
}

impl From<Timestamp> for carla_sys::Timestamp {
    fn from(timestamp: Timestamp) -> Self {
        Self::new(
            timestamp.frame,
            timestamp.elapsed_seconds,
            timestamp.delta_seconds,
            timestamp.platform_timestamp,
        )
    }
}
