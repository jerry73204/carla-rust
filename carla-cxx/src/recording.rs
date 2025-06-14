//! Recording and playback utilities for CARLA simulator FFI.

/// Actor type filters for collision analysis
pub mod actor_types {
    /// All actor types
    pub const ALL: u8 = b'a';
    /// Vehicle actors only
    pub const VEHICLE: u8 = b'v';
    /// Walker (pedestrian) actors only  
    pub const WALKER: u8 = b'w';
    /// Hero vehicles only
    pub const HERO: u8 = b'h';
}

/// Default values for recording analysis
pub mod defaults {
    /// Default minimum time threshold for blocked actor analysis (30 seconds)
    pub const MIN_TIME_BLOCKED: f64 = 30.0;
    /// Default minimum distance threshold for blocked actor analysis (10 meters)
    pub const MIN_DISTANCE_BLOCKED: f64 = 10.0;
    /// Default replay speed (normal speed)
    pub const NORMAL_SPEED: f64 = 1.0;
}

/// Playback control helper functions
impl crate::ffi::bridge::SimpleRecorderInfo {
    /// Create a default recorder info structure
    pub fn new() -> Self {
        Self {
            version: 0,
            magic: String::new(),
            date: 0,
            mapfile: String::new(),
            frame_count: 0,
            duration_seconds: 0.0,
            total_size_mb: 0.0,
        }
    }
}

impl Default for crate::ffi::bridge::SimpleRecorderInfo {
    fn default() -> Self {
        Self::new()
    }
}
