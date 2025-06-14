//! Time and timestamp functionality for CARLA simulator.
//!
//! This module provides time-related types including timestamps, durations,
//! and utilities for working with simulation time.

use std::time::Duration;

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

    /// Get the frame number
    pub fn frame(&self) -> u64 {
        self.frame
    }

    /// Get elapsed seconds since episode start
    pub fn elapsed_seconds(&self) -> f64 {
        self.elapsed_seconds
    }

    /// Get delta time since last frame
    pub fn delta_seconds(&self) -> f64 {
        self.delta_seconds
    }

    /// Get platform timestamp
    pub fn platform_timestamp(&self) -> f64 {
        self.platform_timestamp
    }

    /// Get elapsed time as Duration
    pub fn elapsed_duration(&self) -> Duration {
        Duration::from_secs_f64(self.elapsed_seconds)
    }

    /// Get delta time as Duration
    pub fn delta_duration(&self) -> Duration {
        Duration::from_secs_f64(self.delta_seconds)
    }

    /// Check if this timestamp is after another
    pub fn is_after(&self, other: &Timestamp) -> bool {
        self.frame > other.frame
    }

    /// Check if this timestamp is before another
    pub fn is_before(&self, other: &Timestamp) -> bool {
        self.frame < other.frame
    }

    /// Calculate the time difference between two timestamps
    pub fn time_since(&self, other: &Timestamp) -> f64 {
        self.elapsed_seconds - other.elapsed_seconds
    }

    /// Calculate the frame difference between two timestamps
    pub fn frames_since(&self, other: &Timestamp) -> i64 {
        self.frame as i64 - other.frame as i64
    }
}

impl Default for Timestamp {
    fn default() -> Self {
        Self::zero()
    }
}

impl std::fmt::Display for Timestamp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Timestamp(frame={}, elapsed_seconds={:.3}, delta_seconds={:.3}, platform_timestamp={:.3})",
            self.frame, self.elapsed_seconds, self.delta_seconds, self.platform_timestamp
        )
    }
}

/// Time duration for CARLA operations
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct TimeDuration {
    milliseconds: u64,
}

impl TimeDuration {
    /// Create a duration from seconds
    pub fn from_seconds(seconds: f64) -> Self {
        Self {
            milliseconds: (seconds * 1000.0) as u64,
        }
    }

    /// Create a duration from milliseconds
    pub fn from_milliseconds(milliseconds: u64) -> Self {
        Self { milliseconds }
    }

    /// Get the duration in seconds
    pub fn as_seconds(&self) -> f64 {
        self.milliseconds as f64 / 1000.0
    }

    /// Get the duration in milliseconds
    pub fn as_milliseconds(&self) -> u64 {
        self.milliseconds
    }

    /// Convert to std::time::Duration
    pub fn to_std_duration(&self) -> Duration {
        Duration::from_millis(self.milliseconds)
    }

    /// Create from std::time::Duration
    pub fn from_std_duration(duration: Duration) -> Self {
        Self {
            milliseconds: duration.as_millis() as u64,
        }
    }
}

impl From<Duration> for TimeDuration {
    fn from(duration: Duration) -> Self {
        Self::from_std_duration(duration)
    }
}

impl From<TimeDuration> for Duration {
    fn from(duration: TimeDuration) -> Self {
        duration.to_std_duration()
    }
}

impl Default for TimeDuration {
    fn default() -> Self {
        Self { milliseconds: 0 }
    }
}

/// Simulation clock for tracking time
pub struct SimulationClock {
    start_timestamp: Timestamp,
    current_timestamp: Timestamp,
    time_scale: f64,
    paused: bool,
}

impl SimulationClock {
    /// Create a new simulation clock
    pub fn new() -> Self {
        Self {
            start_timestamp: Timestamp::zero(),
            current_timestamp: Timestamp::zero(),
            time_scale: 1.0,
            paused: false,
        }
    }

    /// Update the clock with a new timestamp
    pub fn update(&mut self, timestamp: Timestamp) {
        if self.start_timestamp.frame == 0 {
            self.start_timestamp = timestamp;
        }
        self.current_timestamp = timestamp;
    }

    /// Get the current timestamp
    pub fn current(&self) -> &Timestamp {
        &self.current_timestamp
    }

    /// Get the start timestamp
    pub fn start(&self) -> &Timestamp {
        &self.start_timestamp
    }

    /// Get total elapsed time since start
    pub fn total_elapsed(&self) -> f64 {
        self.current_timestamp.elapsed_seconds - self.start_timestamp.elapsed_seconds
    }

    /// Get total frames since start
    pub fn total_frames(&self) -> u64 {
        self.current_timestamp.frame - self.start_timestamp.frame
    }

    /// Get average FPS since start
    pub fn average_fps(&self) -> f64 {
        let elapsed = self.total_elapsed();
        if elapsed > 0.0 {
            self.total_frames() as f64 / elapsed
        } else {
            0.0
        }
    }

    /// Get current FPS based on delta time
    pub fn current_fps(&self) -> f64 {
        if self.current_timestamp.delta_seconds > 0.0 {
            1.0 / self.current_timestamp.delta_seconds
        } else {
            0.0
        }
    }

    /// Set time scale (for slow motion or fast forward)
    pub fn set_time_scale(&mut self, scale: f64) {
        self.time_scale = scale.max(0.0);
    }

    /// Get current time scale
    pub fn time_scale(&self) -> f64 {
        self.time_scale
    }

    /// Pause the clock
    pub fn pause(&mut self) {
        self.paused = true;
    }

    /// Resume the clock
    pub fn resume(&mut self) {
        self.paused = false;
    }

    /// Check if paused
    pub fn is_paused(&self) -> bool {
        self.paused
    }

    /// Reset the clock
    pub fn reset(&mut self) {
        self.start_timestamp = Timestamp::zero();
        self.current_timestamp = Timestamp::zero();
        self.time_scale = 1.0;
        self.paused = false;
    }
}

impl Default for SimulationClock {
    fn default() -> Self {
        Self::new()
    }
}

/// Time utilities
pub mod utils {
    use super::*;

    /// Format duration as human-readable string
    pub fn format_duration(duration: f64) -> String {
        if duration < 1.0 {
            format!("{:.0}ms", duration * 1000.0)
        } else if duration < 60.0 {
            format!("{:.1}s", duration)
        } else if duration < 3600.0 {
            let minutes = (duration / 60.0) as u32;
            let seconds = duration % 60.0;
            format!("{}m {:.1}s", minutes, seconds)
        } else {
            let hours = (duration / 3600.0) as u32;
            let minutes = ((duration % 3600.0) / 60.0) as u32;
            let seconds = duration % 60.0;
            format!("{}h {}m {:.1}s", hours, minutes, seconds)
        }
    }

    /// Calculate frame time for target FPS
    pub fn frame_time_for_fps(target_fps: f64) -> Duration {
        if target_fps > 0.0 {
            Duration::from_secs_f64(1.0 / target_fps)
        } else {
            Duration::from_secs(0)
        }
    }

    /// Sleep for remaining frame time to maintain target FPS
    pub fn sleep_for_target_fps(
        frame_start: std::time::Instant,
        target_fps: f64,
    ) -> Option<Duration> {
        let target_frame_time = frame_time_for_fps(target_fps);
        let elapsed = frame_start.elapsed();

        if elapsed < target_frame_time {
            let sleep_duration = target_frame_time - elapsed;
            std::thread::sleep(sleep_duration);
            Some(sleep_duration)
        } else {
            None
        }
    }

    /// Create a rate limiter for callbacks
    pub struct RateLimiter {
        last_time: Option<std::time::Instant>,
        min_interval: Duration,
    }

    impl RateLimiter {
        /// Create a new rate limiter with target FPS
        pub fn new(max_fps: f64) -> Self {
            Self {
                last_time: None,
                min_interval: frame_time_for_fps(max_fps),
            }
        }

        /// Check if enough time has passed
        pub fn should_run(&mut self) -> bool {
            let now = std::time::Instant::now();

            if let Some(last) = self.last_time {
                if now.duration_since(last) >= self.min_interval {
                    self.last_time = Some(now);
                    true
                } else {
                    false
                }
            } else {
                self.last_time = Some(now);
                true
            }
        }

        /// Get time until next allowed run
        pub fn time_until_next(&self) -> Option<Duration> {
            if let Some(last) = self.last_time {
                let elapsed = last.elapsed();
                if elapsed < self.min_interval {
                    Some(self.min_interval - elapsed)
                } else {
                    Some(Duration::ZERO)
                }
            } else {
                Some(Duration::ZERO)
            }
        }
    }
}

impl From<crate::ffi::SimpleTimestamp> for Timestamp {
    fn from(simple: crate::ffi::SimpleTimestamp) -> Self {
        Timestamp::new(
            simple.frame,
            simple.elapsed_seconds,
            simple.delta_seconds,
            simple.platform_timestamp,
        )
    }
}

impl From<Timestamp> for crate::ffi::SimpleTimestamp {
    fn from(timestamp: Timestamp) -> Self {
        crate::ffi::SimpleTimestamp {
            frame: timestamp.frame,
            elapsed_seconds: timestamp.elapsed_seconds,
            delta_seconds: timestamp.delta_seconds,
            platform_timestamp: timestamp.platform_timestamp,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_timestamp_creation() {
        let ts = Timestamp::new(100, 10.5, 0.016, 1234567890.0);
        assert_eq!(ts.frame(), 100);
        assert_eq!(ts.elapsed_seconds(), 10.5);
        assert_eq!(ts.delta_seconds(), 0.016);
    }

    #[test]
    fn test_timestamp_comparison() {
        let ts1 = Timestamp::new(100, 10.0, 0.016, 0.0);
        let ts2 = Timestamp::new(200, 20.0, 0.016, 0.0);

        assert!(ts2.is_after(&ts1));
        assert!(ts1.is_before(&ts2));
        assert_eq!(ts2.time_since(&ts1), 10.0);
        assert_eq!(ts2.frames_since(&ts1), 100);
    }

    #[test]
    fn test_time_duration() {
        let duration = TimeDuration::from_seconds(1.5);
        assert_eq!(duration.as_milliseconds(), 1500);
        assert_eq!(duration.as_seconds(), 1.5);

        let std_duration = duration.to_std_duration();
        assert_eq!(std_duration.as_millis(), 1500);
    }

    #[test]
    fn test_simulation_clock() {
        let mut clock = SimulationClock::new();

        clock.update(Timestamp::new(0, 0.0, 0.0, 0.0));
        clock.update(Timestamp::new(60, 1.0, 0.016667, 1.0));

        assert_eq!(clock.total_frames(), 60);
        assert_eq!(clock.total_elapsed(), 1.0);
        assert_eq!(clock.average_fps(), 60.0);
    }

    #[test]
    fn test_format_duration() {
        use utils::format_duration;

        assert_eq!(format_duration(0.5), "500ms");
        assert_eq!(format_duration(1.5), "1.5s");
        assert_eq!(format_duration(65.5), "1m 5.5s");
        assert_eq!(format_duration(3665.5), "1h 1m 5.5s");
    }
}
