use std::time::Duration;

pub use carla_sys::carla::client::Timestamp;

/// Extension trait for [`Timestamp`] providing convenient duration conversions.
///
/// This trait provides methods to convert the various timestamp fields to Rust's
/// [`std::time::Duration`] type for easier time manipulation and comparison.
#[cfg_attr(
    carla_version_0916,
    doc = " See [carla.Timestamp](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Timestamp) in the Python API."
)]
#[cfg_attr(
    carla_version_0915,
    doc = " See [carla.Timestamp](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Timestamp) in the Python API."
)]
#[cfg_attr(
    carla_version_0914,
    doc = " See [carla.Timestamp](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Timestamp) in the Python API."
)]
///
/// # Examples
///
/// ```no_run
/// use carla::{client::Client, prelude::*};
///
/// let client = Client::default();
/// let world = client.world();
/// let snapshot = world.wait_for_tick();
/// let timestamp = snapshot.timestamp();
///
/// // Get elapsed time as Duration
/// let elapsed = timestamp.elapsed_duration();
/// println!("Simulation time: {:?}", elapsed);
///
/// // Get delta time for this frame
/// let delta = timestamp.delta_duration();
/// println!("Frame delta: {:?}", delta);
/// ```
pub trait TimestampExt {
    /// Converts `elapsed_seconds` to a [`Duration`].
    ///
    /// Returns the total elapsed simulation time since the beginning of the episode.
    fn elapsed_duration(&self) -> Duration;

    /// Converts `delta_seconds` to a [`Duration`].
    ///
    /// Returns the time elapsed since the previous tick. This is useful for
    /// frame-rate independent calculations.
    fn delta_duration(&self) -> Duration;

    /// Converts `platform_timestamp` to a [`Duration`].
    ///
    /// Returns the platform (OS) time when the frame was computed. This is
    /// useful for synchronizing with real-world time.
    fn platform_duration(&self) -> Duration;
}

impl TimestampExt for Timestamp {
    fn elapsed_duration(&self) -> Duration {
        Duration::from_secs_f64(self.elapsed_seconds)
    }

    fn delta_duration(&self) -> Duration {
        Duration::from_secs_f64(self.delta_seconds)
    }

    fn platform_duration(&self) -> Duration {
        Duration::from_secs_f64(self.platform_timestamp)
    }
}
