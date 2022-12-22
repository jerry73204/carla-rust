use std::time::Duration;

pub use carla_sys::carla::client::Timestamp;

pub trait TimestampExt {
    fn elapsed_duration(&self) -> Duration;
    fn delta_duration(&self) -> Duration;
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
