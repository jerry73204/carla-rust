use crate::ffi;
use std::time::Duration;

impl ffi::time_duration {
    pub fn to_std(&self) -> Duration {
        Duration::from_millis(self.milliseconds() as u64)
    }
}
