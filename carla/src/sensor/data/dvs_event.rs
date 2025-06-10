use crate::sensor::{SensorData, SensorDataBase};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::{ptr, slice};

/// Represents a single DVS (Dynamic Vision Sensor) event.
#[derive(Clone, Debug)]
pub struct DvsEvent {
    /// X coordinate of the event
    pub x: u16,
    /// Y coordinate of the event
    pub y: u16,
    /// Timestamp of the event in microseconds
    pub t: u64,
    /// Polarity of the event (true for positive, false for negative)
    pub pol: bool,
}

/// Represents DVS event array data from a Dynamic Vision Sensor.
/// DVS sensors detect changes in brightness and generate events
/// instead of traditional image frames.
#[derive(Clone, Debug)]
pub struct DvsEventArray {
    inner: *mut carla_dvs_event_array_data_t,
}

impl DvsEventArray {
    /// Create a DvsEventArray from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_dvs_event_array_data_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null DVS event array data pointer"));
        }
        Ok(Self { inner: ptr })
    }

    /// Get the width of the DVS sensor that generated these events.
    pub fn width(&self) -> usize {
        unsafe { carla_dvs_get_width(self.inner) }
    }

    /// Get the height of the DVS sensor that generated these events.
    pub fn height(&self) -> usize {
        unsafe { carla_dvs_get_height(self.inner) }
    }

    /// Get the field of view angle of the DVS sensor.
    pub fn fov_angle(&self) -> f32 {
        unsafe { carla_dvs_get_fov_angle(self.inner) }
    }

    /// Get all DVS events as a slice.
    pub fn get_events(&self) -> &[DvsEvent] {
        let count = self.len();
        let events_ptr = unsafe { carla_dvs_get_events(self.inner) };

        if events_ptr.is_null() || count == 0 {
            &[]
        } else {
            // Convert from C struct array to Rust struct slice
            let c_events = unsafe { slice::from_raw_parts(events_ptr, count) };
            // This is safe because DvsEvent has the same memory layout as carla_dvs_event_t
            unsafe { slice::from_raw_parts(c_events.as_ptr() as *const DvsEvent, count) }
        }
    }

    /// Get the number of events in this array.
    pub fn len(&self) -> usize {
        unsafe { carla_dvs_get_event_count(self.inner) }
    }

    /// Check if the event array is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Analyze the events and return statistics.
    /// Returns (positive_count, negative_count, time_span_us).
    pub fn analyze_events(&self) -> (usize, usize, u64) {
        let mut positive_count = 0;
        let mut negative_count = 0;
        let mut time_span = 0;

        unsafe {
            carla_dvs_analyze_events(
                self.inner,
                &mut positive_count,
                &mut negative_count,
                &mut time_span,
            );
        }

        (positive_count, negative_count, time_span)
    }

    /// Filter events by polarity.
    /// Returns events that match the specified polarity.
    pub fn filter_events_by_polarity(&self, positive_polarity: bool) -> Vec<DvsEvent> {
        let mut filtered_count = 0;
        let filtered_events_ptr = unsafe {
            carla_dvs_filter_events_by_polarity(self.inner, positive_polarity, &mut filtered_count)
        };

        if filtered_events_ptr.is_null() || filtered_count == 0 {
            return Vec::new();
        }

        let c_events = unsafe { slice::from_raw_parts(filtered_events_ptr, filtered_count) };
        let mut events = Vec::with_capacity(filtered_count);

        for c_event in c_events {
            events.push(DvsEvent {
                x: c_event.x,
                y: c_event.y,
                t: c_event.t,
                pol: c_event.pol,
            });
        }

        // Free the filtered events array allocated by C
        unsafe {
            carla_dvs_free_filtered_events(filtered_events_ptr);
        }

        events
    }

    /// Filter events by time range.
    /// Returns events that occurred between start_time and end_time (microseconds).
    pub fn filter_events_by_time(&self, start_time_us: u64, end_time_us: u64) -> Vec<DvsEvent> {
        let events = self.get_events();
        events
            .iter()
            .filter(|event| event.t >= start_time_us && event.t <= end_time_us)
            .cloned()
            .collect()
    }

    /// Filter events by spatial region.
    /// Returns events that occurred within the specified rectangular region.
    pub fn filter_events_by_region(
        &self,
        x_min: u16,
        y_min: u16,
        x_max: u16,
        y_max: u16,
    ) -> Vec<DvsEvent> {
        let events = self.get_events();
        events
            .iter()
            .filter(|event| {
                event.x >= x_min && event.x <= x_max && event.y >= y_min && event.y <= y_max
            })
            .cloned()
            .collect()
    }

    /// Get the time range of events in this array.
    /// Returns (min_time_us, max_time_us).
    pub fn get_time_range(&self) -> Option<(u64, u64)> {
        let events = self.get_events();
        if events.is_empty() {
            return None;
        }

        let min_time = events.iter().map(|e| e.t).min().unwrap_or(0);
        let max_time = events.iter().map(|e| e.t).max().unwrap_or(0);
        Some((min_time, max_time))
    }
}

impl SensorDataBase for DvsEventArray {
    fn raw_ptr(&self) -> *mut carla_sensor_data_t {
        self.inner as *mut carla_sensor_data_t
    }
}

impl TryFrom<SensorData> for DvsEventArray {
    type Error = SensorData;

    fn try_from(value: SensorData) -> std::result::Result<Self, Self::Error> {
        // Check if the sensor data is actually DVS event data
        let data_type = value.data_type();
        if data_type == carla_sensor_data_type_t_CARLA_SENSOR_DATA_DVS_EVENTS {
            let dvs_ptr = unsafe { carla_sensor_data_as_dvs_events(value.inner) };
            if !dvs_ptr.is_null() {
                return Ok(DvsEventArray { inner: dvs_ptr });
            }
        }
        Err(value)
    }
}

impl Drop for DvsEventArray {
    fn drop(&mut self) {
        // Note: DVS event data is managed by the sensor data lifecycle
        // We don't need to free it separately
        self.inner = ptr::null_mut();
    }
}

// SAFETY: DvsEventArray wraps a thread-safe C API
unsafe impl Send for DvsEventArray {}
unsafe impl Sync for DvsEventArray {}

// Ensure DvsEvent has the same memory layout as carla_dvs_event_t
#[cfg(test)]
mod tests {
    use super::*;
    use std::mem;

    #[test]
    fn test_dvs_event_layout() {
        // Ensure the Rust struct has the same layout as the C struct
        assert_eq!(
            mem::size_of::<DvsEvent>(),
            mem::size_of::<carla_dvs_event_t>()
        );
        assert_eq!(
            mem::align_of::<DvsEvent>(),
            mem::align_of::<carla_dvs_event_t>()
        );
    }
}
