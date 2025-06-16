//! Dynamic Vision Sensor (DVS) implementations.
//!
//! This module provides Rust bindings for CARLA's DVS camera sensor.
//! The API closely mirrors CARLA's C++ DVSEventArray and DVSEvent classes which provide:
//! - Base SensorData fields (timestamp, transform, sensor info)
//! - DVS events with pixel coordinates (x, y), timestamp, and polarity
//! - Image dimensions (width, height) and field of view
//! - Built-in visualization methods (ToImage, ToArray) in C++
//!
//! Additional methods provide convenient access to event data filtering and analysis
//! that builds upon the raw event data that CARLA provides.

use crate::{geom::Transform, sensor_data::SensorData, time::Timestamp};

/// Dynamic Vision Sensor (DVS) event array data.
#[derive(Debug, Clone)]
pub struct DVSData {
    /// Sensor timestamp
    pub timestamp: Timestamp,
    /// Sensor transform when captured
    pub transform: Transform,
    /// Sensor ID
    pub sensor_id: u32,
    /// Image width in pixels
    pub width: u32,
    /// Image height in pixels
    pub height: u32,
    /// Horizontal field of view angle in degrees
    pub fov_angle: f32,
    /// Array of DVS events
    pub events: Vec<DVSEvent>,
}

/// Individual DVS event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DVSEvent {
    /// X pixel coordinate
    pub x: u16,
    /// Y pixel coordinate
    pub y: u16,
    /// Timestamp in nanoseconds
    pub t: i64,
    /// Polarity (true=positive/increase, false=negative/decrease)
    pub pol: bool,
}

impl DVSEvent {
    /// Create a new DVS event.
    pub fn new(x: u16, y: u16, t: i64, pol: bool) -> Self {
        Self { x, y, t, pol }
    }

    /// Check if this is a positive event (brightness increase).
    pub fn is_positive(&self) -> bool {
        self.pol
    }

    /// Check if this is a negative event (brightness decrease).
    pub fn is_negative(&self) -> bool {
        !self.pol
    }

    /// Get the pixel coordinates as a tuple.
    pub fn pixel_coords(&self) -> (u16, u16) {
        (self.x, self.y)
    }
}

impl SensorData for DVSData {
    fn timestamp(&self) -> Timestamp {
        self.timestamp
    }

    fn transform(&self) -> Transform {
        self.transform
    }

    fn sensor_id(&self) -> u32 {
        self.sensor_id
    }

    fn size(&self) -> usize {
        self.events.len() * std::mem::size_of::<DVSEvent>()
    }
}

impl DVSData {
    /// Create DVSData from carla-cxx DVSEventArray
    pub fn from_cxx(cxx_data: carla_cxx::sensor::DVSEventArray) -> Self {
        let events = cxx_data
            .events
            .iter()
            .map(|e| DVSEvent::new(e.x, e.y, e.t, e.pol))
            .collect();

        Self {
            timestamp: Timestamp::new(
                cxx_data.timestamp.frame,
                cxx_data.timestamp.elapsed_seconds,
                cxx_data.timestamp.delta_seconds,
                cxx_data.timestamp.platform_timestamp,
            ),
            transform: Transform::new(
                crate::geom::Location::new(
                    cxx_data.transform.location.x,
                    cxx_data.transform.location.y,
                    cxx_data.transform.location.z,
                ),
                crate::geom::Rotation::new(
                    cxx_data.transform.rotation.pitch as f32,
                    cxx_data.transform.rotation.yaw as f32,
                    cxx_data.transform.rotation.roll as f32,
                ),
            ),
            sensor_id: cxx_data.sensor_id,
            width: cxx_data.width,
            height: cxx_data.height,
            fov_angle: cxx_data.fov_angle,
            events,
        }
    }

    /// Get image dimensions as a tuple.
    pub fn dimensions(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    /// Get the total number of events.
    pub fn event_count(&self) -> usize {
        self.events.len()
    }

    /// Filter events by polarity.
    pub fn filter_by_polarity(&self, positive: bool) -> Vec<DVSEvent> {
        self.events
            .iter()
            .filter(|e| e.pol == positive)
            .copied()
            .collect()
    }

    /// Get only positive events (brightness increases).
    pub fn get_positive_events(&self) -> Vec<DVSEvent> {
        self.filter_by_polarity(true)
    }

    /// Get only negative events (brightness decreases).
    pub fn get_negative_events(&self) -> Vec<DVSEvent> {
        self.filter_by_polarity(false)
    }

    /// Filter events by time range (in nanoseconds).
    pub fn filter_by_time_range(&self, start_ns: i64, end_ns: i64) -> Vec<DVSEvent> {
        self.events
            .iter()
            .filter(|e| e.t >= start_ns && e.t <= end_ns)
            .copied()
            .collect()
    }

    /// Filter events by spatial region.
    pub fn filter_by_region(&self, x1: u16, y1: u16, x2: u16, y2: u16) -> Vec<DVSEvent> {
        let min_x = x1.min(x2);
        let max_x = x1.max(x2);
        let min_y = y1.min(y2);
        let max_y = y1.max(y2);

        self.events
            .iter()
            .filter(|e| e.x >= min_x && e.x <= max_x && e.y >= min_y && e.y <= max_y)
            .copied()
            .collect()
    }

    /// Generate event frame by accumulating events over time.
    ///
    /// This mirrors CARLA C++ DVSEventArray::ToImage() functionality.
    /// Returns a grayscale image where positive events are white, negative are black.
    pub fn generate_event_frame(&self) -> Vec<u8> {
        let mut frame = vec![128u8; (self.width * self.height) as usize]; // Gray background

        for event in &self.events {
            let idx = (event.y as u32 * self.width + event.x as u32) as usize;
            if idx < frame.len() {
                frame[idx] = if event.pol { 255 } else { 0 }; // White for positive, black for negative
            }
        }

        frame
    }
}
