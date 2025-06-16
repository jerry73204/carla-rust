//! Dynamic Vision Sensor (DVS) implementations.

use crate::{geom::Transform, sensor::SensorData, time::Timestamp};

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
            // TODO: Extract proper metadata from carla-cxx DVS data structure
            // This requires adding timestamp, transform, and sensor_id fields to carla-cxx DVSEventArray
            timestamp: todo!(
                "DVSData::from_cxx timestamp extraction not yet implemented - missing FFI metadata"
            ),
            transform: todo!(
                "DVSData::from_cxx transform extraction not yet implemented - missing FFI metadata"
            ),
            sensor_id: todo!(
                "DVSData::from_cxx sensor_id extraction not yet implemented - missing FFI metadata"
            ),
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

    /// Generate accumulation image showing event density.
    pub fn generate_accumulation_image(&self) -> Vec<u32> {
        let mut accumulation = vec![0u32; (self.width * self.height) as usize];

        for event in &self.events {
            let idx = (event.y as u32 * self.width + event.x as u32) as usize;
            if idx < accumulation.len() {
                accumulation[idx] += 1;
            }
        }

        accumulation
    }

    /// Get event statistics.
    pub fn get_statistics(&self) -> DVSStatistics {
        if self.events.is_empty() {
            return DVSStatistics::default();
        }

        let positive_count = self.events.iter().filter(|e| e.pol).count();
        let negative_count = self.events.len() - positive_count;

        let min_time = self.events.iter().map(|e| e.t).min().unwrap_or(0);
        let max_time = self.events.iter().map(|e| e.t).max().unwrap_or(0);

        DVSStatistics {
            total_events: self.events.len(),
            positive_events: positive_count,
            negative_events: negative_count,
            time_span_ns: max_time - min_time,
            min_timestamp_ns: min_time,
            max_timestamp_ns: max_time,
            events_per_pixel: self.events.len() as f32 / (self.width * self.height) as f32,
        }
    }

    /// Convert to sparse event representation (useful for event-based algorithms).
    pub fn to_sparse_representation(&self) -> Vec<SparseEvent> {
        self.events
            .iter()
            .map(|e| SparseEvent {
                x: e.x,
                y: e.y,
                t: e.t,
                pol: if e.pol { 1.0 } else { -1.0 },
            })
            .collect()
    }
}

/// DVS sensor statistics.
#[derive(Debug, Clone, PartialEq)]
pub struct DVSStatistics {
    /// Total number of events
    pub total_events: usize,
    /// Number of positive events
    pub positive_events: usize,
    /// Number of negative events
    pub negative_events: usize,
    /// Time span of events in nanoseconds
    pub time_span_ns: i64,
    /// Minimum timestamp in nanoseconds
    pub min_timestamp_ns: i64,
    /// Maximum timestamp in nanoseconds
    pub max_timestamp_ns: i64,
    /// Average events per pixel
    pub events_per_pixel: f32,
}

impl Default for DVSStatistics {
    fn default() -> Self {
        Self {
            total_events: 0,
            positive_events: 0,
            negative_events: 0,
            time_span_ns: 0,
            min_timestamp_ns: 0,
            max_timestamp_ns: 0,
            events_per_pixel: 0.0,
        }
    }
}

/// Sparse event representation for event-based algorithms.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SparseEvent {
    /// X pixel coordinate
    pub x: u16,
    /// Y pixel coordinate
    pub y: u16,
    /// Timestamp in nanoseconds
    pub t: i64,
    /// Polarity as float (1.0 for positive, -1.0 for negative)
    pub pol: f32,
}
