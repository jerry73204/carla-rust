//! Sensor data streaming functionality for CARLA.
//!
//! This module provides infrastructure for asynchronous sensor data streaming,
//! including callback management, data buffering, and synchronization utilities.

use crate::sensor::SensorData;
use anyhow::Result;
use std::{
    collections::HashMap,
    sync::{
        atomic::{AtomicBool, AtomicU64, Ordering},
        Arc, Mutex, RwLock,
    },
    thread,
    time::{Duration, Instant},
};

/// Stream ID type
pub type StreamId = u64;

/// Stream priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum StreamPriority {
    /// Low priority - may drop frames under load
    Low = 0,
    /// Normal priority - balanced performance
    Normal = 1,
    /// High priority - minimize latency
    High = 2,
}

/// Sensor stream configuration
#[derive(Debug, Clone)]
pub struct StreamConfig {
    /// Buffer size for queued sensor data
    pub buffer_size: usize,
    /// Whether to drop old data when buffer is full
    pub drop_on_overflow: bool,
    /// Priority level for stream processing
    pub priority: StreamPriority,
    /// Optional frame rate limit (Hz)
    pub max_fps: Option<f32>,
    /// Whether to enable time synchronization
    pub enable_sync: bool,
}

impl Default for StreamConfig {
    fn default() -> Self {
        Self {
            buffer_size: 10,
            drop_on_overflow: true,
            priority: StreamPriority::Normal,
            max_fps: None,
            enable_sync: false,
        }
    }
}

/// Stream statistics
#[derive(Debug, Clone, Default)]
pub struct StreamStats {
    /// Total frames received
    pub frames_received: u64,
    /// Frames dropped due to buffer overflow
    pub frames_dropped: u64,
    /// Average processing time per frame
    pub avg_processing_time_ms: f64,
    /// Current buffer utilization (0.0 - 1.0)
    pub buffer_utilization: f32,
    /// Current frame rate
    pub current_fps: f32,
}

/// Individual sensor stream
struct SensorStream {
    config: StreamConfig,
    callback: Box<dyn FnMut(SensorData) + Send>,
    buffer: Vec<(Instant, SensorData)>,
    stats: StreamStats,
    last_frame_time: Option<Instant>,
    processing_times: Vec<Duration>,
}

/// Sensor streaming manager
pub struct StreamingManager {
    streams: Arc<RwLock<HashMap<StreamId, Arc<Mutex<SensorStream>>>>>,
    next_stream_id: AtomicU64,
    running: Arc<AtomicBool>,
    worker_handle: Option<thread::JoinHandle<()>>,
}

impl StreamingManager {
    /// Create a new streaming manager
    pub fn new() -> Self {
        let manager = Self {
            streams: Arc::new(RwLock::new(HashMap::new())),
            next_stream_id: AtomicU64::new(1),
            running: Arc::new(AtomicBool::new(false)),
            worker_handle: None,
        };

        manager
    }

    /// Start the streaming manager
    pub fn start(&mut self) -> Result<()> {
        if self.running.load(Ordering::Relaxed) {
            return Ok(());
        }

        self.running.store(true, Ordering::Relaxed);

        let streams = Arc::clone(&self.streams);
        let running = Arc::clone(&self.running);

        let handle = thread::spawn(move || {
            while running.load(Ordering::Relaxed) {
                // Process streams
                Self::process_streams(&streams);

                // Small sleep to prevent busy waiting
                thread::sleep(Duration::from_micros(100));
            }
        });

        self.worker_handle = Some(handle);
        Ok(())
    }

    /// Stop the streaming manager
    pub fn stop(&mut self) -> Result<()> {
        self.running.store(false, Ordering::Relaxed);

        if let Some(handle) = self.worker_handle.take() {
            handle
                .join()
                .map_err(|_| anyhow::anyhow!("Worker thread panicked"))?;
        }

        Ok(())
    }

    /// Register a new sensor stream
    pub fn register_stream<F>(&self, config: StreamConfig, callback: F) -> Result<StreamId>
    where
        F: FnMut(SensorData) + Send + 'static,
    {
        let stream_id = self.next_stream_id.fetch_add(1, Ordering::Relaxed);

        let stream = SensorStream {
            config: config.clone(),
            callback: Box::new(callback),
            buffer: Vec::with_capacity(config.buffer_size),
            stats: StreamStats::default(),
            last_frame_time: None,
            processing_times: Vec::with_capacity(100),
        };

        self.streams
            .write()
            .unwrap()
            .insert(stream_id, Arc::new(Mutex::new(stream)));

        Ok(stream_id)
    }

    /// Unregister a sensor stream
    pub fn unregister_stream(&self, stream_id: StreamId) -> Result<()> {
        self.streams
            .write()
            .unwrap()
            .remove(&stream_id)
            .ok_or_else(|| anyhow::anyhow!("Stream not found"))?;

        Ok(())
    }

    /// Push sensor data to a stream
    pub fn push_data(&self, stream_id: StreamId, data: SensorData) -> Result<()> {
        let streams = self.streams.read().unwrap();
        let stream = streams
            .get(&stream_id)
            .ok_or_else(|| anyhow::anyhow!("Stream not found"))?;

        let mut stream_lock = stream.lock().unwrap();

        // Update stats
        stream_lock.stats.frames_received += 1;

        // Check frame rate limit
        if let Some(max_fps) = stream_lock.config.max_fps {
            if let Some(last_time) = stream_lock.last_frame_time {
                let elapsed = last_time.elapsed();
                let min_interval = Duration::from_secs_f32(1.0 / max_fps);
                if elapsed < min_interval {
                    return Ok(()); // Skip this frame
                }
            }
        }

        // Handle buffer overflow
        if stream_lock.buffer.len() >= stream_lock.config.buffer_size {
            if stream_lock.config.drop_on_overflow {
                stream_lock.buffer.remove(0);
                stream_lock.stats.frames_dropped += 1;
            } else {
                return Err(anyhow::anyhow!("Stream buffer full"));
            }
        }

        // Add data to buffer
        stream_lock.buffer.push((Instant::now(), data));
        stream_lock.last_frame_time = Some(Instant::now());

        // Update buffer utilization
        stream_lock.stats.buffer_utilization =
            stream_lock.buffer.len() as f32 / stream_lock.config.buffer_size as f32;

        Ok(())
    }

    /// Get stream statistics
    pub fn get_stats(&self, stream_id: StreamId) -> Result<StreamStats> {
        let streams = self.streams.read().unwrap();
        let stream = streams
            .get(&stream_id)
            .ok_or_else(|| anyhow::anyhow!("Stream not found"))?;

        let stream_lock = stream.lock().unwrap();
        Ok(stream_lock.stats.clone())
    }

    /// Process all active streams
    fn process_streams(streams: &Arc<RwLock<HashMap<StreamId, Arc<Mutex<SensorStream>>>>>) {
        let streams_read = streams.read().unwrap();

        // Sort streams by priority
        let mut stream_list: Vec<_> = streams_read.iter().collect();
        stream_list.sort_by_key(|(_, stream)| {
            let s = stream.lock().unwrap();
            std::cmp::Reverse(s.config.priority)
        });

        // Process each stream
        for (_, stream) in stream_list {
            let mut stream_lock = match stream.try_lock() {
                Ok(lock) => lock,
                Err(_) => continue, // Skip if locked
            };

            // Process buffered data
            while let Some((timestamp, data)) = stream_lock.buffer.pop() {
                let start_time = Instant::now();

                // Call the callback
                (stream_lock.callback)(data);

                // Update processing time stats
                let processing_time = start_time.elapsed();
                stream_lock.processing_times.push(processing_time);

                // Keep only last 100 processing times
                if stream_lock.processing_times.len() > 100 {
                    stream_lock.processing_times.remove(0);
                }

                // Update average processing time
                if !stream_lock.processing_times.is_empty() {
                    let total: Duration = stream_lock.processing_times.iter().sum();
                    stream_lock.stats.avg_processing_time_ms =
                        total.as_secs_f64() * 1000.0 / stream_lock.processing_times.len() as f64;
                }

                // Update FPS
                if let Some(last_time) = stream_lock.last_frame_time {
                    let elapsed = timestamp.saturating_duration_since(last_time);
                    if elapsed > Duration::ZERO {
                        stream_lock.stats.current_fps = 1.0 / elapsed.as_secs_f32();
                    }
                }
            }
        }
    }

    /// Create a synchronized stream group
    pub fn create_sync_group(&self, stream_ids: Vec<StreamId>) -> Result<SyncGroup> {
        // Verify all streams exist
        let streams = self.streams.read().unwrap();
        for id in &stream_ids {
            if !streams.contains_key(id) {
                return Err(anyhow::anyhow!("Stream {} not found", id));
            }
        }

        Ok(SyncGroup::new(stream_ids))
    }
}

impl Drop for StreamingManager {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

/// Synchronized stream group for multi-sensor synchronization
pub struct SyncGroup {
    stream_ids: Vec<StreamId>,
    sync_buffer: Arc<Mutex<HashMap<StreamId, Vec<(Instant, SensorData)>>>>,
    sync_threshold: Duration,
}

impl SyncGroup {
    /// Create a new sync group
    fn new(stream_ids: Vec<StreamId>) -> Self {
        let mut sync_buffer = HashMap::new();
        for id in &stream_ids {
            sync_buffer.insert(*id, Vec::new());
        }

        Self {
            stream_ids,
            sync_buffer: Arc::new(Mutex::new(sync_buffer)),
            sync_threshold: Duration::from_millis(10), // 10ms sync window
        }
    }

    /// Set synchronization threshold
    pub fn set_sync_threshold(&mut self, threshold: Duration) {
        self.sync_threshold = threshold;
    }

    /// Add data to sync group
    pub fn add_data(&self, stream_id: StreamId, data: SensorData) -> Result<Option<SyncedData>> {
        let mut buffer = self.sync_buffer.lock().unwrap();

        // Add data to stream's buffer
        if let Some(stream_buffer) = buffer.get_mut(&stream_id) {
            stream_buffer.push((Instant::now(), data));

            // Check if we have synchronized data
            if let Some(synced) = self.check_sync(&mut buffer) {
                return Ok(Some(synced));
            }
        }

        Ok(None)
    }

    /// Check for synchronized data across all streams
    fn check_sync(
        &self,
        buffer: &mut HashMap<StreamId, Vec<(Instant, SensorData)>>,
    ) -> Option<SyncedData> {
        // Find common timestamp window
        let mut min_timestamp = None;

        // Ensure all streams have data
        for id in &self.stream_ids {
            if let Some(stream_data) = buffer.get(id) {
                if stream_data.is_empty() {
                    return None;
                }

                let timestamp = stream_data[0].0;
                min_timestamp =
                    Some(min_timestamp.map_or(timestamp, |t: Instant| t.min(timestamp)));
            } else {
                return None;
            }
        }

        let base_time = min_timestamp?;

        // Collect synchronized data
        let mut synced_data = HashMap::new();

        for id in &self.stream_ids {
            if let Some(stream_data) = buffer.get_mut(id) {
                // Find data within sync threshold
                let mut found_idx = None;
                for (idx, (timestamp, _)) in stream_data.iter().enumerate() {
                    let diff = timestamp.saturating_duration_since(base_time);
                    if diff <= self.sync_threshold {
                        found_idx = Some(idx);
                        break;
                    }
                }

                if let Some(idx) = found_idx {
                    let (timestamp, data) = stream_data.remove(idx);
                    synced_data.insert(*id, (timestamp, data));
                } else {
                    return None; // No synchronized data for this stream
                }
            }
        }

        Some(SyncedData {
            timestamp: base_time,
            data: synced_data,
        })
    }
}

/// Synchronized sensor data from multiple streams
#[derive(Debug)]
pub struct SyncedData {
    /// Common timestamp for synchronized data
    pub timestamp: Instant,
    /// Map of stream ID to sensor data
    pub data: HashMap<StreamId, (Instant, SensorData)>,
}

/// Stream event for reactive processing
#[derive(Debug, Clone)]
pub enum StreamEvent {
    /// New data received
    DataReceived(StreamId),
    /// Stream buffer overflow
    BufferOverflow(StreamId),
    /// Stream error
    Error(StreamId, String),
}

/// Event-based stream processor
pub struct StreamProcessor {
    event_handlers: Arc<RwLock<Vec<Box<dyn Fn(StreamEvent) + Send + Sync>>>>,
}

impl StreamProcessor {
    /// Create a new stream processor
    pub fn new() -> Self {
        Self {
            event_handlers: Arc::new(RwLock::new(Vec::new())),
        }
    }

    /// Register an event handler
    pub fn on_event<F>(&self, handler: F)
    where
        F: Fn(StreamEvent) + Send + Sync + 'static,
    {
        self.event_handlers.write().unwrap().push(Box::new(handler));
    }

    /// Emit an event to all handlers
    pub fn emit(&self, event: StreamEvent) {
        let handlers = self.event_handlers.read().unwrap();
        for handler in handlers.iter() {
            handler(event.clone());
        }
    }
}

/// Utility functions for sensor data streaming
pub mod utils {
    use super::*;
    use std::{fs::File, io::Write, path::Path};

    /// Record sensor data to file
    pub struct DataRecorder {
        file: File,
        format: RecordFormat,
    }

    /// Recording format
    pub enum RecordFormat {
        /// Binary format for efficiency
        Binary,
        /// JSON format for readability
        Json,
        /// CSV format for analysis
        Csv,
    }

    impl DataRecorder {
        /// Create a new data recorder
        pub fn new<P: AsRef<Path>>(path: P, format: RecordFormat) -> Result<Self> {
            let file = File::create(path)?;
            Ok(Self { file, format })
        }

        /// Record sensor data
        pub fn record(&mut self, timestamp: Instant, data: &SensorData) -> Result<()> {
            match self.format {
                RecordFormat::Binary => {
                    // Implement binary serialization
                    todo!("Binary recording not yet implemented")
                }
                RecordFormat::Json => {
                    // Implement JSON serialization
                    let json = serde_json::json!({
                        "timestamp": timestamp.elapsed().as_secs_f64(),
                        "data": format!("{:?}", data), // Simplified for now
                    });
                    writeln!(self.file, "{}", json)?;
                }
                RecordFormat::Csv => {
                    // Implement CSV serialization based on data type
                    todo!("CSV recording not yet implemented")
                }
            }
            Ok(())
        }
    }

    /// Create a rate-limited stream
    pub fn rate_limit<F>(max_fps: f32, mut callback: F) -> impl FnMut(SensorData)
    where
        F: FnMut(SensorData),
    {
        let min_interval = Duration::from_secs_f32(1.0 / max_fps);
        let mut last_time = None;

        move |data| {
            let now = Instant::now();
            if let Some(last) = last_time {
                if now.duration_since(last) < min_interval {
                    return;
                }
            }
            last_time = Some(now);
            callback(data);
        }
    }

    /// Create a filtered stream
    pub fn filter<F, P>(mut predicate: P, mut callback: F) -> impl FnMut(SensorData)
    where
        F: FnMut(SensorData),
        P: FnMut(&SensorData) -> bool,
    {
        move |data| {
            if predicate(&data) {
                callback(data);
            }
        }
    }

    /// Create a transformed stream
    pub fn transform<F, T, R>(mut transformer: T, mut callback: F) -> impl FnMut(SensorData)
    where
        F: FnMut(R),
        T: FnMut(SensorData) -> Option<R>,
    {
        move |data| {
            if let Some(transformed) = transformer(data) {
                callback(transformed);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_streaming_manager() {
        let mut manager = StreamingManager::new();
        assert!(manager.start().is_ok());

        let stream_id = manager
            .register_stream(StreamConfig::default(), |_data| {
                // Process data
            })
            .unwrap();

        assert!(manager.unregister_stream(stream_id).is_ok());
        assert!(manager.stop().is_ok());
    }

    #[test]
    fn test_sync_group() {
        let group = SyncGroup::new(vec![1, 2, 3]);
        assert_eq!(group.stream_ids.len(), 3);
    }
}
