//! Sensor data streaming implementation.

use crate::{
    actor::ActorId,
    error::{CarlaResult, SensorError},
    sensor_data::SensorData,
};
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

/// Sensor data stream for receiving data from multiple sensors.
#[derive(Debug)]
pub struct SensorStream {
    /// Active sensor subscriptions
    subscriptions: Arc<Mutex<HashMap<ActorId, SensorSubscription>>>,
    /// Stream configuration
    config: StreamConfig,
}

/// Configuration for sensor streams.
#[derive(Debug, Clone)]
pub struct StreamConfig {
    /// Buffer size for each sensor
    pub buffer_size: usize,
    /// Whether to drop old data when buffer is full
    pub drop_old_data: bool,
    /// Maximum number of concurrent sensors
    pub max_sensors: usize,
}

impl Default for StreamConfig {
    fn default() -> Self {
        Self {
            buffer_size: 10,
            drop_old_data: true,
            max_sensors: 100,
        }
    }
}

/// Subscription to a sensor data stream.
pub struct SensorSubscription {
    /// Sensor ID
    pub sensor_id: ActorId,
    /// Data buffer
    pub buffer: Vec<Box<dyn SensorData>>,
    /// Whether the subscription is active
    pub active: bool,
}

impl std::fmt::Debug for SensorSubscription {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SensorSubscription")
            .field("sensor_id", &self.sensor_id)
            .field("buffer_size", &self.buffer.len())
            .field("active", &self.active)
            .finish()
    }
}

impl SensorStream {
    /// Create a new sensor stream.
    pub fn new(config: StreamConfig) -> Self {
        Self {
            subscriptions: Arc::new(Mutex::new(HashMap::new())),
            config,
        }
    }

    /// Subscribe to a sensor's data stream.
    pub fn subscribe(&self, sensor_id: ActorId) -> CarlaResult<()> {
        let mut subscriptions = self.subscriptions.lock().unwrap();

        if subscriptions.len() >= self.config.max_sensors {
            return Err(SensorError::InvalidConfiguration(
                "Maximum number of sensor subscriptions reached".to_string(),
            )
            .into());
        }

        subscriptions.insert(
            sensor_id,
            SensorSubscription {
                sensor_id,
                buffer: Vec::with_capacity(self.config.buffer_size),
                active: true,
            },
        );

        Ok(())
    }

    /// Unsubscribe from a sensor's data stream.
    pub fn unsubscribe(&self, sensor_id: ActorId) -> CarlaResult<()> {
        let mut subscriptions = self.subscriptions.lock().unwrap();

        if subscriptions.remove(&sensor_id).is_none() {
            return Err(SensorError::NotListening(sensor_id).into());
        }

        Ok(())
    }

    /// Check if the latest data exists for a sensor.
    pub fn has_latest_data(&self, sensor_id: ActorId) -> bool {
        let subscriptions = self.subscriptions.lock().unwrap();

        if let Some(subscription) = subscriptions.get(&sensor_id) {
            !subscription.buffer.is_empty()
        } else {
            false
        }
    }

    /// Get the number of buffered items for a sensor.
    pub fn buffer_size(&self, sensor_id: ActorId) -> usize {
        let subscriptions = self.subscriptions.lock().unwrap();

        if let Some(subscription) = subscriptions.get(&sensor_id) {
            subscription.buffer.len()
        } else {
            0
        }
    }

    /// Clear the buffer for a specific sensor.
    pub fn clear_buffer(&self, sensor_id: ActorId) -> CarlaResult<()> {
        let mut subscriptions = self.subscriptions.lock().unwrap();

        if let Some(subscription) = subscriptions.get_mut(&sensor_id) {
            subscription.buffer.clear();
            Ok(())
        } else {
            Err(SensorError::NotListening(sensor_id).into())
        }
    }

    /// Get the number of active subscriptions.
    pub fn subscription_count(&self) -> usize {
        let subscriptions = self.subscriptions.lock().unwrap();
        subscriptions.len()
    }

    /// Check if a sensor is subscribed.
    pub fn is_subscribed(&self, sensor_id: ActorId) -> bool {
        let subscriptions = self.subscriptions.lock().unwrap();
        subscriptions.contains_key(&sensor_id)
    }

    /// Get list of all subscribed sensor IDs.
    pub fn subscribed_sensors(&self) -> Vec<ActorId> {
        let subscriptions = self.subscriptions.lock().unwrap();
        subscriptions.keys().copied().collect()
    }
}

/// Async sensor stream for tokio-based applications.
#[cfg(feature = "async")]
pub struct AsyncSensorStream {
    /// Async channels for each sensor
    channels: Arc<Mutex<HashMap<ActorId, tokio::sync::mpsc::Receiver<Box<dyn SensorData>>>>>,
    /// Stream configuration
    config: StreamConfig,
}

#[cfg(feature = "async")]
impl AsyncSensorStream {
    /// Create a new async sensor stream.
    pub fn new(config: StreamConfig) -> Self {
        Self {
            channels: Arc::new(Mutex::new(HashMap::new())),
            config,
        }
    }

    /// Subscribe to a sensor's data stream.
    pub async fn subscribe(
        &self,
        sensor_id: ActorId,
    ) -> CarlaResult<tokio::sync::mpsc::Receiver<Box<dyn SensorData>>> {
        let mut channels = self.channels.lock().unwrap();

        if channels.len() >= self.config.max_sensors {
            return Err(SensorError::InvalidConfiguration(
                "Maximum number of sensor subscriptions reached".to_string(),
            )
            .into());
        }

        let (tx, rx) = tokio::sync::mpsc::channel(self.config.buffer_size);

        // TODO: Connect tx to actual sensor data stream
        // This would be implemented when integrating with carla-cxx

        channels.insert(sensor_id, rx);

        Ok(channels.remove(&sensor_id).unwrap())
    }

    /// Unsubscribe from a sensor's data stream.
    pub async fn unsubscribe(&self, sensor_id: ActorId) -> CarlaResult<()> {
        let mut channels = self.channels.lock().unwrap();

        if channels.remove(&sensor_id).is_none() {
            return Err(SensorError::NotListening(sensor_id).into());
        }

        Ok(())
    }
}
