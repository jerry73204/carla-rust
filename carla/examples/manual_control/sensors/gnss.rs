//! GNSS (GPS) sensor
//!
//! Provides GPS coordinates (latitude, longitude).
//!
//! ## Phase 13 Implementation
//!
//! - Phase 5.2: GPS coordinate tracking
//! - Phase 5.2: Display in HUD with 6 decimal precision

use carla::{client::Sensor, rpc::AttachmentType, sensor::data::GnssMeasurement};
use eyre::{eyre, Result};
use std::sync::{Arc, Mutex};
use tracing::info;

/// GNSS sensor
///
/// Tracks GPS coordinates
#[allow(dead_code)]
pub struct GnssSensor {
    pub sensor: Option<Sensor>,
    data: Arc<Mutex<GnssData>>,
}

/// GNSS sensor data (shared with listener)
#[derive(Debug, Clone)]
struct GnssData {
    pub latitude: f64,
    pub longitude: f64,
}

impl GnssSensor {
    /// Create a new GNSS sensor
    pub fn new() -> Self {
        Self {
            sensor: None,
            data: Arc::new(Mutex::new(GnssData {
                latitude: 0.0,
                longitude: 0.0,
            })),
        }
    }

    /// Spawn the GNSS sensor
    ///
    /// ✅ Subphase 12.5.2: Spawn GNSS and set up listener
    pub fn spawn(&mut self, world: &mut crate::world::World) -> Result<()> {
        let player = world
            .player
            .as_ref()
            .ok_or_else(|| eyre!("No player vehicle available"))?;

        // Get blueprint for sensor.other.gnss
        let blueprint_library = world.world.blueprint_library();
        let gnss_bp = blueprint_library
            .find("sensor.other.gnss")
            .ok_or_else(|| eyre!("sensor.other.gnss blueprint not found"))?;

        info!("Spawning GNSS sensor");

        // Set sensor location (x=1.0, z=2.8)
        let transform = nalgebra::Isometry3::new(
            nalgebra::Vector3::new(1.0, 0.0, 2.8),
            nalgebra::Vector3::new(0.0, 0.0, 0.0),
        );

        // Spawn GNSS sensor attached to vehicle
        let gnss_actor = world
            .world
            .spawn_actor_opt(&gnss_bp, &transform, Some(player), AttachmentType::Rigid)
            .map_err(|e| eyre!("Failed to spawn GNSS sensor: {:?}", e))?;

        let gnss_sensor =
            Sensor::try_from(gnss_actor).map_err(|_| eyre!("Failed to convert to Sensor"))?;

        // Set up listener to receive GNSS data
        let data_clone = Arc::clone(&self.data);
        gnss_sensor.listen(move |sensor_data| {
            if let Ok(gnss_measurement) = GnssMeasurement::try_from(sensor_data) {
                let latitude = gnss_measurement.latitude();
                let longitude = gnss_measurement.longitude();

                // Update shared data
                if let Ok(mut data) = data_clone.lock() {
                    data.latitude = latitude;
                    data.longitude = longitude;
                }
            }
        });

        self.sensor = Some(gnss_sensor);
        info!("✓ GNSS sensor spawned and listening");

        Ok(())
    }

    /// Get latitude
    pub fn latitude(&self) -> f64 {
        self.data.lock().unwrap().latitude
    }

    /// Get longitude
    pub fn longitude(&self) -> f64 {
        self.data.lock().unwrap().longitude
    }
}

impl Default for GnssSensor {
    fn default() -> Self {
        Self::new()
    }
}
