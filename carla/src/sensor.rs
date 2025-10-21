//! Sensor data types for processing sensor measurements.
//!
//! This module provides types for handling data from CARLA sensors:
//! - [`SensorData`] - Base type for all sensor data
//! - [`SensorDataBase`] - Trait providing common sensor data methods
//! - [`data`] - Specific sensor data types (images, LiDAR, collision, etc.)
//!
//! # Sensor Data Types
//!
//! The [`data`] submodule contains specialized types for each sensor:
//! - **Cameras**: [`data::Image`] - RGB, depth, semantic segmentation
//! - **LiDAR**: [`data::LidarMeasurement`] - Point cloud data
//! - **Collision**: [`data::CollisionEvent`] - Collision detection
//! - **GNSS**: [`data::GnssMeasurement`] - GPS coordinates
//! - **IMU**: [`data::ImuMeasurement`] - Accelerometer and gyroscope
//! - **Lane Invasion**: [`data::LaneInvasionEvent`] - Lane crossing detection
//!
//! # Examples
//!
//! ```no_run
//! use carla::{
//!     client::Client,
//!     sensor::{data::Image, SensorDataBase},
//! };
//!
//! let client = Client::default();
//! let mut world = client.world();
//!
//! // Spawn a camera
//! let bp_lib = world.blueprint_library();
//! let camera_bp = bp_lib.filter("sensor.camera.rgb").get(0).unwrap();
//! let spawn_points = world.map().recommended_spawn_points();
//! let camera = world
//!     .spawn_actor(&camera_bp, &spawn_points.get(0).unwrap())
//!     .unwrap();
//! let sensor: carla::client::Sensor = camera.try_into().unwrap();
//!
//! // Listen for images
//! sensor.listen(|data| {
//!     if let Ok(image) = Image::try_from(data) {
//!         println!("Received {}x{} image", image.width(), image.height());
//!     }
//! });
//! ```

mod sensor_data;
pub use sensor_data::*;

pub mod data;
