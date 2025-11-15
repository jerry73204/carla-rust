//! Sensor data types for processing sensor measurements.
//!
//! This module provides types for handling data from CARLA sensors. It corresponds
//! to the `carla.sensor` namespace in the Python API.
//!
//! # Key Types
//! - [`SensorData`] - Base type for all sensor data
//! - [`SensorDataBase`] - Trait providing common sensor data methods
//! - [`data`] - Specific sensor data types (images, LiDAR, collision, etc.)
//! - [`camera`] - Camera projection and coordinate transformation utilities
//!
//! # Sensor Data Types
//!
//! The [`data`] submodule contains specialized types for each sensor:
//!
//! ## Vision Sensors
//! - **RGB Camera**: [`data::Image`] - Color images
//! - **Depth Camera**: [`data::Image`] - Depth maps
//! - **Semantic Segmentation**: [`data::Image`] - Semantic class labels per pixel
//!
//! ## 3D Sensors
//! - **LiDAR**: [`data::LidarMeasurement`] - 3D point cloud data
//! - **Semantic LiDAR**: [`data::SemanticLidarMeasurement`] - Point cloud with semantic labels
//! - **Radar**: [`data::RadarMeasurement`] - Radar detection points
//!
//! ## Physics Sensors
//! - **Collision Detector**: [`data::CollisionEvent`] - Collision detection and impulse
//! - **Lane Invasion**: [`data::LaneInvasionEvent`] - Lane crossing detection
//! - **Obstacle Detector**: [`data::ObstacleDetectionEvent`] - Proximity detection
//!
//! ## Navigation Sensors
//! - **GNSS**: [`data::GnssMeasurement`] - GPS coordinates (latitude, longitude, altitude)
//! - **IMU**: [`data::ImuMeasurement`] - Accelerometer, gyroscope, compass
//!
//! ## Advanced Sensors
//! - **DVS Camera**: [`data::DVSEventArray`] - Event-based vision sensor
//! - **Optical Flow**: [`data::OpticalFlowImage`] - Motion vectors
//!
//! # Camera Utilities
//!
//! The [`camera`] module provides utilities for camera projection and sensor fusion:
//! - [`camera::build_projection_matrix`] - Build camera intrinsic matrix
//! - [`camera::world_to_camera`] - Transform world coordinates to camera space
//! - [`camera::project_to_2d`] - Project 3D points to 2D image coordinates
//!
//! # Python API Reference
//!
//! See the [carla.sensor](https://carla.readthedocs.io/en/0.9.16/python_api/#carla-sensor)
//! documentation for the Python equivalent types.
//!
//! # Examples
//!
//! ## RGB Camera
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

pub mod camera;
pub mod data;
