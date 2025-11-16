use super::LidarDetection;
use crate::sensor::SensorData;
use carla_sys::carla_rust::sensor::data::FfiLidarMeasurement;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;
use std::slice;

/// LiDAR point cloud measurement data.
///
/// This type represents point cloud data from a LiDAR sensor. Each point contains
/// 3D coordinates and intensity information. LiDAR sensors are commonly used for
/// object detection, mapping, and localization in autonomous driving.
///
/// Corresponds to [`carla.LidarMeasurement`] in the Python API.
#[cfg_attr(carla_version_0916, doc = "")]
#[cfg_attr(
    carla_version_0916,
    doc = " [`carla.LidarMeasurement`]: https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LidarMeasurement"
)]
#[cfg_attr(carla_version_0915, doc = "")]
#[cfg_attr(
    carla_version_0915,
    doc = " [`carla.LidarMeasurement`]: https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LidarMeasurement"
)]
#[cfg_attr(carla_version_0914, doc = "")]
#[cfg_attr(
    carla_version_0914,
    doc = " [`carla.LidarMeasurement`]: https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LidarMeasurement"
)]
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     client::{ActorBase, Client},
///     sensor::data::LidarMeasurement,
/// };
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// # let bp_lib = world.blueprint_library();
/// # let lidar_bp = bp_lib.filter("sensor.lidar.ray_cast").get(0).unwrap();
/// # let spawn_points = world.map().recommended_spawn_points();
/// # let lidar = world.spawn_actor(&lidar_bp, spawn_points.get(0).unwrap()).unwrap();
/// # let sensor: carla::client::Sensor = lidar.try_into().unwrap();
///
/// sensor.listen(|sensor_data| {
///     if let Ok(lidar_data) = LidarMeasurement::try_from(sensor_data) {
///         println!("Received {} points", lidar_data.len());
///         println!("Horizontal angle: {}°", lidar_data.horizontal_angle());
///         println!("Channels: {}", lidar_data.channel_count());
///
///         // Access point cloud data
///         for point in lidar_data.as_slice().iter().take(10) {
///             println!(
///                 "Point: ({}, {}, {}) intensity={}",
///                 point.x, point.y, point.z, point.intensity
///             );
///         }
///     }
/// });
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct LidarMeasurement {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiLidarMeasurement>,
}

impl LidarMeasurement {
    /// Returns the horizontal angle of the LiDAR measurement in degrees.
    ///
    /// This represents the current rotation angle of the LiDAR sensor.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LidarMeasurement.horizontal_angle](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LidarMeasurement.horizontal_angle)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LidarMeasurement.horizontal_angle](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LidarMeasurement.horizontal_angle)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LidarMeasurement.horizontal_angle](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LidarMeasurement.horizontal_angle)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn horizontal_angle(&self) -> f32 {
        self.inner.GetHorizontalAngle()
    }

    /// Returns the number of points detected in the specified channel.
    ///
    /// Returns `None` if the channel index is out of bounds.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LidarMeasurement.get_point_count](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LidarMeasurement.get_point_count)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LidarMeasurement.get_point_count](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LidarMeasurement.get_point_count)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LidarMeasurement.get_point_count](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LidarMeasurement.get_point_count)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn point_count(&self, channel: usize) -> Option<usize> {
        (channel < self.channel_count()).then(|| self.inner.GetPointCount(channel) as usize)
    }

    /// Returns the number of channels (laser beams) in the LiDAR sensor.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LidarMeasurement.channels](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LidarMeasurement.channels)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LidarMeasurement.channels](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LidarMeasurement.channels)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LidarMeasurement.channels](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LidarMeasurement.channels)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn channel_count(&self) -> usize {
        self.inner.GetChannelCount() as usize
    }

    /// Returns the point cloud data as a slice of LidarDetection points.
    ///
    /// This provides zero-copy access to all detected points. Each point contains
    /// 3D coordinates (x, y, z) and intensity information.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # use carla::sensor::data::LidarMeasurement;
    /// # let client = Client::default();
    /// # let world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let lidar_bp = bp_lib.filter("sensor.lidar.ray_cast").get(0).unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let lidar = world.spawn_actor(&lidar_bp, spawn_points.get(0).unwrap()).unwrap();
    /// # let sensor: carla::client::Sensor = lidar.try_into().unwrap();
    /// sensor.listen(|sensor_data| {
    ///     if let Ok(lidar_data) = LidarMeasurement::try_from(sensor_data) {
    ///         let points = lidar_data.as_slice();
    ///         // Process point cloud
    ///         for point in points {
    ///             let distance = (point.x * point.x + point.y * point.y + point.z * point.z).sqrt();
    ///             println!("Point at distance: {:.2}m", distance);
    ///         }
    ///     }
    /// });
    /// ```
    pub fn as_slice(&self) -> &[LidarDetection] {
        let ptr = self.inner.data();
        let len = self.len();

        debug_assert!(!ptr.is_null(), "LidarMeasurement data pointer is null");
        debug_assert!(
            (ptr as usize).is_multiple_of(std::mem::align_of::<LidarDetection>()),
            "LidarMeasurement data pointer not properly aligned"
        );

        unsafe { slice::from_raw_parts(ptr, len) }
    }

    /// Returns the total number of detected points in the measurement.
    pub fn len(&self) -> usize {
        self.inner.size()
    }

    /// Returns true if no points were detected.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiLidarMeasurement>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl TryFrom<SensorData> for LidarMeasurement {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_lidar_measurement();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(LidarMeasurement: Send, Sync);
