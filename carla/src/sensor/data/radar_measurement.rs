use super::RadarDetection;
use crate::sensor::SensorData;
use carla_sys::carla_rust::sensor::data::FfiRadarMeasurement;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;
use std::slice;

/// Radar measurement data containing detected objects.
///
/// This type represents data from a radar sensor. Each detection includes the object's
/// position, velocity, azimuth, altitude, and depth relative to the sensor. Radar sensors
/// are useful for detecting moving objects and measuring their velocity.
///
/// Corresponds to [`carla.RadarMeasurement`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.RadarMeasurement) in the Python API.
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     client::{ActorBase, Client},
///     sensor::data::RadarMeasurement,
/// };
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// # let bp_lib = world.blueprint_library();
/// # let radar_bp = bp_lib.filter("sensor.other.radar").get(0).unwrap();
/// # let spawn_points = world.map().recommended_spawn_points();
/// # let radar = world.spawn_actor(&radar_bp, spawn_points.get(0).unwrap()).unwrap();
/// # let sensor: carla::client::Sensor = radar.try_into().unwrap();
///
/// sensor.listen(|sensor_data| {
///     if let Ok(radar_data) = RadarMeasurement::try_from(sensor_data) {
///         println!("Detected {} objects", radar_data.detection_amount());
///
///         // Access radar detections
///         for detection in radar_data.as_slice() {
///             println!(
///                 "Object: depth={:.2}m, velocity={:.2}m/s, azimuth={:.2}°",
///                 detection.depth, detection.velocity, detection.azimuth
///             );
///         }
///     }
/// });
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct RadarMeasurement {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiRadarMeasurement>,
}

impl RadarMeasurement {
    /// Returns the number of objects detected by the radar.
    ///
    /// See [carla.RadarMeasurement.get_detection_count](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.RadarMeasurement.get_detection_count)
    /// in the Python API.
    pub fn detection_amount(&self) -> usize {
        self.inner.GetDetectionAmount()
    }

    /// Returns the radar detections as a slice.
    ///
    /// This provides zero-copy access to all radar detections. Each detection contains
    /// information about position, velocity, and angles relative to the radar sensor.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # use carla::sensor::data::RadarMeasurement;
    /// # let client = Client::default();
    /// # let world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let radar_bp = bp_lib.filter("sensor.other.radar").get(0).unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let radar = world.spawn_actor(&radar_bp, spawn_points.get(0).unwrap()).unwrap();
    /// # let sensor: carla::client::Sensor = radar.try_into().unwrap();
    /// sensor.listen(|sensor_data| {
    ///     if let Ok(radar_data) = RadarMeasurement::try_from(sensor_data) {
    ///         let detections = radar_data.as_slice();
    ///         // Find closest object
    ///         if let Some(closest) = detections
    ///             .iter()
    ///             .min_by(|a, b| a.depth.partial_cmp(&b.depth).unwrap())
    ///         {
    ///             println!("Closest object at {:.2}m", closest.depth);
    ///         }
    ///     }
    /// });
    /// ```
    pub fn as_slice(&self) -> &[RadarDetection] {
        let ptr = self.inner.data();
        let len = self.inner.size();

        debug_assert!(!ptr.is_null(), "RadarMeasurement data pointer is null");
        debug_assert!(
            (ptr as usize).is_multiple_of(std::mem::align_of::<RadarDetection>()),
            "RadarMeasurement data pointer not properly aligned"
        );

        unsafe { slice::from_raw_parts(ptr, len) }
    }

    /// Returns the total number of detections.
    pub fn len(&self) -> usize {
        self.inner.size()
    }

    /// Returns true if no objects were detected.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiRadarMeasurement>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl TryFrom<SensorData> for RadarMeasurement {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_radar_measurement();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(RadarMeasurement: Send, Sync);
