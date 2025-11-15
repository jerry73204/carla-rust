use crate::{geom::GeoLocation, sensor::SensorData};
use carla_sys::carla_rust::sensor::data::FfiGnssMeasurement;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// GNSS (GPS) sensor measurement data.
///
/// This type represents geographic position data from a GNSS sensor. It provides
/// latitude, longitude, and altitude information in the WGS84 coordinate system.
///
/// Corresponds to [`carla.GnssMeasurement`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.GnssMeasurement) in the Python API.
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     client::{ActorBase, Client},
///     sensor::data::GnssMeasurement,
/// };
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// # let bp_lib = world.blueprint_library();
/// # let gnss_bp = bp_lib.filter("sensor.other.gnss").get(0).unwrap();
/// # let spawn_points = world.map().recommended_spawn_points();
/// # let gnss = world.spawn_actor(&gnss_bp, spawn_points.get(0).unwrap()).unwrap();
/// # let sensor: carla::client::Sensor = gnss.try_into().unwrap();
///
/// sensor.listen(|sensor_data| {
///     if let Ok(gnss_data) = GnssMeasurement::try_from(sensor_data) {
///         println!(
///             "Position: lat={:.6}°, lon={:.6}°, alt={:.2}m",
///             gnss_data.latitude(),
///             gnss_data.longitude(),
///             gnss_data.attitude()
///         );
///
///         let geo = gnss_data.geo_location();
///         println!("GeoLocation: {:?}", geo);
///     }
/// });
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct GnssMeasurement {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiGnssMeasurement>,
}

impl GnssMeasurement {
    /// Returns the geographic location as a GeoLocation struct.
    ///
    /// See [carla.GnssMeasurement.transform](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.GnssMeasurement.transform)
    /// in the Python API.
    pub fn geo_location(&self) -> GeoLocation {
        // SAFETY: carla::geom::GeoLocation and FfiGeoLocation have identical memory layout
        unsafe {
            let cpp_geo = self.inner.GetGeoLocation();
            GeoLocation::from_ffi(std::mem::transmute::<
                carla_sys::carla::geom::GeoLocation,
                crate::geom::FfiGeoLocation,
            >(cpp_geo))
        }
    }

    /// Returns the longitude in degrees (WGS84 coordinate system).
    ///
    /// See [carla.GnssMeasurement.longitude](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.GnssMeasurement.longitude)
    /// in the Python API.
    pub fn longitude(&self) -> f64 {
        self.inner.GetLongitude()
    }

    /// Returns the latitude in degrees (WGS84 coordinate system).
    ///
    /// See [carla.GnssMeasurement.latitude](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.GnssMeasurement.latitude)
    /// in the Python API.
    pub fn latitude(&self) -> f64 {
        self.inner.GetLatitude()
    }

    /// Returns the altitude in meters above sea level (WGS84 coordinate system).
    ///
    /// See [carla.GnssMeasurement.altitude](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.GnssMeasurement.altitude)
    /// in the Python API.
    pub fn attitude(&self) -> f64 {
        self.inner.GetAltitude()
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiGnssMeasurement>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl TryFrom<SensorData> for GnssMeasurement {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_gnss_measurement();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(GnssMeasurement: Send, Sync);
