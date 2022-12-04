use crate::{geom::GeoLocation, sensor::SensorData};
use carla_sys::carla_rust::sensor::data::FfiGnssMeasurement;
use cxx::SharedPtr;

#[derive(Clone)]
#[repr(transparent)]
pub struct GnssMeasurement {
    inner: SharedPtr<FfiGnssMeasurement>,
}

impl GnssMeasurement {
    pub fn geo_location(&self) -> GeoLocation {
        self.inner.GetGeoLocation()
    }

    pub fn longitude(&self) -> f64 {
        self.inner.GetLongitude()
    }

    pub fn latitude(&self) -> f64 {
        self.inner.GetLatitude()
    }

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
