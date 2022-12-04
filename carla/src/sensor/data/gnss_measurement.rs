use crate::geom::GeoLocation;
use carla_sys::carla::sensor::data::GnssMeasurement as FfiGnssMeasurement;
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
}
