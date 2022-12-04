use super::RadarDetection;
use carla_sys::carla_rust::sensor::data::FfiRadarMeasurement;
use cxx::SharedPtr;
use std::slice;

#[derive(Clone)]
#[repr(transparent)]
pub struct RadarMeasurement {
    inner: SharedPtr<FfiRadarMeasurement>,
}

impl RadarMeasurement {
    pub fn detection_amount(&self) -> usize {
        self.inner.GetDetectionAmount()
    }

    pub fn as_slice(&self) -> &[RadarDetection] {
        unsafe { slice::from_raw_parts(self.inner.data(), self.inner.size()) }
    }

    pub fn len(&self) -> usize {
        self.inner.size()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}
