use crate::sensor::SensorData;

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
