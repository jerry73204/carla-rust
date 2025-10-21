use super::RadarDetection;
use crate::sensor::SensorData;
use carla_sys::carla_rust::sensor::data::FfiRadarMeasurement;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;
use std::slice;

#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct RadarMeasurement {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiRadarMeasurement>,
}

impl RadarMeasurement {
    pub fn detection_amount(&self) -> usize {
        self.inner.GetDetectionAmount()
    }

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

assert_impl_all!(RadarMeasurement: Send, Sync);
