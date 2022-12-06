use super::LidarDetection;
use crate::sensor::SensorData;
use carla_sys::carla_rust::sensor::data::FfiLidarMeasurement;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;
use std::slice;

#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct LidarMeasurement {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiLidarMeasurement>,
}

impl LidarMeasurement {
    pub fn horizontal_angle(&self) -> f32 {
        self.inner.GetHorizontalAngle()
    }

    pub fn point_count(&self, channel: usize) -> Option<usize> {
        (channel < self.channel_count()).then(|| self.inner.GetPointCount(channel) as usize)
    }

    pub fn channel_count(&self) -> usize {
        self.inner.GetChannelCount() as usize
    }

    pub fn as_slice(&self) -> &[LidarDetection] {
        unsafe { slice::from_raw_parts(self.inner.data(), self.len()) }
    }

    pub fn len(&self) -> usize {
        self.inner.size()
    }

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
