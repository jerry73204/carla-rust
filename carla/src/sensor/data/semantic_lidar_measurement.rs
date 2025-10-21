use super::SemanticLidarDetection;
use crate::sensor::SensorData;
use carla_sys::carla_rust::sensor::data::FfiSemanticLidarMeasurement;
use cxx::SharedPtr;
use derivative::Derivative;
use ndarray::ArrayView2;
use static_assertions::assert_impl_all;
use std::slice;

#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct SemanticLidarMeasurement {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiSemanticLidarMeasurement>,
}

impl SemanticLidarMeasurement {
    pub fn horizontal_angle(&self) -> f32 {
        self.inner.GetHorizontalAngle()
    }

    pub fn point_count(&self, channel: usize) -> Option<usize> {
        (channel < self.channel_count()).then(|| self.inner.GetPointCount(channel) as usize)
    }

    pub fn channel_count(&self) -> usize {
        self.inner.GetChannelCount() as usize
    }

    pub fn as_slice(&self) -> &[SemanticLidarDetection] {
        let ptr = self.inner.data();
        let len = self.len();

        debug_assert!(
            !ptr.is_null(),
            "SemanticLidarMeasurement data pointer is null"
        );
        debug_assert!(
            (ptr as usize).is_multiple_of(std::mem::align_of::<SemanticLidarDetection>()),
            "SemanticLidarMeasurement data pointer not properly aligned"
        );

        unsafe { slice::from_raw_parts(ptr, len) }
    }

    pub fn as_array(&self) -> ArrayView2<'_, SemanticLidarDetection> {
        let len = self.len();
        let ih = self.channel_count();
        assert!(ih > 0, "Channel count is zero, cannot create array");
        let iw = len / ih;
        assert!(
            ih * iw == len,
            "Invalid dimensions: length {} not evenly divisible by channel count {}",
            len,
            ih
        );
        ArrayView2::from_shape((iw, ih), self.as_slice())
            .expect("Failed to create array view with valid dimensions")
    }

    pub fn len(&self) -> usize {
        self.inner.size()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiSemanticLidarMeasurement>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl TryFrom<SensorData> for SemanticLidarMeasurement {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_semantic_lidar_measurement();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(SemanticLidarMeasurement: Send, Sync);
