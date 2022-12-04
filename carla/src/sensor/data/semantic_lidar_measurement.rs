use super::SemanticLidarDetection;
use carla_sys::carla_rust::sensor::data::FfiSemanticLidarMeasurement;
use cxx::SharedPtr;
use ndarray::ArrayView2;
use std::slice;

#[derive(Clone)]
#[repr(transparent)]
pub struct SemanticLidarMeasurement {
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
        unsafe { slice::from_raw_parts(self.inner.data(), self.len()) }
    }

    pub fn as_array(&self) -> ArrayView2<'_, SemanticLidarDetection> {
        let len = self.len();
        let ih = self.channel_count();
        let iw = len / ih;
        assert!(ih * iw == len);
        ArrayView2::from_shape((iw, ih), self.as_slice()).unwrap()
    }

    pub fn len(&self) -> usize {
        self.inner.size()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl SemanticLidarMeasurement {
    pub(crate) fn from_cxx(ptr: SharedPtr<FfiSemanticLidarMeasurement>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}
