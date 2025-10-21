// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::Color;
use crate::sensor::SensorData;
use carla_sys::carla_rust::sensor::data::FfiImage;
use cxx::SharedPtr;
use derivative::Derivative;
use ndarray::ArrayView2;
use static_assertions::assert_impl_all;
use std::slice;

#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Image {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiImage>,
}

impl Image {
    pub fn height(&self) -> usize {
        self.inner.GetHeight()
    }

    pub fn width(&self) -> usize {
        self.inner.GetWidth()
    }

    pub fn len(&self) -> usize {
        self.inner.size()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn fov_angle(&self) -> f32 {
        self.inner.GetFOVAngle()
    }

    pub fn as_slice(&self) -> &[Color] {
        let len = self.len();
        let data = self.inner.data();
        unsafe { slice::from_raw_parts(data, len) }
    }

    pub fn as_array(&self) -> ArrayView2<'_, Color> {
        ArrayView2::from_shape((self.width(), self.height()), self.as_slice()).unwrap()
    }

    pub fn get(&self, index: usize) -> Option<&Color> {
        if index < self.inner.size() {
            Some(self.inner.at(index))
        } else {
            None
        }
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiImage>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl TryFrom<SensorData> for Image {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_image();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(Image: Send, Sync);
