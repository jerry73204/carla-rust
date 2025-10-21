use crate::geom::BoundingBox;
use carla_sys::carla_rust::{client::FfiBoundingBoxList, geom::FfiBoundingBox};
use core::slice;
use cxx::UniquePtr;
use derivative::Derivative;

/// A list of bounding boxes.
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct BoundingBoxList {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiBoundingBoxList>,
}

impl BoundingBoxList {
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn get(&self, index: usize) -> Option<BoundingBox<f32>> {
        let orig = self.as_slice().get(index)?;
        Some(BoundingBox::from_native(orig))
    }

    pub fn iter(&self) -> impl Iterator<Item = BoundingBox<f32>> + '_ {
        self.as_slice().iter().map(BoundingBox::from_native)
    }

    fn as_slice(&self) -> &[FfiBoundingBox] {
        unsafe {
            // SAFETY: FfiBoundingBox is repr(C) and has the same layout as the underlying
            // C++ type. This is guaranteed by the FFI bindings. The data() method already
            // returns the correct pointer type.
            slice::from_raw_parts(self.inner.data(), self.len())
        }
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiBoundingBoxList>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}
