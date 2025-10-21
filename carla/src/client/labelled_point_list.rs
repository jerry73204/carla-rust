use core::slice;

use crate::rpc::LabelledPoint;
use carla_sys::carla_rust::client::FfiLabelledPointList;
use static_assertions::assert_impl_all;
use cxx::UniquePtr;
use derivative::Derivative;

/// A list of labeled points.
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct LabelledPointList {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiLabelledPointList>,
}

impl LabelledPointList {
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn as_slice(&self) -> &[LabelledPoint] {
        unsafe { slice::from_raw_parts(self.inner.data(), self.len()) }
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiLabelledPointList>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

assert_impl_all!(LabelledPointList: Send, Sync);
