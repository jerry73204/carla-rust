use core::slice;

use crate::rpc::LabelledPoint;
use carla_sys::carla_rust::client::FfiLabelledPointList;
use cxx::UniquePtr;
use derivative::Derivative;

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
