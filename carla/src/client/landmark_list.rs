use carla_sys::carla_rust::client::FfiLandmarkList;
use cxx::UniquePtr;
use derivative::Derivative;

use super::Landmark;
use static_assertions::assert_impl_all;

/// A list of landmarks.
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct LandmarkList {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiLandmarkList>,
}

impl LandmarkList {
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn get(&self, index: usize) -> Option<Landmark> {
        if index > self.len() {
            return None;
        }
        let ptr = self.inner.get(index);
        Some(unsafe { Landmark::from_cxx(ptr).unwrap_unchecked() })
    }

    pub fn iter(&self) -> impl Iterator<Item = Landmark> + '_ {
        (0..self.len()).map(|index| {
            let ptr = self.inner.get(index);
            unsafe { Landmark::from_cxx(ptr).unwrap_unchecked() }
        })
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiLandmarkList>) -> Option<Self> {
        if ptr.is_null() {
            return None;
        }
        Some(Self { inner: ptr })
    }
}

assert_impl_all!(LandmarkList: Send, Sync);
