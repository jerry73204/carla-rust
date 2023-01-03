use carla_sys::carla_rust::client::FfiLandmarkList;
use cxx::UniquePtr;
use derivative::Derivative;

use super::Landmark;

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
        Some(Landmark::from_cxx(ptr).unwrap())
    }

    pub fn iter(&self) -> impl Iterator<Item = Landmark> + '_ {
        (0..self.len()).map(|index| {
            let ptr = self.inner.get(index);
            Landmark::from_cxx(ptr).unwrap()
        })
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiLandmarkList>) -> Option<Self> {
        if ptr.is_null() {
            return None;
        }
        Some(Self { inner: ptr })
    }
}
