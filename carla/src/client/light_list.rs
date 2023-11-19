use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::client::FfiLightList;
use cxx::UniquePtr;
use derivative::Derivative;

use super::LightMut;

/// A list of lights.
#[derive(Derivative)]
#[repr(transparent)]
pub struct LightList {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: UniquePtr<FfiLightList>,
}

impl LightList {
    pub fn len(&self) -> usize {
        self.inner.size()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn get_mut(&mut self, index: usize) -> Option<LightMut<'_>> {
        if index >= self.len() {
            return None;
        }
        let ptr = self.inner.pin_mut().at(index).within_unique_ptr();
        unsafe { LightMut::from_cxx(ptr) }
    }

    pub fn from_cxx(ptr: UniquePtr<FfiLightList>) -> Option<Self> {
        if ptr.is_null() {
            return None;
        }

        Some(Self { inner: ptr })
    }
}
