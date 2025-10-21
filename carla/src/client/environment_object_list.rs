use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::client::FfiEnvironmentObjectList;
use cxx::UniquePtr;
use derivative::Derivative;

use crate::rpc::EnvironmentObjectRef;

/// A list of environment objects.
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct EnvironmentObjectList {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiEnvironmentObjectList>,
}

impl EnvironmentObjectList {
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn get(&self, index: usize) -> Option<EnvironmentObjectRef<'_>> {
        if self.len() >= index {
            return None;
        }

        Some(unsafe {
            let ptr = self.inner.get(index).within_unique_ptr();
            EnvironmentObjectRef::from_cxx(ptr).unwrap()
        })
    }
}

impl EnvironmentObjectList {
    pub(crate) fn from_cxx(ptr: UniquePtr<FfiEnvironmentObjectList>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}
