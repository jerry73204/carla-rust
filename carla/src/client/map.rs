use carla_sys::carla_rust::client::FfiMap;
use cxx::UniquePtr;

#[repr(transparent)]
pub struct Map {
    inner: UniquePtr<FfiMap>,
}

impl Map {
    pub fn from_cxx(ptr: UniquePtr<FfiMap>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}
