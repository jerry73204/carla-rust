use carla_sys::carla_rust::client::FfiLandmark;
use cxx::SharedPtr;

#[derive(Clone)]
#[repr(transparent)]
pub struct Landmark {
    pub(crate) inner: SharedPtr<FfiLandmark>,
}

impl Landmark {
    pub(crate) fn from_cxx(ptr: SharedPtr<FfiLandmark>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}
