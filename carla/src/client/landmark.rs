use carla_sys::carla_rust::client::FfiLandmark;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Landmark {
    #[derivative(Debug = "ignore")]
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

assert_impl_all!(Landmark: Send, Sync);
