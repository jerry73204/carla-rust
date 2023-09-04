use carla_sys::carla_rust::client::FfiClientLightState;
use cxx::UniquePtr;
use derivative::Derivative;

#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct LightState {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: UniquePtr<FfiClientLightState>,
}

impl LightState {
    pub(crate) fn from_cxx(ptr: UniquePtr<FfiClientLightState>) -> Option<LightState> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}
