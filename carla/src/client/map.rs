use carla_sys::carla_rust::client::FfiMap;
use cxx::UniquePtr;

#[repr(transparent)]
pub struct Map {
    pub(crate) inner: UniquePtr<FfiMap>,
}
