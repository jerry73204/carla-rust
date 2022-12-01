use carla_sys::carla::client::WorldSnapshot as FfiWorldSnapshot;
use cxx::UniquePtr;

pub struct WorldSnapshot {
    inner: UniquePtr<FfiWorldSnapshot>,
}

impl WorldSnapshot {
    pub fn id(&self) -> u64 {
        self.inner.GetId()
    }

    pub fn frame(&self) -> usize {
        self.inner.GetFrame()
    }

    // pub fn timestamp(&self) ->

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiWorldSnapshot>) -> Self {
        Self { inner: ptr }
    }
}
