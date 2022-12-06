use carla_sys::carla::client::WorldSnapshot as FfiWorldSnapshot;
use cxx::UniquePtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct WorldSnapshot {
    #[derivative(Debug = "ignore")]
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

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiWorldSnapshot>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

assert_impl_all!(WorldSnapshot: Send);
