use super::Timestamp;
use crate::rpc::ActorId;
use carla_sys::carla_rust::client::FfiWorldSnapshot;
use cxx::UniquePtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Provides information for every actor at a certain moment of time,
/// Corresponds to [`carla.WorldSnapshot`](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.WorldSnapshot) in the Python API.
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

    pub fn timestamp(&self) -> &Timestamp {
        self.inner.GetTimestamp()
    }

    pub fn contains(&self, actor_id: ActorId) -> bool {
        self.inner.Contains(actor_id)
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiWorldSnapshot>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

assert_impl_all!(WorldSnapshot: Send);
