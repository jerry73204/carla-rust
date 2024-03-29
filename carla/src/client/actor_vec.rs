use super::Actor;
use carla_sys::carla_rust::client::FfiActorVec;
use cxx::UniquePtr;
use derivative::Derivative;

/// A vector containing a list of actors.
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct ActorVec {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiActorVec>,
}

impl ActorVec {
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn get(&self, index: usize) -> Option<Actor> {
        if index >= self.len() {
            return None;
        }

        let ptr = self.inner.get(index);
        Some(Actor::from_cxx(ptr).unwrap())
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiActorVec>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}
