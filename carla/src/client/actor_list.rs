use crate::rpc::ActorId;
use carla_sys::carla_rust::client::FfiActorList;
use cxx::{let_cxx_string, SharedPtr};

use super::Actor;

#[derive(Clone)]
#[repr(transparent)]
pub struct ActorList {
    inner: SharedPtr<FfiActorList>,
}

impl ActorList {
    pub fn find(&self, actor_id: ActorId) -> Option<Actor> {
        let ptr = self.inner.Find(actor_id);
        Actor::from_cxx(ptr)
    }

    pub fn filter(&self, pattern: &str) -> Self {
        let_cxx_string!(pattern = pattern);
        let ptr = self.inner.Filter(&pattern);
        Self::from_cxx(ptr).unwrap()
    }

    pub fn get(&self, index: usize) -> Option<Actor> {
        let ptr = self.inner.at(index);
        Actor::from_cxx(ptr)
    }

    pub fn len(&self) -> usize {
        self.inner.size()
    }

    pub fn is_empty(&self) -> bool {
        self.inner.empty()
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiActorList>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}
