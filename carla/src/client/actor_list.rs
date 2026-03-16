use crate::{error::ffi::with_ffi_error, rpc::ActorId};
use carla_sys::carla_rust::client::FfiActorList;
use cxx::{SharedPtr, let_cxx_string};
use derivative::Derivative;
use static_assertions::assert_impl_all;

use super::Actor;

/// A list of actors in the simulation.
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct ActorList {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiActorList>,
}

impl ActorList {
    pub fn find(&self, actor_id: ActorId) -> crate::Result<Option<Actor>> {
        let ptr = with_ffi_error("Find", |e| self.inner.Find(actor_id, e))?;
        Ok(Actor::from_cxx(ptr))
    }

    pub fn filter(&self, pattern: &str) -> crate::Result<Self> {
        let_cxx_string!(pattern = pattern);
        let ptr = with_ffi_error("Filter", |e| self.inner.Filter(&pattern, e))?;
        Ok(unsafe { Self::from_cxx(ptr).unwrap_unchecked() })
    }

    pub fn get(&self, index: usize) -> crate::Result<Option<Actor>> {
        let ptr = with_ffi_error("at", |e| self.inner.at(index, e))?;
        Ok(Actor::from_cxx(ptr))
    }

    pub fn len(&self) -> usize {
        self.inner.size()
    }

    pub fn is_empty(&self) -> bool {
        self.inner.empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = Actor> + Send {
        let list = self.clone();
        (0..self.len()).filter_map(move |idx| list.get(idx).ok().flatten())
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiActorList>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

assert_impl_all!(ActorList: Send, Sync);
