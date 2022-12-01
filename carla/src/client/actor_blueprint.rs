use carla_sys::carla::client::ActorBlueprint as FfiActorBlueprint;
use cxx::{let_cxx_string, UniquePtr};

#[repr(transparent)]
pub struct ActorBlueprint {
    pub(crate) inner: UniquePtr<FfiActorBlueprint>,
}

impl ActorBlueprint {
    pub(crate) fn from_cxx(ptr: UniquePtr<FfiActorBlueprint>) -> Self {
        Self { inner: ptr }
    }

    pub fn contains_attribute(&self, id: &str) -> bool {
        let_cxx_string!(id = id);
        self.inner.ContainsAttribute(&id)
    }

    pub fn set_attribute(&mut self, id: &str, value: &str) -> bool {
        if !self.contains_attribute(id) {
            return false;
        }
        let_cxx_string!(id = id);
        self.inner.pin_mut().SetAttribute(&id, value);
        true
    }

    pub fn len(&self) -> usize {
        self.inner.size()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}
