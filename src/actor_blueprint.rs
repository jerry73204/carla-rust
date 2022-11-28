use cxx::{let_cxx_string, UniquePtr};

use crate::ffi;

pub struct ActorBlueprint {
    pub(crate) inner: UniquePtr<ffi::ActorBlueprint>,
}

impl ActorBlueprint {
    pub fn contains_attribute(&self, id: &str) -> bool {
        let_cxx_string!(id = id);
        self.inner.ContainsAttribute(&id)
    }

    pub fn set_attribute(&mut self, id: &str, value: &str) -> bool {
        if !self.contains_attribute(id) {
            return false;
        }

        let_cxx_string!(id = id);
        let_cxx_string!(value = value);
        ffi::actor_bp_set_attribute(self.inner.pin_mut(), &id, &value);
        true
    }

    pub fn len(&self) -> usize {
        self.inner.size()
    }
}
