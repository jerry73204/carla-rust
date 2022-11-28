use crate::{ffi, ActorBlueprint};
use cxx::{let_cxx_string, UniquePtr};

pub struct BlueprintLibrary {
    pub(crate) inner: UniquePtr<ffi::SharedBlueprintLibrary>,
}

impl BlueprintLibrary {
    pub fn filter(&self, pattern: &str) -> Self {
        let_cxx_string!(pattern = pattern);

        Self {
            inner: ffi::bp_filter(&self.inner, &pattern),
        }
    }

    pub fn find(&self, key: &str) -> Option<ActorBlueprint> {
        let_cxx_string!(key = key);
        unsafe {
            // let bp_lib = ffi::bp_to_raw(&self.inner).as_ref().unwrap();
            // let actor_bp = bp_lib.find(&key).as_ref()?;
            let actor_bp = (*self.inner).as_ref().Find(&key).as_ref()?;
            let actor_bp = ffi::actor_bp_copy(actor_bp);
            Some(ActorBlueprint { inner: actor_bp })
        }
    }

    pub fn get(&self, index: usize) -> Option<ActorBlueprint> {
        if !(0..self.len()).contains(&index) {
            return None;
        }
        let actor_bp = (*self.inner).as_ref().at(index);
        let actor_bp = ffi::actor_bp_copy(actor_bp);
        Some(ActorBlueprint { inner: actor_bp })
    }

    pub fn len(&self) -> usize {
        (*self.inner).as_ref().size()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        (*self.inner).as_ref().empty()
    }
}

impl AsRef<ffi::BlueprintLibrary> for ffi::SharedBlueprintLibrary {
    fn as_ref(&self) -> &ffi::BlueprintLibrary {
        unsafe { ffi::bp_to_raw(self).as_ref().unwrap() }
    }
}
