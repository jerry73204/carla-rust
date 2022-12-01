use carla_sys::carla::client::ActorBlueprint as FfiActorBlueprint;
use cxx::{let_cxx_string, UniquePtr};

#[repr(transparent)]
pub struct ActorBlueprint {
    pub(crate) inner: UniquePtr<FfiActorBlueprint>,
}

impl ActorBlueprint {
    pub fn id(&self) -> String {
        self.inner.GetId().to_string()
    }

    pub fn contains_tag(&self, tag: &str) -> bool {
        let_cxx_string!(tag = tag);
        self.inner.ContainsTag(&tag)
    }

    pub fn match_tags(&self, pattern: &str) -> bool {
        let_cxx_string!(pattern = pattern);
        self.inner.MatchTags(&pattern)
    }

    pub fn tags(&self) -> Vec<String> {
        self.inner
            .GetTags()
            .iter()
            .map(|tag| tag.to_string())
            .collect()
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

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiActorBlueprint>) -> Self {
        Self { inner: ptr }
    }
}
