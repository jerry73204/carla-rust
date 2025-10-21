use core::slice;
use std::marker::PhantomData;

use carla_sys::carla_rust::client::FfiActorAttributeValueList;
use cxx::UniquePtr;
use derivative::Derivative;

use super::ActorAttributeValue;

/// The list of actor attribute values on an actor.
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct ActorAttributeValueList<'a> {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiActorAttributeValueList>,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> ActorAttributeValueList<'a> {
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn as_slice(&self) -> &[ActorAttributeValue] {
        unsafe {
            // SAFETY: ActorAttributeValue is repr(transparent) and has the same layout
            // as the underlying C++ type. This is guaranteed by the FFI bindings.
            slice::from_raw_parts(self.inner.data() as *const ActorAttributeValue, self.len())
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = &ActorAttributeValue> {
        self.as_slice().iter()
    }

    pub(crate) unsafe fn from_cxx(ptr: UniquePtr<FfiActorAttributeValueList>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self {
                inner: ptr,
                _phantom: PhantomData,
            })
        }
    }
}
