use std::marker::PhantomData;

use carla_sys::carla_rust::rpc::FfiEnvironmentObjectRef;
use cxx::UniquePtr;
use derivative::Derivative;

use crate::geom::{BoundingBox, Transform};

#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct EnvironmentObjectRef<'a> {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiEnvironmentObjectRef>,
    #[derivative(Debug = "ignore")]
    _phantom: PhantomData<&'a ()>,
}

impl<'a> EnvironmentObjectRef<'a> {
    pub fn id(&self) -> u64 {
        self.inner.id()
    }

    pub fn transform(&self) -> Transform {
        Transform::from_ffi(self.inner.transform().clone())
    }

    pub fn bounding_box(&self) -> BoundingBox {
        let bbox = self.inner.bounding_box();
        BoundingBox::from_native(&bbox)
    }

    pub(crate) unsafe fn from_cxx(ptr: UniquePtr<FfiEnvironmentObjectRef>) -> Option<Self> {
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
