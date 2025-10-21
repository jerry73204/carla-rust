use std::marker::PhantomData;

use carla_sys::carla_rust::rpc::FfiEnvironmentObjectRef;
use cxx::UniquePtr;
use derivative::Derivative;
use nalgebra::Isometry3;

use crate::geom::{BoundingBox, TransformExt};

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
    pub fn transform(&self) -> Isometry3<f32> {
        self.inner.transform().to_na()
    }

    pub fn bounding_box(&self) -> BoundingBox<f32> {
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
