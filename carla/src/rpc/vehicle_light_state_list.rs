use super::ActorId;
use crate::rpc::VehicleLightState;
use carla_sys::carla_rust::rpc::{FfiVehicleLightStateElementRef, FfiVehicleLightStateList};
use cxx::UniquePtr;
use derivative::Derivative;
use std::marker::PhantomData;

#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct VehicleLightStateList {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiVehicleLightStateList>,
}

impl VehicleLightStateList {
    pub fn len(&self) -> usize {
        self.inner.len()
    }

    pub fn get(&self, index: usize) -> Option<VehicleLightStateListElement<'_>> {
        if index >= self.len() {
            return None;
        }
        Some(unsafe {
            let ptr = self.inner.get(index);
            VehicleLightStateListElement::from_cxx(ptr).unwrap()
        })
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiVehicleLightStateList>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

pub struct VehicleLightStateListElement<'a> {
    inner: UniquePtr<FfiVehicleLightStateElementRef>,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> VehicleLightStateListElement<'a> {
    pub fn id(&self) -> ActorId {
        self.inner.id()
    }

    pub fn light_state(&self) -> VehicleLightState {
        self.inner.light_state()
    }

    pub(crate) unsafe fn from_cxx(ptr: UniquePtr<FfiVehicleLightStateElementRef>) -> Option<Self> {
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
