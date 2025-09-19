use super::Action;
use crate::client::Waypoint;
use carla_sys::carla_rust::traffic_manager::{FfiAction, FfiActionBuffer};
use cxx::UniquePtr;
use derivative::Derivative;
use std::slice;

#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct ActionBuffer {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiActionBuffer>,
}

impl ActionBuffer {
    pub fn len(&self) -> usize {
        self.inner.size()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn get(&self, index: usize) -> Option<Action> {
        let ffi_action = self.as_slice().get(index)?;
        let option = ffi_action.road_option();
        let waypoint = Waypoint::from_cxx(ffi_action.waypoint()).unwrap();
        Some((option, waypoint))
    }

    pub fn iter(&self) -> impl Iterator<Item = Action> + '_ {
        self.as_slice().iter().map(|ffi_action| {
            let option = ffi_action.road_option();
            let waypoint = Waypoint::from_cxx(ffi_action.waypoint()).unwrap();
            (option, waypoint)
        })
    }

    pub(crate) fn as_slice(&self) -> &[FfiAction] {
        // Get a &FfiActionBuffer then call its as_ptr() -> *const FfiAction
        let ptr = self
            .inner
            .as_ref()
            .expect("ActionBuffer pointer must be non-null")
            .as_ptr();
        unsafe { slice::from_raw_parts(ptr, self.len()) }
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiActionBuffer>) -> Option<Self> {
        if ptr.is_null() {
            return None;
        }
        Some(Self { inner: ptr })
    }
}
