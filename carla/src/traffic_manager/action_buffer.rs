use super::Action;
use crate::client::Waypoint;
use carla_sys::carla_rust::traffic_manager::{FfiAction, FfiActionBuffer};
use cxx::UniquePtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;
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
        let waypoint = unsafe { Waypoint::from_cxx(ffi_action.waypoint()).unwrap_unchecked() };
        Some((option, waypoint))
    }

    pub fn iter(&self) -> impl Iterator<Item = Action> + '_ {
        self.as_slice().iter().map(|ffi_action| {
            let option = ffi_action.road_option();
            let waypoint = unsafe { Waypoint::from_cxx(ffi_action.waypoint()).unwrap_unchecked() };
            (option, waypoint)
        })
    }

    pub(crate) fn as_slice(&self) -> &[FfiAction] {
        let ptr = self.inner.data();
        let len = self.len();

        debug_assert!(!ptr.is_null(), "ActionBuffer data pointer is null");
        debug_assert!(
            (ptr as usize).is_multiple_of(std::mem::align_of::<FfiAction>()),
            "ActionBuffer data pointer not properly aligned"
        );

        unsafe { slice::from_raw_parts(ptr, len) }
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiActionBuffer>) -> Option<Self> {
        if ptr.is_null() {
            return None;
        }
        Some(Self { inner: ptr })
    }
}

assert_impl_all!(ActionBuffer: Send, Sync);
