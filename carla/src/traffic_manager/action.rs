use super::RoadOption;
use crate::client::Waypoint;
use carla_sys::carla_rust::traffic_manager::FfiAction;
use cxx::UniquePtr;
use derivative::Derivative;

pub type Action = (RoadOption, Waypoint);

#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub(crate) struct PrivateAction {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiAction>,
}

impl PrivateAction {
    pub fn to_pair(&self) -> Action {
        let option = self.inner.road_option();
        let waypoint = unsafe { Waypoint::from_cxx(self.inner.waypoint()).unwrap_unchecked() };
        (option, waypoint)
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiAction>) -> Option<Self> {
        if ptr.is_null() {
            return None;
        }
        Some(Self { inner: ptr })
    }
}
