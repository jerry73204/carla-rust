use super::{LightList, LightState};
use crate::{
    rpc::{LightGroup, LightId},
    sensor::data::Color,
};
use static_assertions::assert_impl_all;
use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::client::FfiLightManager;
use cxx::SharedPtr;
use derivative::Derivative;

/// Manages the states of lights in the simulation.
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct LightManager {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiLightManager>,
}

impl LightManager {
    pub fn all_lights(&self, group: LightGroup) -> LightList {
        let list = self.inner.GetAllLights(group).within_unique_ptr();
        LightList::from_cxx(list).unwrap()
    }

    pub fn color(&self, id: LightId) -> Color {
        self.inner.GetColor(id)
    }

    pub fn intensity(&self, id: LightId) -> f32 {
        self.inner.GetIntensity(id)
    }

    pub fn light_state(&self, id: LightId) -> LightState {
        self.inner.GetLightState(id)
    }

    pub fn light_group(&self, id: LightId) -> LightGroup {
        self.inner.GetLightGroup(id)
    }

    pub fn is_active(&self, id: LightId) -> bool {
        self.inner.IsActive(id)
    }

    pub fn set_active(&self, id: LightId, active: bool) {
        self.inner.SetActive(id, active);
    }

    pub fn set_color(&self, id: LightId, color: Color) {
        self.inner.SetColor(id, color);
    }

    pub fn set_intensity(&self, id: LightId, intensity: f32) {
        self.inner.SetIntensity(id, intensity);
    }

    pub fn set_light_state(&self, id: LightId, state: &LightState) {
        self.inner.SetLightState(id, state);
    }

    pub fn set_light_group(&self, id: LightId, group: LightGroup) {
        self.inner.SetLightGroup(id, group);
    }

    pub fn from_cxx(ptr: SharedPtr<FfiLightManager>) -> Option<Self> {
        if ptr.is_null() {
            return None;
        }

        Some(Self { inner: ptr })
    }
}

assert_impl_all!(LightManager: Send, Sync);
