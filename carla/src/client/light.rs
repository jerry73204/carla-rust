use crate::{
    client::LightState,
    geom::Location,
    rpc::{LightGroup, LightId},
    sensor::data::Color,
};
use carla_sys::carla_rust::client::FfiLightRef;
use cxx::UniquePtr;
use derivative::Derivative;
use std::marker::PhantomData;

#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct LightMut<'a> {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiLightRef>,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> LightMut<'a> {
    pub fn color(&self) -> Color {
        self.inner.GetColor()
    }

    pub fn id(&self) -> LightId {
        self.inner.GetId()
    }

    pub fn intensity(&self) -> f32 {
        self.inner.GetIntensity()
    }

    pub fn location(&self) -> Location {
        self.inner.GetLocation()
    }

    pub fn light_group(&self) -> LightGroup {
        self.inner.GetLightGroup()
    }

    pub fn light_state(&self) -> LightState {
        self.inner.GetLightState()
    }

    pub fn is_on(&self) -> bool {
        self.inner.IsOn()
    }

    pub fn is_off(&self) -> bool {
        self.inner.IsOff()
    }

    pub fn set_color(&mut self, color: Color) {
        self.inner.pin_mut().SetColor(color)
    }

    pub fn set_intensity(&mut self, intensity: f32) {
        self.inner.pin_mut().SetIntensity(intensity)
    }

    pub fn set_light_group(&mut self, group: LightGroup) {
        self.inner.pin_mut().SetLightGroup(group)
    }

    pub fn set_light_state(&mut self, state: &LightState) {
        self.inner.pin_mut().SetLightState(state);
    }

    pub fn turn_on(&mut self) {
        self.inner.pin_mut().TurnOn();
    }

    pub fn turn_off(&mut self) {
        self.inner.pin_mut().TurnOff();
    }

    pub unsafe fn from_cxx(ptr: UniquePtr<FfiLightRef>) -> Option<Self> {
        if ptr.is_null() {
            return None;
        }

        Some(Self {
            inner: ptr,
            _phantom: PhantomData,
        })
    }
}
