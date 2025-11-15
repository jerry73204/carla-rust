// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::{LightList, LightState};
use crate::{
    rpc::{LightGroup, LightId},
    sensor::data::Color,
};
use autocxx::WithinUniquePtr;
use carla_sys::carla_rust::client::FfiLightManager;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Manages the states of lights in the simulation.
///
/// The LightManager provides methods to query and modify lights in the CARLA world,
/// including street lights, building lights, and other light sources. You can control
/// individual lights or groups of lights, and adjust properties such as color, intensity,
/// and on/off state.
///
/// Corresponds to [`carla.LightManager`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager) in the Python API.
///
/// # Examples
///
/// ```no_run
/// use carla::{client::Client, rpc::LightGroup, sensor::data::Color};
///
/// let client = Client::default();
/// let world = client.world();
/// let light_manager = world.light_manager();
///
/// // Get all street lights
/// let street_lights = light_manager.all_lights(LightGroup::Street);
///
/// // Turn on all street lights with red color
/// for light in street_lights.iter() {
///     let id = light.id();
///     light_manager.set_active(id, true);
///     light_manager.set_color(
///         id,
///         Color {
///             r: 255,
///             g: 0,
///             b: 0,
///             a: 255,
///         },
///     );
///     light_manager.set_intensity(id, 100.0);
/// }
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct LightManager {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiLightManager>,
}

impl LightManager {
    /// Returns a list of lights belonging to the specified group.
    ///
    /// See [carla.LightManager.get_all_lights](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.get_all_lights)
    /// in the Python API.
    pub fn all_lights(&self, group: LightGroup) -> LightList {
        let list = self.inner.GetAllLights(group).within_unique_ptr();
        unsafe { LightList::from_cxx(list).unwrap_unchecked() }
    }

    /// Returns the color of the light with the given ID.
    ///
    /// See [carla.LightManager.get_color](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.get_color)
    /// in the Python API.
    pub fn color(&self, id: LightId) -> Color {
        self.inner.GetColor(id)
    }

    /// Returns the intensity of the light with the given ID.
    ///
    /// See [carla.LightManager.get_intensity](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.get_intensity)
    /// in the Python API.
    pub fn intensity(&self, id: LightId) -> f32 {
        self.inner.GetIntensity(id)
    }

    /// Returns the state of the light with the given ID.
    ///
    /// See [carla.LightManager.get_light_state](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.get_light_state)
    /// in the Python API.
    pub fn light_state(&self, id: LightId) -> LightState {
        self.inner.GetLightState(id)
    }

    /// Returns the group of the light with the given ID.
    ///
    /// See [carla.LightManager.get_light_group](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.get_light_group)
    /// in the Python API.
    pub fn light_group(&self, id: LightId) -> LightGroup {
        self.inner.GetLightGroup(id)
    }

    /// Returns true if the light with the given ID is active (on).
    ///
    /// See [carla.LightManager.is_active](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.is_active)
    /// in the Python API.
    pub fn is_active(&self, id: LightId) -> bool {
        self.inner.IsActive(id)
    }

    /// Sets whether the light with the given ID is active (on/off).
    ///
    /// See [carla.LightManager.set_active](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.set_active)
    /// in the Python API.
    pub fn set_active(&self, id: LightId, active: bool) {
        self.inner.SetActive(id, active);
    }

    /// Sets the color of the light with the given ID.
    ///
    /// See [carla.LightManager.set_color](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.set_color)
    /// in the Python API.
    pub fn set_color(&self, id: LightId, color: Color) {
        self.inner.SetColor(id, color);
    }

    /// Sets the intensity of the light with the given ID.
    ///
    /// See [carla.LightManager.set_intensity](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.set_intensity)
    /// in the Python API.
    pub fn set_intensity(&self, id: LightId, intensity: f32) {
        self.inner.SetIntensity(id, intensity);
    }

    /// Sets the complete state of the light with the given ID.
    ///
    /// See [carla.LightManager.set_light_state](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.set_light_state)
    /// in the Python API.
    pub fn set_light_state(&self, id: LightId, state: &LightState) {
        self.inner.SetLightState(id, state);
    }

    /// Sets the group of the light with the given ID.
    ///
    /// See [carla.LightManager.set_light_group](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.set_light_group)
    /// in the Python API.
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
