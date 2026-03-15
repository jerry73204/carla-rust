use super::{LightList, LightState};
use crate::{
    error::ffi::with_ffi_error,
    rpc::{LightGroup, LightId},
    sensor::data::Color,
};
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
/// Corresponds to [`carla.LightManager`] in the Python API.
#[cfg_attr(carla_version_0916, doc = "")]
#[cfg_attr(
    carla_version_0916,
    doc = " [`carla.LightManager`]: https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager"
)]
#[cfg_attr(carla_version_0915, doc = "")]
#[cfg_attr(
    carla_version_0915,
    doc = " [`carla.LightManager`]: https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LightManager"
)]
#[cfg_attr(carla_version_0914, doc = "")]
#[cfg_attr(
    carla_version_0914,
    doc = " [`carla.LightManager`]: https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LightManager"
)]
///
/// # Examples
///
/// ```no_run
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// use carla::{client::Client, rpc::LightGroup, sensor::data::Color};
///
/// let client = Client::connect("localhost", 2000, None)?;
/// let world = client.world()?;
/// let light_manager = world.light_manager()?;
///
/// // Get all street lights
/// let street_lights = light_manager.all_lights(LightGroup::Street)?;
///
/// // Turn on all street lights with red color
/// for light in street_lights.iter() {
///     let id = light.id();
///     light_manager.set_active(id, true)?;
///     light_manager.set_color(
///         id,
///         Color {
///             r: 255,
///             g: 0,
///             b: 0,
///             a: 255,
///         },
///     )?;
///     light_manager.set_intensity(id, 100.0)?;
/// }
/// # Ok(())
/// # }
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
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LightManager.get_all_lights](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.get_all_lights)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LightManager.get_all_lights](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LightManager.get_all_lights)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LightManager.get_all_lights](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LightManager.get_all_lights)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn all_lights(&self, group: LightGroup) -> crate::Result<LightList> {
        let ptr = with_ffi_error("all_lights", |e| self.inner.GetAllLights(group, e))?;
        Ok(unsafe { LightList::from_cxx(ptr).unwrap_unchecked() })
    }

    /// Returns the color of the light with the given ID.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LightManager.get_color](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.get_color)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LightManager.get_color](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LightManager.get_color)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LightManager.get_color](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LightManager.get_color)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn color(&self, id: LightId) -> crate::Result<Color> {
        with_ffi_error("color", |e| self.inner.GetColor(id, e))
    }

    /// Returns the intensity of the light with the given ID.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LightManager.get_intensity](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.get_intensity)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LightManager.get_intensity](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LightManager.get_intensity)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LightManager.get_intensity](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LightManager.get_intensity)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn intensity(&self, id: LightId) -> crate::Result<f32> {
        with_ffi_error("intensity", |e| self.inner.GetIntensity(id, e))
    }

    /// Returns the state of the light with the given ID.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LightManager.get_light_state](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.get_light_state)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LightManager.get_light_state](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LightManager.get_light_state)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LightManager.get_light_state](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LightManager.get_light_state)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn light_state(&self, id: LightId) -> crate::Result<LightState> {
        with_ffi_error("light_state", |e| self.inner.GetLightState(id, e))
    }

    /// Returns the group of the light with the given ID.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LightManager.get_light_group](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.get_light_group)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LightManager.get_light_group](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LightManager.get_light_group)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LightManager.get_light_group](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LightManager.get_light_group)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn light_group(&self, id: LightId) -> crate::Result<LightGroup> {
        with_ffi_error("light_group", |e| self.inner.GetLightGroup(id, e))
    }

    /// Returns true if the light with the given ID is active (on).
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LightManager.is_active](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.is_active)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LightManager.is_active](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LightManager.is_active)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LightManager.is_active](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LightManager.is_active)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn is_active(&self, id: LightId) -> crate::Result<bool> {
        with_ffi_error("is_active", |e| self.inner.IsActive(id, e))
    }

    /// Sets whether the light with the given ID is active (on/off).
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LightManager.set_active](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.set_active)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LightManager.set_active](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LightManager.set_active)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LightManager.set_active](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LightManager.set_active)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn set_active(&self, id: LightId, active: bool) -> crate::Result<()> {
        with_ffi_error("set_active", |e| self.inner.SetActive(id, active, e))
    }

    /// Sets the color of the light with the given ID.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LightManager.set_color](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.set_color)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LightManager.set_color](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LightManager.set_color)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LightManager.set_color](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LightManager.set_color)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn set_color(&self, id: LightId, color: Color) -> crate::Result<()> {
        with_ffi_error("set_color", |e| self.inner.SetColor(id, color, e))
    }

    /// Sets the intensity of the light with the given ID.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LightManager.set_intensity](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.set_intensity)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LightManager.set_intensity](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LightManager.set_intensity)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LightManager.set_intensity](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LightManager.set_intensity)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn set_intensity(&self, id: LightId, intensity: f32) -> crate::Result<()> {
        with_ffi_error("set_intensity", |e| {
            self.inner.SetIntensity(id, intensity, e)
        })
    }

    /// Sets the complete state of the light with the given ID.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LightManager.set_light_state](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.set_light_state)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LightManager.set_light_state](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LightManager.set_light_state)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LightManager.set_light_state](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LightManager.set_light_state)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn set_light_state(&self, id: LightId, state: &LightState) -> crate::Result<()> {
        with_ffi_error("set_light_state", |e| {
            self.inner.SetLightState(id, state, e)
        })
    }

    /// Sets the group of the light with the given ID.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.LightManager.set_light_group](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.LightManager.set_light_group)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.LightManager.set_light_group](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.LightManager.set_light_group)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.LightManager.set_light_group](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.LightManager.set_light_group)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn set_light_group(&self, id: LightId, group: LightGroup) -> crate::Result<()> {
        with_ffi_error("set_light_group", |e| {
            self.inner.SetLightGroup(id, group, e)
        })
    }

    pub fn from_cxx(ptr: SharedPtr<FfiLightManager>) -> Option<Self> {
        if ptr.is_null() {
            return None;
        }

        Some(Self { inner: ptr })
    }
}

assert_impl_all!(LightManager: Send, Sync);
