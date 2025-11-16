use crate::{client::LightState, geom::Location, rpc::LightId, sensor::data::Color};
use carla_sys::carla_rust::client::FfiLightRef;
use cxx::UniquePtr;
use derivative::Derivative;
use std::marker::PhantomData;

pub use crate::rpc::LightGroup;

/// A mutable reference to a light in the simulation.
///
/// Lights represent street lights, building lights, and other light sources in the CARLA world.
/// This type allows you to query and modify light properties such as color, intensity, and on/off state.
///
/// Corresponds to [`carla.Light`] in the Python API.
#[cfg_attr(carla_version_0916, doc = "")]
#[cfg_attr(
    carla_version_0916,
    doc = " [`carla.Light`]: https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light"
)]
#[cfg_attr(carla_version_0915, doc = "")]
#[cfg_attr(
    carla_version_0915,
    doc = " [`carla.Light`]: https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light"
)]
#[cfg_attr(carla_version_0914, doc = "")]
#[cfg_attr(
    carla_version_0914,
    doc = " [`carla.Light`]: https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light"
)]
///
/// # Examples
///
/// ```no_run
/// use carla::{client::Client, sensor::data::Color};
///
/// let client = Client::default();
/// let world = client.world();
/// let mut light_manager = world.light_manager();
///
/// // Get all street lights
/// let lights = light_manager.all_lights();
///
/// // Modify the first light
/// if let Some(mut light) = lights.get_mut(0) {
///     println!("Light ID: {}", light.id());
///     println!("Location: {:?}", light.location());
///     println!("Is on: {}", light.is_on());
///
///     // Change light properties
///     light.set_color(Color {
///         r: 255,
///         g: 0,
///         b: 0,
///         a: 255,
///     });
///     light.set_intensity(100.0);
///     light.turn_on();
/// }
/// ```
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct LightMut<'a> {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiLightRef>,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> LightMut<'a> {
    /// Returns the color of the light.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.color](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.color)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.color](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.color)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.color](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.color)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn color(&self) -> Color {
        self.inner.GetColor()
    }

    /// Returns the unique identifier of this light.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.id)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.id](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.id)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.id](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.id)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn id(&self) -> LightId {
        self.inner.GetId()
    }

    /// Returns the intensity of the light.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.intensity](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.intensity)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.intensity](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.intensity)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.intensity](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.intensity)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn intensity(&self) -> f32 {
        self.inner.GetIntensity()
    }

    /// Returns the location of the light in the world.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.location](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.location)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.location](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.location)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.location](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.location)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn location(&self) -> Location {
        Location::from_ffi(self.inner.GetLocation())
    }

    /// Returns the group that this light belongs to.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.light_group](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.light_group)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.light_group](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.light_group)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.light_group](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.light_group)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn light_group(&self) -> LightGroup {
        self.inner.GetLightGroup()
    }

    /// Returns the current state of the light.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.light_state](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.light_state)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.light_state](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.light_state)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.light_state](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.light_state)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn light_state(&self) -> LightState {
        self.inner.GetLightState()
    }

    /// Returns true if the light is currently on.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.is_on](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.is_on)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.is_on](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.is_on)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.is_on](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.is_on)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn is_on(&self) -> bool {
        self.inner.IsOn()
    }

    /// Returns true if the light is currently off.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.is_off](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.is_off)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.is_off](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.is_off)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.is_off](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.is_off)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn is_off(&self) -> bool {
        self.inner.IsOff()
    }

    /// Sets the color of the light.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.set_color](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.set_color)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.set_color](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.set_color)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.set_color](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.set_color)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn set_color(&mut self, color: Color) {
        self.inner.pin_mut().SetColor(color)
    }

    /// Sets the intensity of the light.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.set_intensity](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.set_intensity)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.set_intensity](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.set_intensity)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.set_intensity](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.set_intensity)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn set_intensity(&mut self, intensity: f32) {
        self.inner.pin_mut().SetIntensity(intensity)
    }

    /// Sets the group that this light belongs to.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.set_light_group](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.set_light_group)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.set_light_group](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.set_light_group)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.set_light_group](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.set_light_group)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn set_light_group(&mut self, group: LightGroup) {
        self.inner.pin_mut().SetLightGroup(group)
    }

    /// Sets the complete state of the light.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.set_light_state](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.set_light_state)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.set_light_state](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.set_light_state)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.set_light_state](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.set_light_state)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn set_light_state(&mut self, state: &LightState) {
        self.inner.pin_mut().SetLightState(state);
    }

    /// Turns the light on.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.turn_on](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.turn_on)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.turn_on](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.turn_on)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.turn_on](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.turn_on)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn turn_on(&mut self) {
        self.inner.pin_mut().TurnOn();
    }

    /// Turns the light off.
    ///
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.Light.turn_off](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Light.turn_off)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.Light.turn_off](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.Light.turn_off)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.Light.turn_off](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Light.turn_off)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    pub fn turn_off(&mut self) {
        self.inner.pin_mut().TurnOff();
    }

    /// Constructs a `LightMut` from a raw FFI pointer.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the pointer is valid and that the lifetime
    /// of the returned `LightMut` does not outlive the underlying C++ object.
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
