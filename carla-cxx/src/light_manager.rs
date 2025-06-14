//! Light management system for CARLA simulator.
//!
//! This module provides interfaces for controlling lights in the CARLA world,
//! including street lights, building lights, and vehicle lights through both
//! the LightManager for bulk operations and individual Light control.

use crate::ffi::bridge;
use cxx::SharedPtr;

/// Light group classification for organizing lights by type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LightGroup {
    None = 0,
    Vehicle = 1,
    Street = 2,
    Building = 3,
    Other = 4,
}

impl From<u8> for LightGroup {
    fn from(value: u8) -> Self {
        match value {
            0 => LightGroup::None,
            1 => LightGroup::Vehicle,
            2 => LightGroup::Street,
            3 => LightGroup::Building,
            4 => LightGroup::Other,
            _ => LightGroup::None,
        }
    }
}

impl From<LightGroup> for u8 {
    fn from(group: LightGroup) -> Self {
        group as u8
    }
}

/// State of a light including intensity, color, group, and active status.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LightState {
    pub intensity: f32,
    pub color: (u8, u8, u8), // RGB
    pub group: LightGroup,
    pub active: bool,
}

impl From<bridge::SimpleLightState> for LightState {
    fn from(simple: bridge::SimpleLightState) -> Self {
        LightState {
            intensity: simple.intensity,
            color: (simple.color.r, simple.color.g, simple.color.b),
            group: LightGroup::from(simple.group),
            active: simple.active,
        }
    }
}

impl From<LightState> for bridge::SimpleLightState {
    fn from(state: LightState) -> Self {
        bridge::SimpleLightState {
            intensity: state.intensity,
            color: bridge::SimpleColor {
                r: state.color.0,
                g: state.color.1,
                b: state.color.2,
            },
            group: state.group.into(),
            active: state.active,
        }
    }
}

/// Information about a light in the CARLA world.
#[derive(Debug, Clone, PartialEq)]
pub struct LightInfo {
    pub id: u32,
    pub location: (f64, f64, f64), // x, y, z
    pub state: LightState,
}

impl From<bridge::SimpleLight> for LightInfo {
    fn from(simple: bridge::SimpleLight) -> Self {
        LightInfo {
            id: simple.id,
            location: (simple.location.x, simple.location.y, simple.location.z),
            state: simple.state.into(),
        }
    }
}

/// High-level interface for managing lights in the CARLA world.
///
/// The LightManager provides bulk operations for controlling multiple lights
/// efficiently, as well as day/night cycle management.
pub struct LightManagerWrapper {
    light_manager: SharedPtr<bridge::LightManager>,
}

impl LightManagerWrapper {
    /// Create a new LightManagerWrapper from a shared pointer.
    pub fn new(light_manager: SharedPtr<bridge::LightManager>) -> Self {
        Self { light_manager }
    }

    /// Get all lights of a specific group.
    ///
    /// # Arguments
    /// * `group` - The light group to filter by (None for all lights)
    ///
    /// # Returns
    /// Vector of LightInfo containing all lights matching the group
    pub fn get_all_lights(&self, group: LightGroup) -> Vec<LightInfo> {
        bridge::LightManager_GetAllLights(&self.light_manager, group.into())
            .into_iter()
            .map(|light| light.into())
            .collect()
    }

    /// Enable or disable the automatic day/night cycle.
    ///
    /// When enabled, the light manager will automatically turn lights on/off
    /// based on the time of day in the simulation.
    ///
    /// # Arguments
    /// * `active` - Whether to enable the day/night cycle
    pub fn set_day_night_cycle(&self, active: bool) {
        bridge::LightManager_SetDayNightCycle(&self.light_manager, active);
    }

    /// Turn on multiple lights by their IDs.
    ///
    /// # Arguments
    /// * `light_ids` - Vector of light IDs to turn on
    pub fn turn_on_lights(&self, light_ids: &[u32]) {
        bridge::LightManager_TurnOnLights(&self.light_manager, light_ids);
    }

    /// Turn off multiple lights by their IDs.
    ///
    /// # Arguments
    /// * `light_ids` - Vector of light IDs to turn off
    pub fn turn_off_lights(&self, light_ids: &[u32]) {
        bridge::LightManager_TurnOffLights(&self.light_manager, light_ids);
    }

    /// Set the intensity for multiple lights.
    ///
    /// # Arguments
    /// * `light_ids` - Vector of light IDs to modify
    /// * `intensity` - New intensity value (in lumens)
    pub fn set_light_intensities(&self, light_ids: &[u32], intensity: f32) {
        bridge::LightManager_SetLightIntensities(&self.light_manager, light_ids, intensity);
    }

    /// Set the color for multiple lights.
    ///
    /// # Arguments
    /// * `light_ids` - Vector of light IDs to modify
    /// * `color` - RGB color tuple (r, g, b) with values 0-255
    pub fn set_light_colors(&self, light_ids: &[u32], color: (u8, u8, u8)) {
        let simple_color = bridge::SimpleColor {
            r: color.0,
            g: color.1,
            b: color.2,
        };
        bridge::LightManager_SetLightColors(&self.light_manager, light_ids, simple_color);
    }

    /// Set the complete state for multiple lights.
    ///
    /// # Arguments
    /// * `light_ids` - Vector of light IDs to modify
    /// * `state` - New light state to apply
    pub fn set_light_states(&self, light_ids: &[u32], state: LightState) {
        bridge::LightManager_SetLightStates(&self.light_manager, light_ids, state.into());
    }
}
