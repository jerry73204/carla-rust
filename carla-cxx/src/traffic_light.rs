//! Traffic Light actor implementation for CARLA.

use crate::ffi::{self, Actor, TrafficLight};
use cxx::SharedPtr;

/// High-level wrapper for CARLA TrafficLight
pub struct TrafficLightWrapper {
    inner: SharedPtr<TrafficLight>,
}

impl TrafficLightWrapper {
    /// Create a TrafficLightWrapper from an Actor (performs cast)
    pub fn from_actor(actor: &Actor) -> Option<Self> {
        let traffic_light_ptr = ffi::Actor_CastToTrafficLight(actor);
        if traffic_light_ptr.is_null() {
            None
        } else {
            Some(Self {
                inner: traffic_light_ptr,
            })
        }
    }

    /// Get current traffic light state
    pub fn get_state(&self) -> TrafficLightState {
        let state_value = ffi::TrafficLight_GetState(&self.inner);
        TrafficLightState::from_u32(state_value).unwrap_or(TrafficLightState::Unknown)
    }

    /// Set traffic light state
    pub fn set_state(&self, state: TrafficLightState) {
        ffi::TrafficLight_SetState(&self.inner, state as u32);
    }

    /// Get elapsed time in current state
    pub fn get_elapsed_time(&self) -> f32 {
        ffi::TrafficLight_GetElapsedTime(&self.inner)
    }

    /// Set red light duration
    pub fn set_red_time(&self, red_time: f32) {
        ffi::TrafficLight_SetRedTime(&self.inner, red_time);
    }

    /// Set yellow light duration
    pub fn set_yellow_time(&self, yellow_time: f32) {
        ffi::TrafficLight_SetYellowTime(&self.inner, yellow_time);
    }

    /// Set green light duration
    pub fn set_green_time(&self, green_time: f32) {
        ffi::TrafficLight_SetGreenTime(&self.inner, green_time);
    }

    /// Get red light duration
    pub fn get_red_time(&self) -> f32 {
        ffi::TrafficLight_GetRedTime(&self.inner)
    }

    /// Get yellow light duration
    pub fn get_yellow_time(&self) -> f32 {
        ffi::TrafficLight_GetYellowTime(&self.inner)
    }

    /// Get green light duration
    pub fn get_green_time(&self) -> f32 {
        ffi::TrafficLight_GetGreenTime(&self.inner)
    }

    /// Freeze or unfreeze the traffic light (stops automatic state changes)
    pub fn freeze(&self, freeze: bool) {
        ffi::TrafficLight_Freeze(&self.inner, freeze);
    }

    /// Check if the traffic light is frozen
    pub fn is_frozen(&self) -> bool {
        ffi::TrafficLight_IsFrozen(&self.inner)
    }
}

/// Traffic light states
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u32)]
pub enum TrafficLightState {
    /// Red light - stop
    Red = 0,
    /// Yellow light - caution
    Yellow = 1,
    /// Green light - go
    Green = 2,
    /// Traffic light is off
    Off = 3,
    /// Unknown state
    Unknown = 4,
}

impl TrafficLightState {
    /// Convert from u32 value
    pub fn from_u32(value: u32) -> Option<Self> {
        match value {
            0 => Some(Self::Red),
            1 => Some(Self::Yellow),
            2 => Some(Self::Green),
            3 => Some(Self::Off),
            _ => None,
        }
    }
}
