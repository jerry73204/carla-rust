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

    /// Set complete timing configuration
    pub fn set_timing(&self, config: &TrafficLightTiming) {
        self.set_red_time(config.red_time);
        self.set_yellow_time(config.yellow_time);
        self.set_green_time(config.green_time);
    }

    /// Get complete timing configuration
    pub fn get_timing(&self) -> TrafficLightTiming {
        TrafficLightTiming {
            red_time: self.get_red_time(),
            yellow_time: self.get_yellow_time(),
            green_time: self.get_green_time(),
        }
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

    /// Check if the light allows traffic to proceed
    pub fn is_go(&self) -> bool {
        matches!(self, Self::Green)
    }

    /// Check if the light requires traffic to stop
    pub fn is_stop(&self) -> bool {
        matches!(self, Self::Red)
    }

    /// Check if the light is in caution state
    pub fn is_caution(&self) -> bool {
        matches!(self, Self::Yellow)
    }

    /// Get the next state in the normal traffic light cycle
    pub fn next_state(&self) -> Self {
        match self {
            Self::Red => Self::Green,
            Self::Yellow => Self::Red,
            Self::Green => Self::Yellow,
            Self::Off => Self::Red,
            Self::Unknown => Self::Red,
        }
    }
}

/// Traffic light timing configuration
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrafficLightTiming {
    /// Duration of red light in seconds
    pub red_time: f32,
    /// Duration of yellow light in seconds
    pub yellow_time: f32,
    /// Duration of green light in seconds
    pub green_time: f32,
}

impl Default for TrafficLightTiming {
    fn default() -> Self {
        Self {
            red_time: 30.0,   // 30 seconds red
            yellow_time: 5.0, // 5 seconds yellow
            green_time: 25.0, // 25 seconds green
        }
    }
}

impl TrafficLightTiming {
    /// Create a new timing configuration
    pub fn new(red_time: f32, yellow_time: f32, green_time: f32) -> Self {
        Self {
            red_time: red_time.max(0.0),
            yellow_time: yellow_time.max(0.0),
            green_time: green_time.max(0.0),
        }
    }

    /// Get total cycle time
    pub fn total_cycle_time(&self) -> f32 {
        self.red_time + self.yellow_time + self.green_time
    }

    /// Create a quick timing (shorter durations)
    pub fn quick() -> Self {
        Self::new(10.0, 2.0, 8.0)
    }

    /// Create a slow timing (longer durations)
    pub fn slow() -> Self {
        Self::new(60.0, 10.0, 50.0)
    }
}

impl std::fmt::Display for TrafficLightState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Red => write!(f, "🔴 Red"),
            Self::Yellow => write!(f, "🟡 Yellow"),
            Self::Green => write!(f, "🟢 Green"),
            Self::Off => write!(f, "⚫ Off"),
            Self::Unknown => write!(f, "❓ Unknown"),
        }
    }
}
