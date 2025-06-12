use super::{Actor, ActorBase};
use crate::{
    geom::Transform,
    stubs::{
        carla_actor_is_traffic_light, carla_traffic_light_get_green_time,
        carla_traffic_light_get_group_traffic_lights, carla_traffic_light_get_red_time,
        carla_traffic_light_get_state, carla_traffic_light_get_yellow_time,
        carla_traffic_light_set_green_time, carla_traffic_light_set_red_time,
        carla_traffic_light_set_state, carla_traffic_light_set_yellow_time, carla_traffic_light_t,
    },
    utils::check_carla_error,
};
use anyhow::{anyhow, Result};
use carla_sys::*;

/// A traffic light actor in the simulation.
/// This is a newtype wrapper around Actor that provides traffic light-specific functionality.
#[derive(Clone, Debug)]
pub struct TrafficLight(pub Actor);

impl TrafficLight {
    /// Create a TrafficLight from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null, and must point to a traffic light actor.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_actor_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null traffic light pointer"));
        }

        // Verify it's actually a traffic light
        if !unsafe { carla_actor_is_traffic_light(ptr) } {
            return Err(anyhow!("Actor is not a traffic light"));
        }

        // Create the base Actor first
        let actor = Actor::from_raw_ptr(ptr)?;
        Ok(Self(actor))
    }

    /// Convert this TrafficLight back into a generic Actor.
    pub fn into_actor(self) -> Actor {
        self.0
    }

    /// Get access to the underlying Actor.
    pub fn actor(&self) -> &Actor {
        &self.0
    }

    /// Get mutable access to the underlying Actor.
    pub fn actor_mut(&mut self) -> &mut Actor {
        &mut self.0
    }

    pub(crate) fn raw_traffic_light_ptr(&self) -> *mut carla_traffic_light_t {
        self.0.raw_ptr() as *mut carla_traffic_light_t
    }

    /// Set the traffic light state.
    pub fn set_state(&self, state: TrafficLightState) -> Result<()> {
        let c_state = state.to_c_state();
        let error = unsafe { carla_traffic_light_set_state(self.raw_traffic_light_ptr(), c_state) };
        check_carla_error(error)
    }

    /// Get the current traffic light state.
    pub fn get_state(&self) -> TrafficLightState {
        let c_state = unsafe { carla_traffic_light_get_state(self.raw_traffic_light_ptr()) };
        TrafficLightState::from_c_state(c_state)
    }

    /// Set the duration of the green light phase in seconds.
    pub fn set_green_time(&self, green_time: f32) -> Result<()> {
        let error = unsafe {
            carla_traffic_light_set_green_time(self.raw_traffic_light_ptr(), green_time)
        };
        check_carla_error(error)
    }

    /// Get the duration of the green light phase in seconds.
    pub fn get_green_time(&self) -> f32 {
        unsafe { carla_traffic_light_get_green_time(self.raw_traffic_light_ptr()) }
    }

    /// Set the duration of the yellow light phase in seconds.
    pub fn set_yellow_time(&self, yellow_time: f32) -> Result<()> {
        let error = unsafe {
            carla_traffic_light_set_yellow_time(self.raw_traffic_light_ptr(), yellow_time)
        };
        check_carla_error(error)
    }

    /// Get the duration of the yellow light phase in seconds.
    pub fn get_yellow_time(&self) -> f32 {
        unsafe { carla_traffic_light_get_yellow_time(self.raw_traffic_light_ptr()) }
    }

    /// Set the duration of the red light phase in seconds.
    pub fn set_red_time(&self, red_time: f32) -> Result<()> {
        let error =
            unsafe { carla_traffic_light_set_red_time(self.raw_traffic_light_ptr(), red_time) };
        check_carla_error(error)
    }

    /// Get the duration of the red light phase in seconds.
    pub fn get_red_time(&self) -> f32 {
        unsafe { carla_traffic_light_get_red_time(self.raw_traffic_light_ptr()) }
    }

    /// Get all traffic lights that belong to the same group as this traffic light.
    pub fn get_group_traffic_lights(&self) -> Result<Vec<TrafficLight>> {
        let group_ptr =
            unsafe { carla_traffic_light_get_group_traffic_lights(self.raw_traffic_light_ptr()) };
        
        if group_ptr.is_null() {
            return Ok(Vec::new());
        }

        // TODO: Implement conversion from C traffic light list when C API is available
        Err(anyhow!(
            "Traffic light group retrieval not yet implemented in C API"
        ))
    }

    /// Set timing for all phases at once.
    pub fn set_timing(&self, timing: &TrafficLightTiming) -> Result<()> {
        self.set_green_time(timing.green_time)?;
        self.set_yellow_time(timing.yellow_time)?;
        self.set_red_time(timing.red_time)?;
        Ok(())
    }

    /// Get timing for all phases.
    pub fn get_timing(&self) -> TrafficLightTiming {
        TrafficLightTiming {
            green_time: self.get_green_time(),
            yellow_time: self.get_yellow_time(),
            red_time: self.get_red_time(),
        }
    }

    /// Reset traffic light to default timing.
    pub fn reset_timing(&self) -> Result<()> {
        let default_timing = TrafficLightTiming::default();
        self.set_timing(&default_timing)
    }

    /// Get the elapsed time in the current state.
    pub fn get_elapsed_time(&self) -> f32 {
        // TODO: Implement when C API provides carla_traffic_light_get_elapsed_time
        0.0
    }

    /// Check if this traffic light is part of a traffic light group.
    pub fn is_in_group(&self) -> bool {
        !self.get_group_traffic_lights().unwrap_or_default().is_empty()
    }

    /// Destroy this traffic light.
    pub fn destroy(self) -> Result<()> {
        self.0.destroy()
    }
}

/// Traffic light state enumeration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TrafficLightState {
    /// Red light - stop.
    Red,
    /// Yellow light - caution.
    Yellow,
    /// Green light - go.
    Green,
    /// Traffic light is off.
    Off,
    /// Blinking yellow - proceed with caution.
    Blinking,
    /// Unknown or invalid state.
    Unknown,
}

impl TrafficLightState {
    /// Convert to C traffic light state.
    pub(crate) fn to_c_state(self) -> carla_traffic_light_state_t {
        match self {
            Self::Red => carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_RED,
            Self::Yellow => carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_YELLOW,
            Self::Green => carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_GREEN,
            Self::Off => carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_OFF,
            Self::Blinking => carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_BLINKING,
            Self::Unknown => carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_UNKNOWN,
        }
    }

    /// Create from C traffic light state.
    pub(crate) fn from_c_state(c_state: carla_traffic_light_state_t) -> Self {
        match c_state {
            carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_RED => Self::Red,
            carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_YELLOW => Self::Yellow,
            carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_GREEN => Self::Green,
            carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_OFF => Self::Off,
            carla_traffic_light_state_t_CARLA_TRAFFIC_LIGHT_BLINKING => Self::Blinking,
            _ => Self::Unknown,
        }
    }
}

/// Traffic light timing configuration.
#[derive(Debug, Clone)]
pub struct TrafficLightTiming {
    /// Duration of green light phase in seconds.
    pub green_time: f32,
    /// Duration of yellow light phase in seconds.
    pub yellow_time: f32,
    /// Duration of red light phase in seconds.
    pub red_time: f32,
}

impl TrafficLightTiming {
    /// Create a new traffic light timing configuration.
    pub fn new(green_time: f32, yellow_time: f32, red_time: f32) -> Self {
        Self {
            green_time,
            yellow_time,
            red_time,
        }
    }

    /// Get the total cycle time.
    pub fn total_cycle_time(&self) -> f32 {
        self.green_time + self.yellow_time + self.red_time
    }
}

impl Default for TrafficLightTiming {
    fn default() -> Self {
        Self {
            green_time: 30.0,  // 30 seconds green
            yellow_time: 5.0,  // 5 seconds yellow
            red_time: 30.0,    // 30 seconds red
        }
    }
}

// Implement ActorBase trait for TrafficLight
impl ActorBase for TrafficLight {
    fn raw_ptr(&self) -> *mut carla_actor_t {
        self.0.raw_ptr()
    }

    fn id(&self) -> u32 {
        self.0.id()
    }

    fn type_id(&self) -> String {
        self.0.type_id()
    }

    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }

    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
}

// Note: TrafficLight doesn't implement Drop because it's a newtype wrapper around Actor,
// and the underlying carla_actor_t will be freed when the Actor is dropped

// SAFETY: TrafficLight wraps a thread-safe C API
unsafe impl Send for TrafficLight {}
unsafe impl Sync for TrafficLight {}
