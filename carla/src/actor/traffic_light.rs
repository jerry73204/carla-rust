//! Traffic light actor implementation.

use crate::{
    actor::{Actor, ActorFfi, ActorId},
    error::CarlaResult,
    road::WaypointList,
};
use carla_sys::TrafficLightWrapper;
use std::time::Duration;

/// Traffic light actor.
#[derive(Debug)]
pub struct TrafficLight {
    /// Internal traffic light wrapper for FFI calls
    inner: TrafficLightWrapper,
}

impl TrafficLight {
    /// Create a traffic light from a carla-sys TrafficLightWrapper and actor ID.
    pub(crate) fn from_cxx(traffic_light_wrapper: TrafficLightWrapper) -> CarlaResult<Self> {
        Ok(Self {
            inner: traffic_light_wrapper,
        })
    }

    /// Create a traffic light from an actor by casting.
    pub fn from_actor(actor: Actor) -> Result<Self, Actor> {
        let actor_ref = actor.inner_actor();
        if let Some(traffic_light_wrapper) = TrafficLightWrapper::from_actor(actor_ref) {
            Ok(Self {
                inner: traffic_light_wrapper,
            })
        } else {
            Err(actor)
        }
    }

    /// Convert this traffic light to a generic Actor.
    ///
    /// This creates a new Actor instance that represents the same traffic light.
    /// This is useful when you need to work with generic actor functionality.
    pub fn into_actor(self) -> Actor {
        let actor_wrapper = self.inner.as_actor_wrapper();
        Actor::from_cxx(actor_wrapper)
    }

    /// Get current traffic light state.
    pub fn state(&self) -> TrafficLightState {
        let cxx_state = self.inner.get_state();
        TrafficLightState::from_cxx(cxx_state)
    }

    /// Set traffic light state.
    pub fn set_state(&self, state: TrafficLightState) -> CarlaResult<()> {
        let cxx_state = state.into_cxx();
        self.inner.set_state(cxx_state);
        Ok(())
    }

    /// Get the remaining time for current state.
    pub fn elapsed_time(&self) -> Duration {
        let elapsed_seconds = self.inner.get_elapsed_time();
        Duration::from_secs_f32(elapsed_seconds)
    }

    /// Get traffic light group (affected lanes).
    /// Returns waypoint information for lanes affected by this traffic light.
    pub fn affected_lane_waypoints(&self) -> WaypointList {
        let waypoint_vec = self.inner.get_affected_lane_waypoints();
        WaypointList::new(waypoint_vec)
    }

    /// Get traffic light pole index.
    pub fn pole_index(&self) -> u32 {
        // Call the wrapper method which has the FFI implementation
        self.inner.get_pole_index()
    }

    /// Get all traffic lights in the same group.
    /// Returns actor IDs of all traffic lights in the same group as this one.
    ///
    /// Note: To get the actual TrafficLight objects, use World::actor(id) and cast to TrafficLight.
    pub fn group_traffic_lights(&self) -> Vec<ActorId> {
        // Call the wrapper method which has the FFI implementation
        let traffic_light_infos = self.inner.get_group_traffic_lights();

        // Extract actor IDs from the traffic light info
        traffic_light_infos
            .into_iter()
            .map(|info| info.actor_id)
            .collect()
    }

    /// Freeze traffic light (stop automatic state changes).
    pub fn freeze(&self, freeze: bool) -> CarlaResult<()> {
        self.inner.freeze(freeze);
        Ok(())
    }

    /// Check if traffic light is frozen.
    pub fn is_frozen(&self) -> bool {
        self.inner.is_frozen()
    }

    /// Set traffic light green time.
    pub fn set_green_time(&self, time: Duration) -> CarlaResult<()> {
        let time_seconds = time.as_secs_f32();
        self.inner.set_green_time(time_seconds);
        Ok(())
    }

    /// Get traffic light green time.
    pub fn green_time(&self) -> Duration {
        let time_seconds = self.inner.get_green_time();
        Duration::from_secs_f32(time_seconds)
    }

    /// Set traffic light yellow time.
    pub fn set_yellow_time(&self, time: Duration) -> CarlaResult<()> {
        let time_seconds = time.as_secs_f32();
        self.inner.set_yellow_time(time_seconds);
        Ok(())
    }

    /// Get traffic light yellow time.
    pub fn yellow_time(&self) -> Duration {
        let time_seconds = self.inner.get_yellow_time();
        Duration::from_secs_f32(time_seconds)
    }

    /// Set traffic light red time.
    pub fn set_red_time(&self, time: Duration) -> CarlaResult<()> {
        let time_seconds = time.as_secs_f32();
        self.inner.set_red_time(time_seconds);
        Ok(())
    }

    /// Get traffic light red time.
    pub fn red_time(&self) -> Duration {
        let time_seconds = self.inner.get_red_time();
        Duration::from_secs_f32(time_seconds)
    }
}

impl ActorFfi for TrafficLight {
    fn as_actor_ffi(&self) -> carla_sys::ActorWrapper {
        self.inner.as_actor_wrapper()
    }
}

impl Drop for TrafficLight {
    fn drop(&mut self) {
        // Only destroy if the traffic light is still alive
        if self.inner.is_alive() {
            let _ = self.inner.destroy();
        }
    }
}

/// Traffic light states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrafficLightState {
    /// Red light
    Red,
    /// Yellow light
    Yellow,
    /// Green light
    Green,
    /// Off (no light)
    Off,
    /// Unknown state
    Unknown,
}

impl Default for TrafficLightState {
    fn default() -> Self {
        Self::Red
    }
}

impl TrafficLightState {
    /// Convert from carla-sys TrafficLightState
    pub(crate) fn from_cxx(cxx_state: carla_sys::TrafficLightState) -> Self {
        use carla_sys::TrafficLightState as CxxState;
        match cxx_state {
            CxxState::Red => Self::Red,
            CxxState::Yellow => Self::Yellow,
            CxxState::Green => Self::Green,
            CxxState::Off => Self::Off,
            CxxState::Unknown => Self::Unknown,
        }
    }

    /// Convert to carla-sys TrafficLightState
    pub(crate) fn into_cxx(self) -> carla_sys::TrafficLightState {
        use carla_sys::TrafficLightState as CxxState;
        match self {
            Self::Red => CxxState::Red,
            Self::Yellow => CxxState::Yellow,
            Self::Green => CxxState::Green,
            Self::Off => CxxState::Off,
            Self::Unknown => CxxState::Unknown,
        }
    }
}
