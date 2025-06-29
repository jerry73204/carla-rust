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
        if let Some(traffic_light_wrapper) =
            TrafficLightWrapper::from_actor(actor.inner_wrapper().get_shared_ptr())
        {
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn test_traffic_light_state_enum() {
        // Test all traffic light states
        let states = [
            TrafficLightState::Red,
            TrafficLightState::Yellow,
            TrafficLightState::Green,
            TrafficLightState::Off,
            TrafficLightState::Unknown,
        ];

        assert_eq!(states.len(), 5);

        // Test Debug output
        assert_eq!(format!("{:?}", TrafficLightState::Red), "Red");
        assert_eq!(format!("{:?}", TrafficLightState::Green), "Green");
        assert_eq!(format!("{:?}", TrafficLightState::Unknown), "Unknown");
    }

    #[test]
    fn test_traffic_light_state_default() {
        let default_state = TrafficLightState::default();
        assert_eq!(default_state, TrafficLightState::Red);
    }

    #[test]
    fn test_traffic_light_state_equality() {
        assert_eq!(TrafficLightState::Red, TrafficLightState::Red);
        assert_ne!(TrafficLightState::Red, TrafficLightState::Green);
        assert_ne!(TrafficLightState::Yellow, TrafficLightState::Off);
    }

    #[test]
    fn test_traffic_light_state_clone() {
        let state = TrafficLightState::Green;
        let cloned_state = state;
        assert_eq!(state, cloned_state);
    }

    #[test]
    fn test_traffic_light_state_copy() {
        let state = TrafficLightState::Yellow;
        let copied_state = state; // Copy trait
        assert_eq!(state, copied_state);
        // Both variables should still be usable
        assert_eq!(state, TrafficLightState::Yellow);
        assert_eq!(copied_state, TrafficLightState::Yellow);
    }

    // Note: Testing actual traffic light functionality would require FFI mocks
    // Here we focus on testing the data structures and enums

    #[test]
    fn test_duration_conversion_concepts() {
        // Test that Duration operations work as expected
        let green_time = Duration::from_secs(30);
        let yellow_time = Duration::from_secs(5);
        let red_time = Duration::from_secs(25);

        assert_eq!(green_time.as_secs(), 30);
        assert_eq!(yellow_time.as_secs(), 5);
        assert_eq!(red_time.as_secs(), 25);

        // Test fractional seconds
        let fractional_time = Duration::from_secs_f32(2.5);
        assert_eq!(fractional_time.as_secs_f32(), 2.5);
    }

    #[test]
    fn test_traffic_light_timing_constants() {
        // Test common traffic light timing values
        let typical_green = Duration::from_secs(30);
        let typical_yellow = Duration::from_secs(3);
        let typical_red = Duration::from_secs(30);

        let total_cycle = typical_green + typical_yellow + typical_red;
        assert_eq!(total_cycle.as_secs(), 63);
    }

    #[test]
    fn test_traffic_light_short_cycles() {
        // Test pedestrian crossing timing
        let ped_green = Duration::from_secs(15);
        let ped_yellow = Duration::from_secs(3);
        let ped_red = Duration::from_secs(45);

        assert!(ped_green < Duration::from_secs(20));
        assert!(ped_yellow >= Duration::from_secs(3));
        assert!(ped_red > ped_green);
    }

    #[test]
    fn test_traffic_light_state_pattern_matching() {
        fn get_next_state(current: TrafficLightState) -> TrafficLightState {
            match current {
                TrafficLightState::Red => TrafficLightState::Green,
                TrafficLightState::Green => TrafficLightState::Yellow,
                TrafficLightState::Yellow => TrafficLightState::Red,
                TrafficLightState::Off => TrafficLightState::Off,
                TrafficLightState::Unknown => TrafficLightState::Red,
            }
        }

        assert_eq!(
            get_next_state(TrafficLightState::Red),
            TrafficLightState::Green
        );
        assert_eq!(
            get_next_state(TrafficLightState::Green),
            TrafficLightState::Yellow
        );
        assert_eq!(
            get_next_state(TrafficLightState::Yellow),
            TrafficLightState::Red
        );
        assert_eq!(
            get_next_state(TrafficLightState::Off),
            TrafficLightState::Off
        );
        assert_eq!(
            get_next_state(TrafficLightState::Unknown),
            TrafficLightState::Red
        );
    }

    #[test]
    fn test_traffic_light_state_is_active() {
        fn is_active_state(state: TrafficLightState) -> bool {
            matches!(
                state,
                TrafficLightState::Red | TrafficLightState::Yellow | TrafficLightState::Green
            )
        }

        assert!(is_active_state(TrafficLightState::Red));
        assert!(is_active_state(TrafficLightState::Yellow));
        assert!(is_active_state(TrafficLightState::Green));
        assert!(!is_active_state(TrafficLightState::Off));
        assert!(!is_active_state(TrafficLightState::Unknown));
    }
}
