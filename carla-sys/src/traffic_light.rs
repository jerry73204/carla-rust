//! Traffic Light actor implementation for CARLA.

use crate::ffi::{self, Actor, TrafficLight};
use cxx::SharedPtr;

/// High-level wrapper for CARLA TrafficLight
pub struct TrafficLightWrapper {
    inner: SharedPtr<TrafficLight>,
}

impl std::fmt::Debug for TrafficLightWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TrafficLightWrapper")
            .field("inner", &"SharedPtr<TrafficLight>")
            .finish()
    }
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

    /// Create a TrafficLightWrapper from a SharedPtr<TrafficLight>
    pub fn from_shared_ptr(traffic_light_ptr: SharedPtr<TrafficLight>) -> Option<Self> {
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

    /// Get the traffic light's type ID
    pub fn get_type_id(&self) -> String {
        ffi::TrafficLight_GetTypeId(&self.inner)
    }

    /// Get the traffic light's transform
    pub fn get_transform(&self) -> crate::ffi::SimpleTransform {
        ffi::TrafficLight_GetTransform(&self.inner)
    }

    /// Set the traffic light's transform
    pub fn set_transform(&self, transform: &crate::ffi::SimpleTransform) {
        ffi::TrafficLight_SetTransform(&self.inner, transform);
    }

    /// Get the traffic light's velocity
    pub fn get_velocity(&self) -> crate::ffi::SimpleVector3D {
        ffi::TrafficLight_GetVelocity(&self.inner)
    }

    /// Get the traffic light's angular velocity
    pub fn get_angular_velocity(&self) -> crate::ffi::SimpleVector3D {
        ffi::TrafficLight_GetAngularVelocity(&self.inner)
    }

    /// Get the traffic light's acceleration
    pub fn get_acceleration(&self) -> crate::ffi::SimpleVector3D {
        ffi::TrafficLight_GetAcceleration(&self.inner)
    }

    /// Check if the traffic light is alive
    pub fn is_alive(&self) -> bool {
        ffi::TrafficLight_IsAlive(&self.inner)
    }

    /// Destroy the traffic light
    pub fn destroy(&self) -> bool {
        ffi::TrafficLight_Destroy(&self.inner)
    }

    /// Set physics simulation for the traffic light
    pub fn set_simulate_physics(&self, enabled: bool) {
        ffi::TrafficLight_SetSimulatePhysics(&self.inner, enabled);
    }

    /// Add impulse to the traffic light
    pub fn add_impulse(&self, impulse: &crate::ffi::SimpleVector3D) {
        ffi::TrafficLight_AddImpulse(&self.inner, impulse);
    }

    /// Add force to the traffic light
    pub fn add_force(&self, force: &crate::ffi::SimpleVector3D) {
        ffi::TrafficLight_AddForce(&self.inner, force);
    }

    /// Add torque to the traffic light
    pub fn add_torque(&self, torque: &crate::ffi::SimpleVector3D) {
        ffi::TrafficLight_AddTorque(&self.inner, torque);
    }

    /// Get affected lane waypoints for this traffic light
    pub fn get_affected_lane_waypoints(&self) -> crate::map::WaypointVector {
        let vec = ffi::TrafficLight_GetAffectedLaneWaypoints(&self.inner);
        crate::map::WaypointVector::new(vec)
    }

    /// Get the pole index for this traffic light
    pub fn get_pole_index(&self) -> u32 {
        todo!("TrafficLight_GetPoleIndex FFI function added but CXX bridge integration needs debugging")
    }

    /// Get all traffic lights in the same group
    pub fn get_group_traffic_lights(&self) -> Vec<crate::ffi::bridge::SimpleTrafficLightInfo> {
        todo!("TrafficLight_GetGroupTrafficLights FFI function added but CXX bridge integration needs debugging")
    }

    /// Get the inner SharedPtr for direct FFI access
    pub fn get_inner_traffic_light(&self) -> &cxx::SharedPtr<TrafficLight> {
        &self.inner
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
