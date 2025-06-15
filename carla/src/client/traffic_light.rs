//! Traffic light actor implementation.

use crate::{
    client::{Actor, ActorId},
    error::{ActorError, CarlaError, CarlaResult},
    geom::{FromCxx, ToCxx, Transform, Vector3D},
    traits::ActorT,
};
use carla_cxx::TrafficLightWrapper;
use std::time::Duration;

/// Traffic light actor.
#[derive(Debug)]
pub struct TrafficLight {
    /// Actor ID
    id: ActorId,
    /// Internal traffic light wrapper for FFI calls
    inner: TrafficLightWrapper,
}

impl TrafficLight {
    /// Create a traffic light from a carla-cxx TrafficLightWrapper and actor ID.
    pub fn new(traffic_light_wrapper: TrafficLightWrapper, id: ActorId) -> CarlaResult<Self> {
        Ok(Self {
            id,
            inner: traffic_light_wrapper,
        })
    }

    /// Create a traffic light from an actor by casting.
    pub fn from_actor(actor: Actor) -> CarlaResult<Option<Self>> {
        let actor_ref = actor.get_inner_actor();
        let actor_id = actor.get_id();
        if let Some(traffic_light_wrapper) = TrafficLightWrapper::from_actor(actor_ref) {
            Ok(Some(Self {
                id: actor_id,
                inner: traffic_light_wrapper,
            }))
        } else {
            Ok(None)
        }
    }

    /// Get the traffic light's actor ID.
    pub fn get_id(&self) -> ActorId {
        self.id
    }

    /// Get current traffic light state.
    pub fn get_state(&self) -> TrafficLightState {
        let cxx_state = self.inner.get_state();
        TrafficLightState::from_cxx(cxx_state)
    }

    /// Set traffic light state.
    pub fn set_state(&self, state: TrafficLightState) -> CarlaResult<()> {
        let cxx_state = state.to_cxx();
        self.inner.set_state(cxx_state);
        Ok(())
    }

    /// Get the remaining time for current state.
    pub fn get_elapsed_time(&self) -> Duration {
        let elapsed_seconds = self.inner.get_elapsed_time();
        Duration::from_secs_f32(elapsed_seconds)
    }

    /// Get traffic light group (affected lanes).
    /// Returns waypoint information for lanes affected by this traffic light.
    pub fn get_affected_lane_waypoints(&self) -> Vec<carla_cxx::ffi::bridge::SimpleWaypointInfo> {
        todo!("TrafficLight_GetAffectedLaneWaypoints FFI function added but CXX bridge integration needs debugging")
    }

    /// Get traffic light pole index.
    pub fn get_pole_index(&self) -> u32 {
        todo!("TrafficLight_GetPoleIndex FFI function added but CXX bridge integration needs debugging")
    }

    /// Get all traffic lights in the same group.
    /// Returns information about all traffic lights in the same group as this one.
    pub fn get_group_traffic_lights(&self) -> Vec<carla_cxx::ffi::bridge::SimpleTrafficLightInfo> {
        todo!("TrafficLight_GetGroupTrafficLights FFI function added but CXX bridge integration needs debugging")
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
    pub fn get_green_time(&self) -> Duration {
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
    pub fn get_yellow_time(&self) -> Duration {
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
    pub fn get_red_time(&self) -> Duration {
        let time_seconds = self.inner.get_red_time();
        Duration::from_secs_f32(time_seconds)
    }
}

impl ActorT for TrafficLight {
    fn get_id(&self) -> ActorId {
        self.id
    }
    fn get_type_id(&self) -> String {
        self.inner.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        let cxx_transform = self.inner.get_transform();
        Transform::from(cxx_transform)
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        let cxx_transform = transform.to_cxx();
        self.inner.set_transform(&cxx_transform);
        Ok(())
    }
    fn get_velocity(&self) -> Vector3D {
        let vel = self.inner.get_velocity();
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn get_angular_velocity(&self) -> Vector3D {
        let vel = self.inner.get_angular_velocity();
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn get_acceleration(&self) -> Vector3D {
        let acc = self.inner.get_acceleration();
        Vector3D::new(acc.x as f32, acc.y as f32, acc.z as f32)
    }
    fn is_alive(&self) -> bool {
        self.inner.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        if self.inner.destroy() {
            Ok(())
        } else {
            Err(CarlaError::Actor(ActorError::DestroyFailed(self.id)))
        }
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.inner.set_simulate_physics(enabled);
        Ok(())
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        let cxx_impulse = carla_cxx::SimpleVector3D {
            x: impulse.x as f64,
            y: impulse.y as f64,
            z: impulse.z as f64,
        };
        self.inner.add_impulse(&cxx_impulse);
        Ok(())
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        let cxx_force = carla_cxx::SimpleVector3D {
            x: force.x as f64,
            y: force.y as f64,
            z: force.z as f64,
        };
        self.inner.add_force(&cxx_force);
        Ok(())
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        let cxx_torque = carla_cxx::SimpleVector3D {
            x: torque.x as f64,
            y: torque.y as f64,
            z: torque.z as f64,
        };
        self.inner.add_torque(&cxx_torque);
        Ok(())
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
    /// Convert from carla-cxx TrafficLightState
    pub fn from_cxx(cxx_state: carla_cxx::TrafficLightState) -> Self {
        use carla_cxx::TrafficLightState as CxxState;
        match cxx_state {
            CxxState::Red => Self::Red,
            CxxState::Yellow => Self::Yellow,
            CxxState::Green => Self::Green,
            CxxState::Off => Self::Off,
            CxxState::Unknown => Self::Unknown,
        }
    }

    /// Convert to carla-cxx TrafficLightState
    pub fn to_cxx(&self) -> carla_cxx::TrafficLightState {
        use carla_cxx::TrafficLightState as CxxState;
        match self {
            Self::Red => CxxState::Red,
            Self::Yellow => CxxState::Yellow,
            Self::Green => CxxState::Green,
            Self::Off => CxxState::Off,
            Self::Unknown => CxxState::Unknown,
        }
    }
}
