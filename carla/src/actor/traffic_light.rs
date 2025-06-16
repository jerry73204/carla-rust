//! Traffic light actor implementation.

use crate::{
    actor::{Actor, ActorId},
    error::CarlaResult,
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
    pub(crate) fn from_cxx(
        traffic_light_wrapper: TrafficLightWrapper,
        id: ActorId,
    ) -> CarlaResult<Self> {
        Ok(Self {
            id,
            inner: traffic_light_wrapper,
        })
    }

    /// Create a traffic light from an actor by casting.
    pub fn from_actor(actor: Actor) -> CarlaResult<Option<Self>> {
        let actor_ref = actor.get_inner_actor();
        let actor_id = actor.id();
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
    pub fn id(&self) -> ActorId {
        self.id
    }

    /// Get current traffic light state.
    pub fn state(&self) -> TrafficLightState {
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
    pub fn elapsed_time(&self) -> Duration {
        let elapsed_seconds = self.inner.get_elapsed_time();
        Duration::from_secs_f32(elapsed_seconds)
    }

    /// Get traffic light group (affected lanes).
    /// Returns waypoint information for lanes affected by this traffic light.
    pub fn affected_lane_waypoints(&self) -> Vec<crate::road::Waypoint> {
        // Call the wrapper method which has the FFI implementation
        let waypoint_infos = self.inner.get_affected_lane_waypoints();

        // Convert SimpleWaypointInfo to Waypoint
        waypoint_infos
            .into_iter()
            .map(|info| crate::road::Waypoint {
                transform: crate::geom::Transform::from(info.transform),
                lane_id: info.lane_id,
                section_id: info.section_id as i32,
                road_id: info.road_id as i32,
                junction_id: if info.is_junction { 0 } else { -1 }, // We don't have junction_id in SimpleWaypointInfo
                lane_width: info.lane_width as f32,
                lane_change: crate::road::LaneChange::from_u8(info.lane_change),
                lane_type: crate::road::LaneType::from_u8(info.lane_type as u8),
                lane_marking_type: crate::road::LaneMarkingType::Other, // Default, not provided
                lane_marking_color: crate::road::LaneMarkingColor::Standard, // Default, not provided
            })
            .collect()
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

impl ActorT for TrafficLight {
    fn id(&self) -> ActorId {
        self.id
    }
    fn type_id(&self) -> String {
        self.inner.get_type_id()
    }
    fn transform(&self) -> Transform {
        let cxx_transform = self.inner.get_transform();
        Transform::from(cxx_transform)
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        let cxx_transform = transform.to_cxx();
        self.inner.set_transform(&cxx_transform);
        Ok(())
    }
    fn velocity(&self) -> Vector3D {
        let vel = self.inner.get_velocity();
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn angular_velocity(&self) -> Vector3D {
        let vel = self.inner.get_angular_velocity();
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn acceleration(&self) -> Vector3D {
        let acc = self.inner.get_acceleration();
        Vector3D::new(acc.x as f32, acc.y as f32, acc.z as f32)
    }
    fn is_alive(&self) -> bool {
        self.inner.is_alive()
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

    fn bounding_box(&self) -> crate::geom::BoundingBox {
        // TrafficLight doesn't have a direct GetBoundingBox method, need to cast to Actor
        let actor_ptr =
            carla_cxx::ffi::TrafficLight_CastToActor(self.inner.get_inner_traffic_light());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast TrafficLight to Actor");
        }
        let simple_bbox = carla_cxx::ffi::Actor_GetBoundingBox(&actor_ptr);
        crate::geom::BoundingBox::from_cxx(simple_bbox)
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
