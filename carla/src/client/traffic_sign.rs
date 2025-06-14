//! Traffic sign actor implementation.

use crate::{
    client::{Actor, ActorId},
    error::CarlaResult,
    geom::{Transform, Vector3D},
    traits::ActorT,
};

/// Traffic sign actor.
#[derive(Debug)]
pub struct TrafficSign {
    /// Base actor
    pub actor: Actor,
}

impl TrafficSign {
    /// Create a traffic sign from an actor.
    pub fn from_actor(actor: Actor) -> Self {
        Self { actor }
    }

    /// Get the underlying actor.
    pub fn as_actor(&self) -> &Actor {
        &self.actor
    }

    /// Get the traffic sign type.
    pub fn get_sign_type(&self) -> TrafficSignType {
        // TODO: Implement using carla-cxx FFI interface
        todo!("TrafficSign::get_sign_type not yet implemented with carla-cxx FFI")
    }

    /// Get the trigger volume for this traffic sign.
    pub fn get_trigger_volume(&self) -> crate::geom::BoundingBox {
        // TODO: Implement using carla-cxx FFI interface
        todo!("TrafficSign::get_trigger_volume not yet implemented with carla-cxx FFI")
    }

    /// Check if a location is within the sign's influence area.
    pub fn is_in_trigger_volume(&self, location: &crate::geom::Location) -> bool {
        // TODO: Implement using carla-cxx FFI interface
        let _location = location;
        todo!("TrafficSign::is_in_trigger_volume not yet implemented with carla-cxx FFI")
    }

    /// Get affected waypoints (for stop signs, etc.).
    pub fn get_affected_waypoints(&self) -> Vec<crate::road::Waypoint> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("TrafficSign::get_affected_waypoints not yet implemented with carla-cxx FFI")
    }
}

impl ActorT for TrafficSign {
    fn get_id(&self) -> ActorId {
        self.actor.get_id()
    }
    fn get_type_id(&self) -> String {
        self.actor.get_type_id()
    }
    fn get_transform(&self) -> Transform {
        self.actor.get_transform()
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        self.actor.set_transform(transform)
    }
    fn get_velocity(&self) -> Vector3D {
        self.actor.get_velocity()
    }
    fn get_angular_velocity(&self) -> Vector3D {
        self.actor.get_angular_velocity()
    }
    fn get_acceleration(&self) -> Vector3D {
        self.actor.get_acceleration()
    }
    fn is_alive(&self) -> bool {
        self.actor.is_alive()
    }
    fn destroy(&self) -> CarlaResult<()> {
        self.actor.destroy()
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        self.actor.set_simulate_physics(enabled)
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        self.actor.add_impulse(impulse)
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        self.actor.add_force(force)
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        self.actor.add_torque(torque)
    }
}

/// Traffic sign types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrafficSignType {
    /// Stop sign
    Stop,
    /// Yield sign
    Yield,
    /// Speed limit sign
    SpeedLimit,
    /// One way sign
    OneWay,
    /// No entry sign
    NoEntry,
    /// Parking sign
    Parking,
    /// Unknown sign type
    Unknown,
}

impl Default for TrafficSignType {
    fn default() -> Self {
        Self::Unknown
    }
}
