//! Base actor functionality.

use crate::{
    client::{ActorId, Sensor, Vehicle, Walker},
    error::CarlaResult,
    geom::{Transform, Vector3D},
    traits::ActorT,
};

/// Generic actor in the simulation.
#[derive(Debug)]
pub struct Actor {
    /// Actor ID
    pub id: ActorId,
    // Internal handle to carla-cxx Actor
    // This will be implemented when we integrate with carla-cxx
}

impl Actor {
    /// Try to cast this actor to a vehicle.
    pub fn as_vehicle(&self) -> Option<Vehicle> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::as_vehicle not yet implemented with carla-cxx FFI")
    }

    /// Try to cast this actor to a walker.
    pub fn as_walker(&self) -> Option<Walker> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::as_walker not yet implemented with carla-cxx FFI")
    }

    /// Try to cast this actor to a sensor.
    pub fn as_sensor(&self) -> Option<Sensor> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::as_sensor not yet implemented with carla-cxx FFI")
    }

    /// Get the actor's bounding box.
    pub fn get_bounding_box(&self) -> crate::geom::BoundingBox {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_bounding_box not yet implemented with carla-cxx FFI")
    }

    /// Get the actor's world bounding box.
    pub fn get_world_bounding_box(&self) -> crate::geom::BoundingBox {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_world_bounding_box not yet implemented with carla-cxx FFI")
    }

    /// Get semantic tags for this actor.
    pub fn get_semantic_tags(&self) -> Vec<u8> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_semantic_tags not yet implemented with carla-cxx FFI")
    }

    /// Check if actor is a specific type.
    pub fn is_type(&self, type_pattern: &str) -> bool {
        // TODO: Implement using carla-cxx FFI interface
        let _type_pattern = type_pattern;
        todo!("Actor::is_type not yet implemented with carla-cxx FFI")
    }
}

impl ActorT for Actor {
    fn get_id(&self) -> ActorId {
        self.id
    }

    fn get_type_id(&self) -> String {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_type_id not yet implemented with carla-cxx FFI")
    }

    fn get_transform(&self) -> Transform {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_transform not yet implemented with carla-cxx FFI")
    }

    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _transform = transform;
        todo!("Actor::set_transform not yet implemented with carla-cxx FFI")
    }

    fn get_velocity(&self) -> Vector3D {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_velocity not yet implemented with carla-cxx FFI")
    }

    fn get_angular_velocity(&self) -> Vector3D {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_angular_velocity not yet implemented with carla-cxx FFI")
    }

    fn get_acceleration(&self) -> Vector3D {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::get_acceleration not yet implemented with carla-cxx FFI")
    }

    fn is_alive(&self) -> bool {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::is_alive not yet implemented with carla-cxx FFI")
    }

    fn destroy(&self) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Actor::destroy not yet implemented with carla-cxx FFI")
    }

    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _enabled = enabled;
        todo!("Actor::set_simulate_physics not yet implemented with carla-cxx FFI")
    }

    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _impulse = impulse;
        todo!("Actor::add_impulse not yet implemented with carla-cxx FFI")
    }

    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _force = force;
        todo!("Actor::add_force not yet implemented with carla-cxx FFI")
    }

    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _torque = torque;
        todo!("Actor::add_torque not yet implemented with carla-cxx FFI")
    }
}

/// Snapshot of an actor's state.
#[derive(Debug, Clone)]
pub struct ActorSnapshot {
    /// Actor ID
    pub id: ActorId,
    /// Actor transform
    pub transform: Transform,
    /// Actor velocity
    pub velocity: Vector3D,
    /// Actor angular velocity
    pub angular_velocity: Vector3D,
    /// Actor acceleration
    pub acceleration: Vector3D,
}
