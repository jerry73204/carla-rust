//! Walker (pedestrian) actor implementation.

use crate::{
    client::{Actor, ActorId},
    error::CarlaResult,
    geom::{Transform, Vector3D},
    rpc::WalkerControl,
    traits::{ActorT, WalkerT},
};

/// Walker (pedestrian) actor.
#[derive(Debug)]
pub struct Walker {
    /// Base actor
    pub actor: Actor,
    // Additional walker-specific fields
}

impl Walker {
    /// Create a walker from a carla-cxx WalkerWrapper.
    /// TODO: Need Walker to Actor conversion in carla-cxx FFI
    pub fn new(_walker_wrapper: carla_cxx::WalkerWrapper) -> Self {
        todo!("Walker::new - need Walker to Actor conversion FFI function")
    }

    /// Create a walker from an actor.
    pub fn from_actor(actor: Actor) -> Self {
        Self { actor }
    }

    /// Get the underlying actor.
    pub fn as_actor(&self) -> &Actor {
        &self.actor
    }

    /// Get the current speed in m/s.
    pub fn get_speed(&self) -> f32 {
        self.get_velocity().length()
    }

    /// Make the walker go to a specific location.
    pub fn go_to_location(&self, destination: &crate::geom::Location) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _destination = destination;
        todo!("Walker::go_to_location not yet implemented with carla-cxx FFI")
    }

    /// Start walker navigation.
    pub fn start_navigation(&self) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Walker::start_navigation not yet implemented with carla-cxx FFI")
    }

    /// Stop walker navigation.
    pub fn stop_navigation(&self) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Walker::stop_navigation not yet implemented with carla-cxx FFI")
    }

    /// Set walker's maximum speed.
    pub fn set_max_speed(&self, speed: f32) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _speed = speed;
        todo!("Walker::set_max_speed not yet implemented with carla-cxx FFI")
    }
}

impl ActorT for Walker {
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

impl WalkerT for Walker {
    fn apply_control(&self, control: &WalkerControl) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _control = control;
        todo!("Walker::apply_control not yet implemented with carla-cxx FFI")
    }

    fn get_control(&self) -> WalkerControl {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Walker::get_control not yet implemented with carla-cxx FFI")
    }

    fn get_bones(&self) -> Vec<String> {
        // TODO: Implement using carla-cxx FFI interface
        todo!("Walker::get_bones not yet implemented with carla-cxx FFI")
    }

    fn set_bones(&self, bone_transforms: &[(String, Transform)]) -> CarlaResult<()> {
        // TODO: Implement using carla-cxx FFI interface
        let _bone_transforms = bone_transforms;
        todo!("Walker::set_bones not yet implemented with carla-cxx FFI")
    }
}
