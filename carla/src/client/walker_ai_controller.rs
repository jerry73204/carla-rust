use super::{Actor, ActorBase};
use crate::{
    geom::{Transform, Vector3D},
    utils::check_carla_error,
};
use anyhow::{anyhow, Result};
use carla_sys::*;

/// A walker AI controller for autonomous pedestrian navigation.
/// This is a newtype wrapper around Actor that provides AI controller-specific functionality.
#[derive(Clone, Debug)]
pub struct WalkerAIController(pub Actor);

impl WalkerAIController {
    /// Create a WalkerAIController from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null, and must point to a walker AI controller actor.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_actor_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null walker AI controller pointer"));
        }

        // Create the base Actor first
        let actor = Actor::from_raw_ptr(ptr)?;
        Ok(Self(actor))
    }

    /// Convert this WalkerAIController back into a generic Actor.
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

    /// Start the AI controller.
    pub fn start(&self) -> Result<()> {
        let error = unsafe { carla_walker_ai_start(self.0.raw_ptr()) };
        check_carla_error(error)
    }

    /// Stop the AI controller.
    pub fn stop(&self) -> Result<()> {
        let error = unsafe { carla_walker_ai_stop(self.0.raw_ptr()) };
        check_carla_error(error)
    }

    /// Set the maximum walking speed for the AI.
    pub fn set_max_speed(&self, max_speed: f32) -> Result<()> {
        let error = unsafe { carla_walker_ai_set_max_speed(self.0.raw_ptr(), max_speed) };
        check_carla_error(error)
    }

    /// Tell the AI to go to a specific location.
    pub fn go_to_location(&self, destination: Vector3D) -> Result<()> {
        let c_destination = carla_vector3d_t {
            x: destination.x,
            y: destination.y,
            z: destination.z,
        };
        let error = unsafe { carla_walker_ai_go_to_location(self.0.raw_ptr(), &c_destination) };
        check_carla_error(error)
    }

    /// Get a random location for AI navigation.
    pub fn get_random_location(&self) -> Result<Vector3D> {
        let mut c_location = carla_vector3d_t {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let error =
            unsafe { carla_walker_ai_get_random_location(self.0.raw_ptr(), &mut c_location) };
        check_carla_error(error)?;
        Ok(Vector3D {
            x: c_location.x,
            y: c_location.y,
            z: c_location.z,
        })
    }

    /// Destroy this walker AI controller.
    pub fn destroy(self) -> Result<()> {
        self.0.destroy()
    }
}

// Implement ActorBase trait for WalkerAIController
impl ActorBase for WalkerAIController {
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

// SAFETY: WalkerAIController wraps a thread-safe C API
unsafe impl Send for WalkerAIController {}
unsafe impl Sync for WalkerAIController {}
