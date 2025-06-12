use super::ActorBase;
use crate::{
    geom::{Transform, Vector3D},
    utils::check_carla_error,
};
use anyhow::{anyhow, Result};
use carla_sys::*;

use crate::stubs::carla_walker_control_t;
use std::ptr;

/// A walker (pedestrian) actor in the simulation.
#[derive(Clone, Debug)]
pub struct Walker {
    pub(crate) inner: *mut carla_actor_t,
}

impl Walker {
    /// Create a Walker from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_actor_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null walker pointer"));
        }
        Ok(Self { inner: ptr })
    }

    /// Get the walker's unique ID.
    pub fn id(&self) -> u32 {
        unsafe { carla_actor_get_id(self.inner) }
    }

    /// Get the walker's type ID.
    pub fn type_id(&self) -> String {
        let type_id_ptr = unsafe { carla_actor_get_type_id(self.inner) };
        unsafe { crate::utils::c_string_to_rust(type_id_ptr) }
    }

    /// Get the walker's current transform.
    pub fn get_transform(&self) -> Transform {
        let c_transform = unsafe { carla_actor_get_transform(self.inner) };
        Transform::from_c_transform(c_transform)
    }

    /// Set the walker's transform.
    pub fn set_transform(&self, transform: &Transform) -> Result<()> {
        let c_transform = transform.to_c_transform();
        let error = unsafe { carla_actor_set_transform(self.inner, &c_transform) };
        check_carla_error(error)
    }

    /// Get the walker's current velocity.
    pub fn get_velocity(&self) -> Vector3D {
        let c_velocity = unsafe { carla_actor_get_velocity(self.inner) };
        Vector3D::from_c_vector(c_velocity)
    }

    /// Apply control to the walker.
    pub fn apply_control(&self, _control: &WalkerControl) -> Result<()> {
        // TODO: Implement when carla_walker_apply_control is available
        todo!("Walker control not yet implemented in C API")
    }

    /// Check if the walker is still alive in the simulation.
    pub fn is_alive(&self) -> bool {
        unsafe { carla_actor_is_alive(self.inner) }
    }

    /// Destroy this walker.
    pub fn destroy(self) -> Result<()> {
        let error = unsafe { carla_actor_destroy(self.inner) };
        check_carla_error(error)
    }

    // TODO: Add methods for:
    // - get_walker_control() -> WalkerControl
    // - set_walker_ai_controller() for AI navigation
    // - get_bone_transforms() for animation control
    // - set_bone_control() for detailed animation
    // - get_walker_state() for current state
}

/// Walker control parameters.
#[derive(Clone, Debug, Default)]
pub struct WalkerControl {
    /// Walking direction as a 3D vector.
    pub direction: Vector3D,
    /// Walking speed (0.0 to 1.0).
    pub speed: f32,
    /// Whether the walker should jump.
    pub jump: bool,
}

impl WalkerControl {
    /// Create a new walker control.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the walking direction.
    pub fn direction(mut self, direction: Vector3D) -> Self {
        self.direction = direction;
        self
    }

    /// Set the walking speed.
    pub fn speed(mut self, speed: f32) -> Self {
        self.speed = speed.clamp(0.0, 1.0);
        self
    }

    /// Set whether the walker should jump.
    pub fn jump(mut self, jump: bool) -> Self {
        self.jump = jump;
        self
    }

    /// Convert to C control structure.
    pub(crate) fn to_c_control(&self) -> carla_walker_control_t {
        // TODO: Implement proper walker control conversion when C API is available
        carla_walker_control_t {
            throttle: self.speed,
            steer: 0.0, // Not applicable to walker
            brake: 0.0,
            hand_brake: false,
            reverse: false,
            manual_gear_shift: false,
            gear: 0,
        }
    }

    /// Create from C control structure.
    pub(crate) fn from_c_control(c_control: carla_walker_control_t) -> Self {
        // TODO: Implement proper walker control conversion when C API is available
        Self {
            direction: Vector3D::default(), // Can't extract from vehicle control
            speed: c_control.throttle,
            jump: false, // Not available in vehicle control
        }
    }
}

impl ActorBase for Walker {
    fn raw_ptr(&self) -> *mut carla_actor_t {
        self.inner
    }
}

impl Drop for Walker {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_actor_destroy(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

// SAFETY: Walker wraps a thread-safe C API
unsafe impl Send for Walker {}
unsafe impl Sync for Walker {}

// TODO: Implement additional walker types:
// - WalkerAIController for autonomous navigation
// - WalkerBoneControl for detailed animation
// - WalkerNavigation for pathfinding
// - WalkerState for current walker status
