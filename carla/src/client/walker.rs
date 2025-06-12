use super::{Actor, ActorBase};
use crate::{
    geom::{Transform, Vector3D},
    stubs::{carla_actor_is_walker, carla_walker_control_t},
};
use anyhow::{anyhow, Result};
use carla_sys::*;

/// A walker (pedestrian) actor in the simulation.
/// This is a newtype wrapper around Actor that provides walker-specific functionality.
#[derive(Clone, Debug)]
pub struct Walker(pub Actor);

impl Walker {
    /// Create a Walker from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null, and must point to a walker actor.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_actor_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null walker pointer"));
        }

        // Verify it's actually a walker
        if !unsafe { carla_actor_is_walker(ptr) } {
            return Err(anyhow!("Actor is not a walker"));
        }

        // Create the base Actor first
        let actor = Actor::from_raw_ptr(ptr)?;
        Ok(Self(actor))
    }

    /// Convert this Walker back into a generic Actor.
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

    /// Get the walker's current velocity.
    pub fn get_velocity(&self) -> Vector3D {
        self.0.get_velocity()
    }

    /// Apply control to the walker.
    pub fn apply_control(&self, _control: &WalkerControl) -> Result<()> {
        // TODO: Implement when carla_walker_apply_control is available
        todo!("Walker control not yet implemented in C API")
    }

    /// Destroy this walker.
    pub fn destroy(self) -> Result<()> {
        self.0.destroy()
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

// Implement ActorBase trait for Walker
impl ActorBase for Walker {
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

// Note: Walker doesn't implement Drop because it's a newtype wrapper around Actor,
// and the underlying carla_actor_t will be freed when the Actor is dropped

// SAFETY: Walker wraps a thread-safe C API
unsafe impl Send for Walker {}
unsafe impl Sync for Walker {}

// TODO: Implement additional walker types:
// - WalkerAIController for autonomous navigation
// - WalkerBoneControl for detailed animation
// - WalkerNavigation for pathfinding
// - WalkerState for current walker status
