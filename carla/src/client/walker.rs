use super::{Actor, ActorBase};
use crate::{
    geom::{Transform, Vector3D},
    stubs::{
        carla_actor_is_walker, carla_walker_ai_controller_t, carla_walker_apply_control,
        carla_walker_control_t, carla_walker_get_ai_controller, carla_walker_get_control,
        carla_walker_get_speed_limit, carla_walker_get_state, carla_walker_is_simulate_physics,
        carla_walker_set_ai_controller, carla_walker_set_simulate_physics,
        carla_walker_set_speed_limit, carla_walker_state_t, carla_walker_t,
    },
    utils::check_carla_error,
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
    pub fn apply_control(&self, control: &WalkerControl) -> Result<()> {
        let c_control = control.to_c_control();
        let error = unsafe {
            carla_walker_apply_control(self.0.raw_ptr() as *mut carla_walker_t, &c_control)
        };
        check_carla_error(error)
    }

    /// Get the current walker control.
    pub fn get_control(&self) -> WalkerControl {
        let c_control =
            unsafe { carla_walker_get_control(self.0.raw_ptr() as *const carla_walker_t) };
        WalkerControl::from_c_control(c_control)
    }

    /// Set the walker's AI controller.
    pub fn set_ai_controller(&self, controller: &WalkerAIController) -> Result<()> {
        let c_controller = controller.to_c_controller();
        let error = unsafe {
            carla_walker_set_ai_controller(self.0.raw_ptr() as *mut carla_walker_t, c_controller)
        };
        check_carla_error(error)
    }

    /// Get the walker's AI controller settings.
    pub fn get_ai_controller(&self) -> Result<WalkerAIController> {
        let c_controller =
            unsafe { carla_walker_get_ai_controller(self.0.raw_ptr() as *const carla_walker_t) };
        if c_controller.is_null() {
            return Err(anyhow!("Walker has no AI controller"));
        }
        Ok(WalkerAIController::from_c_controller(c_controller))
    }

    /// Set bone control for detailed animation.
    pub fn set_bone_control(&self, bone_control: &WalkerBoneControl) -> Result<()> {
        // TODO: Implement when C API provides carla_walker_set_bone_control
        Err(anyhow!("Walker bone control not yet implemented in C API"))
    }

    /// Get bone transforms for current animation state.
    pub fn get_bone_transforms(&self) -> Result<Vec<BoneTransform>> {
        // TODO: Implement when C API provides carla_walker_get_bone_transforms
        Err(anyhow!(
            "Walker bone transforms not yet implemented in C API"
        ))
    }

    /// Get the walker's current state.
    pub fn get_state(&self) -> WalkerState {
        let c_state = unsafe { carla_walker_get_state(self.0.raw_ptr() as *const carla_walker_t) };
        WalkerState::from_c_state(c_state)
    }

    /// Set the walker's speed limit.
    pub fn set_speed_limit(&self, speed_limit: f32) -> Result<()> {
        let error = unsafe {
            carla_walker_set_speed_limit(self.0.raw_ptr() as *mut carla_walker_t, speed_limit)
        };
        check_carla_error(error)
    }

    /// Get the walker's speed limit.
    pub fn get_speed_limit(&self) -> f32 {
        unsafe { carla_walker_get_speed_limit(self.0.raw_ptr() as *const carla_walker_t) }
    }

    /// Enable or disable walker physics.
    pub fn set_simulate_physics(&self, enabled: bool) -> Result<()> {
        let error = unsafe {
            carla_walker_set_simulate_physics(self.0.raw_ptr() as *mut carla_walker_t, enabled)
        };
        check_carla_error(error)
    }

    /// Check if walker physics is enabled.
    pub fn is_simulate_physics(&self) -> bool {
        unsafe { carla_walker_is_simulate_physics(self.0.raw_ptr() as *const carla_walker_t) }
    }

    /// Destroy this walker.
    pub fn destroy(self) -> Result<()> {
        self.0.destroy()
    }
}

/// Walker control parameters for direct movement control.
#[derive(Clone, Debug, Default)]
pub struct WalkerControl {
    /// Walking direction as a 3D vector (normalized).
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

/// Walker AI controller for autonomous navigation.
#[derive(Clone, Debug)]
pub struct WalkerAIController {
    /// Target destination for AI navigation.
    pub target_location: Vector3D,
    /// Maximum walking speed for AI.
    pub max_speed: f32,
    /// Whether AI should avoid obstacles.
    pub avoid_obstacles: bool,
    /// Navigation precision (0.0 to 1.0).
    pub precision: f32,
}

impl WalkerAIController {
    /// Create a new AI controller with default settings.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the target location for AI navigation.
    pub fn target_location(mut self, location: Vector3D) -> Self {
        self.target_location = location;
        self
    }

    /// Set the maximum speed for AI movement.
    pub fn max_speed(mut self, speed: f32) -> Self {
        self.max_speed = speed.clamp(0.0, 10.0);
        self
    }

    /// Enable or disable obstacle avoidance.
    pub fn avoid_obstacles(mut self, avoid: bool) -> Self {
        self.avoid_obstacles = avoid;
        self
    }

    /// Set navigation precision.
    pub fn precision(mut self, precision: f32) -> Self {
        self.precision = precision.clamp(0.0, 1.0);
        self
    }

    /// Convert to C controller structure.
    /// TODO: Implement when C API provides proper walker AI controller
    pub(crate) fn to_c_controller(&self) -> *mut carla_walker_ai_controller_t {
        // TODO: Implement conversion when C API is available
        std::ptr::null_mut()
    }

    /// Create from C controller structure.
    /// TODO: Implement when C API provides proper walker AI controller
    pub(crate) fn from_c_controller(_c_controller: *mut carla_walker_ai_controller_t) -> Self {
        // TODO: Implement conversion when C API is available
        Self::default()
    }
}

impl Default for WalkerAIController {
    fn default() -> Self {
        Self {
            target_location: Vector3D::default(),
            max_speed: 1.4, // Average human walking speed
            avoid_obstacles: true,
            precision: 0.5,
        }
    }
}

/// Walker bone control for detailed animation.
#[derive(Clone, Debug)]
pub struct WalkerBoneControl {
    /// Bone transforms for animation.
    pub bone_transforms: Vec<BoneTransform>,
    /// Blend weight for animation mixing.
    pub blend_weight: f32,
    /// Animation mode.
    pub animation_mode: AnimationMode,
}

impl WalkerBoneControl {
    /// Create a new bone control.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set bone transforms.
    pub fn bone_transforms(mut self, transforms: Vec<BoneTransform>) -> Self {
        self.bone_transforms = transforms;
        self
    }

    /// Set blend weight.
    pub fn blend_weight(mut self, weight: f32) -> Self {
        self.blend_weight = weight.clamp(0.0, 1.0);
        self
    }

    /// Set animation mode.
    pub fn animation_mode(mut self, mode: AnimationMode) -> Self {
        self.animation_mode = mode;
        self
    }
}

impl Default for WalkerBoneControl {
    fn default() -> Self {
        Self {
            bone_transforms: Vec::new(),
            blend_weight: 1.0,
            animation_mode: AnimationMode::Additive,
        }
    }
}

/// Individual bone transform for animation.
#[derive(Clone, Debug)]
pub struct BoneTransform {
    /// Bone name identifier.
    pub bone_name: String,
    /// Bone transform.
    pub transform: Transform,
    /// Whether this bone is enabled.
    pub enabled: bool,
}

impl BoneTransform {
    /// Create a new bone transform.
    pub fn new(bone_name: String, transform: Transform) -> Self {
        Self {
            bone_name,
            transform,
            enabled: true,
        }
    }
}

/// Animation blending mode.
#[derive(Clone, Debug, PartialEq)]
pub enum AnimationMode {
    /// Replace existing animation.
    Replace,
    /// Add to existing animation.
    Additive,
    /// Blend with existing animation.
    Blend,
}

/// Walker current state information.
#[derive(Clone, Debug)]
pub struct WalkerState {
    /// Current movement speed.
    pub speed: f32,
    /// Current movement direction.
    pub direction: Vector3D,
    /// Whether walker is on ground.
    pub is_on_ground: bool,
    /// Whether walker is jumping.
    pub is_jumping: bool,
    /// Whether walker is controlled by AI.
    pub has_ai_controller: bool,
    /// Current animation state.
    pub animation_state: AnimationState,
}

impl WalkerState {
    /// Create from C state structure.
    /// TODO: Implement when C API provides carla_walker_state_t
    pub(crate) fn from_c_state(_c_state: carla_walker_state_t) -> Self {
        // TODO: Implement conversion when C API is available
        Self::default()
    }
}

impl Default for WalkerState {
    fn default() -> Self {
        Self {
            speed: 0.0,
            direction: Vector3D::default(),
            is_on_ground: true,
            is_jumping: false,
            has_ai_controller: false,
            animation_state: AnimationState::Idle,
        }
    }
}

/// Walker animation state.
#[derive(Clone, Debug, PartialEq)]
pub enum AnimationState {
    /// Walker is standing still.
    Idle,
    /// Walker is walking.
    Walking,
    /// Walker is running.
    Running,
    /// Walker is jumping.
    Jumping,
    /// Walker is falling.
    Falling,
    /// Custom animation.
    Custom(String),
}
