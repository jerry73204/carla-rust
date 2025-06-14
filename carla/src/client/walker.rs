use super::{Actor, ActorBase};
use crate::{
    geom::{Transform, Vector3D},
    utils::check_carla_error,
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ffi::CString;

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
        if !unsafe { carla_sys::carla_actor_is_walker(ptr) } {
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
        let error = unsafe { carla_walker_apply_control(self.0.raw_ptr(), &c_control) };
        check_carla_error(error)
    }

    /// Get the current walker control.
    pub fn get_control(&self) -> WalkerControl {
        let c_control =
            unsafe { carla_walker_get_control(self.0.raw_ptr() as *const carla_actor_t) };
        WalkerControl::from_c_control(c_control)
    }

    // AI controller functions are implemented on the AI controller actor itself,
    // not on the walker. Use World::spawn_actor to create a WalkerAIController
    // and link it to this walker.

    /// Set bone control for detailed animation.
    pub fn set_bones_transform(&self, bone_control: &WalkerBoneControl) -> Result<()> {
        let c_bone_control = bone_control.to_c_bone_control();
        let error = unsafe { carla_walker_set_bones_transform(self.0.raw_ptr(), &c_bone_control) };
        // Clean up allocated memory
        unsafe {
            if !c_bone_control.bone_transforms.is_null() {
                drop(Vec::from_raw_parts(
                    c_bone_control.bone_transforms as *mut carla_walker_bone_transform_t,
                    c_bone_control.bone_count,
                    c_bone_control.bone_count,
                ));
            }
        }
        check_carla_error(error)
    }

    /// Get bone transforms for current animation state.
    pub fn get_bones_transform(&self) -> Result<WalkerBoneControl> {
        let c_bone_control =
            unsafe { carla_walker_get_bones_transform(self.0.raw_ptr() as *const carla_actor_t) };
        let result = WalkerBoneControl::from_c_bone_control(c_bone_control);
        // Free the C memory
        unsafe {
            carla_walker_bone_control_free(&c_bone_control as *const _ as *mut _);
        }
        Ok(result)
    }

    /// Blend the pose with the animation.
    pub fn blend_pose(&self, blend: f32) -> Result<()> {
        let error = unsafe { carla_walker_blend_pose(self.0.raw_ptr(), blend) };
        check_carla_error(error)
    }

    /// Show the pose (blend weight = 1.0).
    pub fn show_pose(&self) -> Result<()> {
        let error = unsafe { carla_walker_show_pose(self.0.raw_ptr()) };
        check_carla_error(error)
    }

    /// Hide the pose (blend weight = 0.0).
    pub fn hide_pose(&self) -> Result<()> {
        let error = unsafe { carla_walker_hide_pose(self.0.raw_ptr()) };
        check_carla_error(error)
    }

    /// Get pose from animation.
    pub fn get_pose_from_animation(&self) -> Result<()> {
        let error = unsafe { carla_walker_get_pose_from_animation(self.0.raw_ptr()) };
        check_carla_error(error)
    }

    // Note: Walker state, speed limit, and physics simulation functions
    // are not available in the C API. Use the base Actor methods instead.

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
        carla_walker_control_t {
            direction: carla_vector3d_t {
                x: self.direction.x,
                y: self.direction.y,
                z: self.direction.z,
            },
            speed: self.speed,
            jump: self.jump,
        }
    }

    /// Create from C control structure.
    pub(crate) fn from_c_control(c_control: carla_walker_control_t) -> Self {
        Self {
            direction: Vector3D {
                x: c_control.direction.x,
                y: c_control.direction.y,
                z: c_control.direction.z,
            },
            speed: c_control.speed,
            jump: c_control.jump,
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

    /// Convert to C bone control structure.
    pub(crate) fn to_c_bone_control(&self) -> carla_walker_bone_control_t {
        let mut bone_transforms = Vec::with_capacity(self.bone_transforms.len());

        for bone in &self.bone_transforms {
            let bone_name_cstr = CString::new(bone.bone_name.clone()).unwrap_or_default();
            let mut c_bone = carla_walker_bone_transform_t {
                transform: carla_transform_t {
                    location: carla_vector3d_t {
                        x: bone.transform.location.x,
                        y: bone.transform.location.y,
                        z: bone.transform.location.z,
                    },
                    rotation: carla_rotation_t {
                        pitch: bone.transform.rotation.pitch,
                        yaw: bone.transform.rotation.yaw,
                        roll: bone.transform.rotation.roll,
                    },
                },
                bone_name: [0; 64],
            };

            // Copy bone name
            let name_bytes = bone_name_cstr.as_bytes();
            let copy_len = std::cmp::min(name_bytes.len(), 63);
            // Convert u8 to i8 for bone_name array
            for (i, &byte) in name_bytes[..copy_len].iter().enumerate() {
                c_bone.bone_name[i] = byte as i8;
            }

            bone_transforms.push(c_bone);
        }

        let bone_ptr = if bone_transforms.is_empty() {
            std::ptr::null_mut()
        } else {
            bone_transforms.as_mut_ptr()
        };

        // Leak the vector so the pointer remains valid
        std::mem::forget(bone_transforms);

        carla_walker_bone_control_t {
            bone_transforms: bone_ptr,
            bone_count: self.bone_transforms.len(),
            blend_weight: self.blend_weight,
        }
    }

    /// Create from C bone control structure.
    pub(crate) fn from_c_bone_control(c_bone_control: carla_walker_bone_control_t) -> Self {
        let mut bone_transforms = Vec::new();

        if !c_bone_control.bone_transforms.is_null() && c_bone_control.bone_count > 0 {
            let bone_slice = unsafe {
                std::slice::from_raw_parts(
                    c_bone_control.bone_transforms,
                    c_bone_control.bone_count,
                )
            };

            for c_bone in bone_slice {
                let bone_name = unsafe {
                    std::ffi::CStr::from_ptr(c_bone.bone_name.as_ptr())
                        .to_string_lossy()
                        .into_owned()
                };

                let transform = Transform {
                    location: Vector3D {
                        x: c_bone.transform.location.x,
                        y: c_bone.transform.location.y,
                        z: c_bone.transform.location.z,
                    },
                    rotation: crate::geom::Rotation {
                        pitch: c_bone.transform.rotation.pitch,
                        yaw: c_bone.transform.rotation.yaw,
                        roll: c_bone.transform.rotation.roll,
                    },
                };

                bone_transforms.push(BoneTransform::new(bone_name, transform));
            }
        }

        Self {
            bone_transforms,
            blend_weight: c_bone_control.blend_weight,
            animation_mode: AnimationMode::Additive, // Default mode
        }
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
