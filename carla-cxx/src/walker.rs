//! Walker actor implementation for CARLA.

use crate::ffi::{self, Actor, SimpleVector3D, SimpleWalkerControl, Walker};
use anyhow::Result;
use cxx::SharedPtr;

/// High-level wrapper for CARLA Walker
pub struct WalkerWrapper {
    inner: SharedPtr<Walker>,
}

impl std::fmt::Debug for WalkerWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("WalkerWrapper")
            .field("inner", &"<SharedPtr<Walker>>")
            .finish()
    }
}

impl WalkerWrapper {
    /// Create a WalkerWrapper from an Actor (performs cast)
    pub fn from_actor(actor: &Actor) -> Option<Self> {
        let walker_ptr = ffi::Actor_CastToWalker(actor);
        if walker_ptr.is_null() {
            None
        } else {
            Some(Self { inner: walker_ptr })
        }
    }

    /// Convert back to an Actor
    pub fn to_actor(&self) -> SharedPtr<Actor> {
        ffi::Walker_CastToActor(&self.inner)
    }

    /// Get access to the inner Walker for direct FFI calls
    pub fn get_inner_walker(&self) -> &SharedPtr<Walker> {
        &self.inner
    }

    /// Apply walker control (direction and speed)
    pub fn apply_control(&self, control: &WalkerControl) -> Result<()> {
        let simple_control = SimpleWalkerControl {
            direction: SimpleVector3D {
                x: control.direction.x,
                y: control.direction.y,
                z: control.direction.z,
            },
            speed: control.speed,
            jump: control.jump,
        };
        ffi::Walker_ApplyControl(&self.inner, &simple_control);
        Ok(())
    }

    /// Get current walker control settings
    pub fn get_control(&self) -> WalkerControl {
        let simple_control = ffi::Walker_GetControl(&self.inner);
        WalkerControl {
            direction: Vector3D {
                x: simple_control.direction.x,
                y: simple_control.direction.y,
                z: simple_control.direction.z,
            },
            speed: simple_control.speed,
            jump: simple_control.jump,
        }
    }

    /// Get current walker speed in m/s
    pub fn get_speed(&self) -> f32 {
        ffi::Walker_GetSpeed(&self.inner)
    }

    // Note: Direct bone transform control is not available through CXX
    // Use blend_pose to control animation blending instead

    /// Blend between animation pose and custom pose
    /// @param blend: 0.0 = full animation, 1.0 = full custom pose
    pub fn blend_pose(&self, blend: f32) -> Result<()> {
        ffi::Walker_BlendPose(&self.inner, blend.clamp(0.0, 1.0));
        Ok(())
    }

    /// Show custom pose (blend = 1.0)
    pub fn show_pose(&self) -> Result<()> {
        ffi::Walker_ShowPose(&self.inner);
        Ok(())
    }

    /// Hide custom pose (blend = 0.0)
    pub fn hide_pose(&self) -> Result<()> {
        ffi::Walker_HidePose(&self.inner);
        Ok(())
    }

    /// Get pose from current animation frame
    pub fn get_pose_from_animation(&self) -> Result<()> {
        ffi::Walker_GetPoseFromAnimation(&self.inner);
        Ok(())
    }
}

/// Walker control structure
#[derive(Debug, Clone, PartialEq)]
pub struct WalkerControl {
    /// Direction vector for walker movement
    pub direction: Vector3D,
    /// Walking speed in m/s
    pub speed: f32,
    /// Jump action
    pub jump: bool,
}

/// 3D vector for walker direction
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for WalkerControl {
    fn default() -> Self {
        Self {
            direction: Vector3D {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            speed: 0.0,
            jump: false,
        }
    }
}

impl WalkerControl {
    /// Create a new WalkerControl with default values
    pub fn new() -> Self {
        Self::default()
    }

    /// Set direction vector
    pub fn direction(mut self, x: f64, y: f64, z: f64) -> Self {
        self.direction = Vector3D { x, y, z };
        self
    }

    /// Set walking speed
    pub fn speed(mut self, speed: f32) -> Self {
        self.speed = speed.max(0.0);
        self
    }

    /// Set jump action
    pub fn jump(mut self, jump: bool) -> Self {
        self.jump = jump;
        self
    }
}

impl Vector3D {
    /// Create a new Vector3D
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Normalize the vector
    pub fn normalized(&self) -> Self {
        let length = (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if length > 0.0 {
            Self {
                x: self.x / length,
                y: self.y / length,
                z: self.z / length,
            }
        } else {
            *self
        }
    }

    /// Get the length of the vector
    pub fn length(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

// Note: Bone transform functionality is limited in the CXX implementation
// Advanced bone control would require different FFI approach
