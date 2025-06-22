//! Walker (pedestrian) actor implementation.

use crate::{
    actor::{Actor, ActorFfi},
    error::CarlaResult,
    geom::{Transform, Vector3D},
};
use carla_sys::WalkerWrapper;

/// Walker (pedestrian) actor.
#[derive(Debug)]
pub struct Walker {
    /// Internal walker wrapper for FFI calls
    inner: WalkerWrapper,
}

impl Walker {
    /// Create a walker from a carla-sys WalkerWrapper and actor ID.
    pub(crate) fn from_cxx(walker_wrapper: WalkerWrapper) -> CarlaResult<Self> {
        Ok(Self {
            inner: walker_wrapper,
        })
    }

    /// Create a walker from an actor by casting.
    pub fn from_actor(actor: Actor) -> Result<Self, Actor> {
        let actor_ref = actor.inner_actor();
        if let Some(walker_wrapper) = WalkerWrapper::from_actor(actor_ref) {
            Ok(Self {
                inner: walker_wrapper,
            })
        } else {
            Err(actor)
        }
    }

    /// Get the walker's actor ID.

    /// Get the current speed in m/s.
    pub fn speed(&self) -> f32 {
        self.inner.get_speed()
    }

    /// Note about navigation functionality.
    ///
    /// Navigation functions like go_to_location, start/stop navigation, and set_max_speed
    /// are provided by WalkerAIController, not the Walker itself. WalkerAIController
    /// is a separate actor that needs to be spawned independently using World::spawn_actor
    /// with a walker AI controller blueprint.
    ///
    /// The WalkerAIController will then control this Walker's movement automatically.

    /// Blend between animation pose and custom pose.
    pub fn blend_pose(&self, blend: f32) -> CarlaResult<()> {
        self.inner.blend_pose(blend).map_err(|e| {
            crate::error::CarlaError::Walker(crate::error::WalkerError::PoseControlFailed(
                e.to_string(),
            ))
        })
    }

    /// Show custom pose (blend = 1.0).
    pub fn show_pose(&self) -> CarlaResult<()> {
        self.inner.show_pose().map_err(|e| {
            crate::error::CarlaError::Walker(crate::error::WalkerError::PoseControlFailed(
                e.to_string(),
            ))
        })
    }

    /// Hide custom pose (blend = 0.0).
    pub fn hide_pose(&self) -> CarlaResult<()> {
        self.inner.hide_pose().map_err(|e| {
            crate::error::CarlaError::Walker(crate::error::WalkerError::PoseControlFailed(
                e.to_string(),
            ))
        })
    }

    /// Get pose from current animation frame.
    pub fn pose_from_animation(&self) -> CarlaResult<()> {
        self.inner.get_pose_from_animation().map_err(|e| {
            crate::error::CarlaError::Walker(crate::error::WalkerError::PoseControlFailed(
                e.to_string(),
            ))
        })
    }
}

impl ActorFfi for Walker {
    fn as_actor_ffi(&self) -> &carla_sys::ActorWrapper {
        todo!()
    }
}

impl Drop for Walker {
    fn drop(&mut self) {
        // Only destroy if the walker is still alive
        if carla_sys::ffi::Walker_IsAlive(self.inner.get_inner_walker()) {
            let _ = carla_sys::ffi::Walker_Destroy(self.inner.get_inner_walker());
        }
    }
}

impl Walker {
    pub fn apply_control(&self, control: &WalkerControl) -> CarlaResult<()> {
        // Convert high-level WalkerControl to carla-sys WalkerControl
        let cxx_control = carla_sys::walker::WalkerControl {
            direction: carla_sys::walker::Vector3D {
                x: control.direction.x as f64,
                y: control.direction.y as f64,
                z: control.direction.z as f64,
            },
            speed: control.speed,
            jump: control.jump,
        };
        self.inner.apply_control(&cxx_control).map_err(|e| {
            crate::error::CarlaError::Walker(crate::error::WalkerError::ControlFailed(
                e.to_string(),
            ))
        })
    }

    pub fn control(&self) -> WalkerControl {
        let cxx_control = self.inner.get_control();
        WalkerControl {
            direction: crate::geom::Vector3D::new(
                cxx_control.direction.x as f32,
                cxx_control.direction.y as f32,
                cxx_control.direction.z as f32,
            ),
            speed: cxx_control.speed,
            jump: cxx_control.jump,
        }
    }

    pub fn bones(&self) -> Vec<String> {
        // Bone control is available through WalkerWrapper but limited
        // Return an empty list as the CXX implementation doesn't expose bone names
        todo!("Walker::bones requires advanced bone FFI not implemented in CXX layer")
    }

    pub fn set_bones(&self, bone_transforms: &[(String, Transform)]) -> CarlaResult<()> {
        // Bone control is available through blend_pose but not individual bone transforms
        let _bone_transforms = bone_transforms;
        todo!("Walker::set_bones requires advanced bone FFI not implemented in CXX layer")
    }
}

/// Walker (pedestrian) control commands.
#[derive(Debug, Clone, PartialEq)]
pub struct WalkerControl {
    /// Direction vector (normalized)
    pub direction: Vector3D,
    /// Speed in m/s
    pub speed: f32,
    /// Jump command
    pub jump: bool,
}

impl Default for WalkerControl {
    fn default() -> Self {
        Self {
            direction: Vector3D::zero(),
            speed: 0.0,
            jump: false,
        }
    }
}
