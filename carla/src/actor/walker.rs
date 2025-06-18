//! Walker (pedestrian) actor implementation.

use crate::{
    actor::{Actor, ActorId},
    error::CarlaResult,
    geom::{FromCxx, ToCxx, Transform, Vector3D},
    traits::{ActorT, WalkerT},
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
    pub fn from_cxx(walker_wrapper: WalkerWrapper) -> CarlaResult<Self> {
        Ok(Self {
            inner: walker_wrapper,
        })
    }

    /// Create a walker from an actor by casting.
    pub fn from_actor(actor: Actor) -> CarlaResult<Option<Self>> {
        let actor_ref = actor.get_inner_actor();
        if let Some(walker_wrapper) = WalkerWrapper::from_actor(actor_ref) {
            Ok(Some(Self {
                inner: walker_wrapper,
            }))
        } else {
            Ok(None)
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

    /// Set walker's maximum speed.
    ///
    /// **Note**: In CARLA, speed control is done through WalkerControl.speed in apply_control().
    /// There is no separate max_speed API on the Walker class itself.
    /// Use apply_control() with the desired speed instead.
    #[deprecated(note = "Use apply_control() with WalkerControl::speed instead")]
    pub fn set_max_speed(&self, _speed: f32) -> CarlaResult<()> {
        Err(crate::error::CarlaError::Walker(crate::error::WalkerError::ControlFailed(
            "set_max_speed not available on Walker - use apply_control() with WalkerControl::speed".to_string()
        )))
    }

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

impl ActorT for Walker {
    fn id(&self) -> ActorId {
        // Walker doesn't have a direct GetId method, need to cast to Actor
        let actor_ptr = carla_sys::ffi::Walker_CastToActor(self.inner.get_inner_walker());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Walker to Actor");
        }
        carla_sys::ffi::Actor_GetId(&actor_ptr)
    }
    fn type_id(&self) -> String {
        carla_sys::ffi::Walker_GetTypeId(self.inner.get_inner_walker())
    }
    fn transform(&self) -> Transform {
        let cxx_transform = carla_sys::ffi::Walker_GetTransform(self.inner.get_inner_walker());
        Transform::from(cxx_transform)
    }
    fn set_transform(&self, transform: &Transform) -> CarlaResult<()> {
        let cxx_transform = transform.to_cxx();
        carla_sys::ffi::Walker_SetTransform(self.inner.get_inner_walker(), &cxx_transform);
        Ok(())
    }
    fn velocity(&self) -> Vector3D {
        let vel = carla_sys::ffi::Walker_GetVelocity(self.inner.get_inner_walker());
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn angular_velocity(&self) -> Vector3D {
        let vel = carla_sys::ffi::Walker_GetAngularVelocity(self.inner.get_inner_walker());
        Vector3D::new(vel.x as f32, vel.y as f32, vel.z as f32)
    }
    fn acceleration(&self) -> Vector3D {
        let acc = carla_sys::ffi::Walker_GetAcceleration(self.inner.get_inner_walker());
        Vector3D::new(acc.x as f32, acc.y as f32, acc.z as f32)
    }
    fn is_alive(&self) -> bool {
        carla_sys::ffi::Walker_IsAlive(self.inner.get_inner_walker())
    }
    fn set_simulate_physics(&self, enabled: bool) -> CarlaResult<()> {
        carla_sys::ffi::Walker_SetSimulatePhysics(self.inner.get_inner_walker(), enabled);
        Ok(())
    }
    fn add_impulse(&self, impulse: &Vector3D) -> CarlaResult<()> {
        let cxx_impulse = carla_sys::SimpleVector3D {
            x: impulse.x as f64,
            y: impulse.y as f64,
            z: impulse.z as f64,
        };
        carla_sys::ffi::Walker_AddImpulse(self.inner.get_inner_walker(), &cxx_impulse);
        Ok(())
    }
    fn add_force(&self, force: &Vector3D) -> CarlaResult<()> {
        let cxx_force = carla_sys::SimpleVector3D {
            x: force.x as f64,
            y: force.y as f64,
            z: force.z as f64,
        };
        carla_sys::ffi::Walker_AddForce(self.inner.get_inner_walker(), &cxx_force);
        Ok(())
    }
    fn add_torque(&self, torque: &Vector3D) -> CarlaResult<()> {
        let cxx_torque = carla_sys::SimpleVector3D {
            x: torque.x as f64,
            y: torque.y as f64,
            z: torque.z as f64,
        };
        carla_sys::ffi::Walker_AddTorque(self.inner.get_inner_walker(), &cxx_torque);
        Ok(())
    }

    fn bounding_box(&self) -> crate::geom::BoundingBox {
        // Walker doesn't have a direct GetBoundingBox method, need to cast to Actor
        let actor_ptr = carla_sys::ffi::Walker_CastToActor(self.inner.get_inner_walker());
        if actor_ptr.is_null() {
            panic!("Internal error: Failed to cast Walker to Actor");
        }
        let simple_bbox = carla_sys::ffi::Actor_GetBoundingBox(&actor_ptr);
        crate::geom::BoundingBox::from_cxx(simple_bbox)
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

impl WalkerT for Walker {
    fn apply_control(&self, control: &WalkerControl) -> CarlaResult<()> {
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

    fn control(&self) -> WalkerControl {
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

    fn bones(&self) -> Vec<String> {
        // Bone control is available through WalkerWrapper but limited
        // Return an empty list as the CXX implementation doesn't expose bone names
        todo!("Walker::bones requires advanced bone FFI not implemented in CXX layer")
    }

    fn set_bones(&self, bone_transforms: &[(String, Transform)]) -> CarlaResult<()> {
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
