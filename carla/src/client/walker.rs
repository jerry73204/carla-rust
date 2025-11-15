//! Walker (pedestrian) actor type for the CARLA client.

use super::{Actor, ActorBase};
use crate::rpc::{WalkerBoneControlIn, WalkerBoneControlOut, WalkerControl};
use autocxx::WithinBox;
use carla_sys::carla_rust::client::{FfiActor, FfiWalker};
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Represents a walker (pedestrian) in the simulation.
///
/// [`Walker`] provides methods for:
/// - Walker movement control (direction, speed)
/// - Jumping
/// - AI controller spawning and management
///
/// Corresponds to [`carla.Walker`](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Walker) in the Python API
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     client::{ActorBase, Client},
///     geom::Vector3D,
///     rpc::WalkerControl,
/// };
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// # let bp_lib = world.blueprint_library();
/// # let walker_bp = bp_lib.filter("walker.pedestrian.*").get(0).unwrap();
/// # let spawn_points = world.map().recommended_spawn_points();
/// # let actor = world.spawn_actor(&walker_bp, spawn_points.get(0).unwrap()).unwrap();
/// let walker: carla::client::Walker = actor.try_into().unwrap();
///
/// // Control the walker
/// let mut control = WalkerControl::default();
/// control.direction = Vector3D {
///     x: 1.0,
///     y: 0.0,
///     z: 0.0,
/// };
/// control.speed = 1.5; // m/s
/// control.jump = false;
/// walker.apply_control(&control);
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Walker {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiWalker>,
}

impl Walker {
    /// Applies walker control (direction, speed, jump).
    ///
    /// See [carla.Walker.apply_control](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Walker.apply_control)
    /// in the Python API.
    ///
    /// # Arguments
    ///
    /// * `control` - Walker control parameters
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let walker_bp = bp_lib.filter("walker.pedestrian.*").get(0).unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let actor = world.spawn_actor(&walker_bp, spawn_points.get(0).unwrap()).unwrap();
    /// # let walker: carla::client::Walker = actor.try_into().unwrap();
    /// use carla::{geom::Vector3D, rpc::WalkerControl};
    ///
    /// let mut control = WalkerControl::default();
    /// control.direction = Vector3D {
    ///     x: 1.0,
    ///     y: 0.0,
    ///     z: 0.0,
    /// };
    /// control.speed = 2.0;
    /// walker.apply_control(&control);
    /// ```
    pub fn apply_control(&self, control: &WalkerControl) {
        self.inner.ApplyControl(control);
    }

    /// Gets the current walker control state.
    ///
    /// See [carla.Walker.get_control](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Walker.get_control)
    /// in the Python API.
    ///
    /// Returns the last control applied to the walker.
    pub fn control(&self) -> WalkerControl {
        self.inner.GetWalkerControl()
    }

    /// Sets bone transforms for custom walker animations.
    ///
    /// See [carla.Walker.set_bones](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Walker.set_bones)
    /// in the Python API.
    ///
    /// Allows direct control over individual bones in the walker's skeleton.
    ///
    /// # Arguments
    ///
    /// * `bones` - Collection of bone transforms to apply
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let walker_bp = bp_lib.filter("walker.pedestrian.*").get(0).unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let actor = world.spawn_actor(&walker_bp, spawn_points.get(0).unwrap()).unwrap();
    /// # let walker: carla::client::Walker = actor.try_into().unwrap();
    /// use carla::{
    ///     geom::Transform,
    ///     rpc::{BoneTransformDataIn, WalkerBoneControlIn},
    /// };
    ///
    /// let bone_control = WalkerBoneControlIn {
    ///     bone_transforms: vec![BoneTransformDataIn {
    ///         bone_name: "crl_arm__L".to_string(),
    ///         transform: Transform::default(),
    ///     }],
    /// };
    /// walker.set_bones(&bone_control);
    /// ```
    pub fn set_bones(&self, bones: &WalkerBoneControlIn) {
        let ffi_bones = bones.to_ffi();
        self.inner.SetBonesTransformFfi(&ffi_bones);
    }

    /// Blends the current pose with the animation pose.
    ///
    /// See [carla.Walker.blend_pose](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Walker.blend_pose)
    /// in the Python API.
    ///
    /// # Arguments
    ///
    /// * `blend` - Blend factor (0.0 = animation pose, 1.0 = custom pose)
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let walker_bp = bp_lib.filter("walker.pedestrian.*").get(0).unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let actor = world.spawn_actor(&walker_bp, spawn_points.get(0).unwrap()).unwrap();
    /// # let walker: carla::client::Walker = actor.try_into().unwrap();
    /// // Blend 50% animation, 50% custom pose
    /// walker.blend_pose(0.5);
    /// ```
    pub fn blend_pose(&self, blend: f32) {
        self.inner.BlendPose(blend);
    }

    /// Shows the custom pose (blend factor 1.0).
    ///
    /// See [carla.Walker.show_pose](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Walker.show_pose)
    /// in the Python API.
    ///
    /// Equivalent to `blend_pose(1.0)`.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let walker_bp = bp_lib.filter("walker.pedestrian.*").get(0).unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let actor = world.spawn_actor(&walker_bp, spawn_points.get(0).unwrap()).unwrap();
    /// # let walker: carla::client::Walker = actor.try_into().unwrap();
    /// walker.show_pose();
    /// ```
    pub fn show_pose(&self) {
        self.blend_pose(1.0);
    }

    /// Hides the custom pose (blend factor 0.0).
    ///
    /// See [carla.Walker.hide_pose](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Walker.hide_pose)
    /// in the Python API.
    ///
    /// Equivalent to `blend_pose(0.0)`, returns to animation pose.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let walker_bp = bp_lib.filter("walker.pedestrian.*").get(0).unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let actor = world.spawn_actor(&walker_bp, spawn_points.get(0).unwrap()).unwrap();
    /// # let walker: carla::client::Walker = actor.try_into().unwrap();
    /// walker.hide_pose();
    /// ```
    pub fn hide_pose(&self) {
        self.blend_pose(0.0);
    }

    /// Gets the current bone transforms.
    ///
    /// See [carla.Walker.get_bones](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.Walker.get_bones)
    /// in the Python API.
    ///
    /// Returns the transforms of all bones in the walker's skeleton in different
    /// coordinate spaces (world, component, relative).
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let walker_bp = bp_lib.filter("walker.pedestrian.*").get(0).unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let actor = world.spawn_actor(&walker_bp, spawn_points.get(0).unwrap()).unwrap();
    /// # let walker: carla::client::Walker = actor.try_into().unwrap();
    /// let bone_transforms = walker.get_bones_transform();
    ///
    /// for bone in &bone_transforms.bone_transforms {
    ///     println!("Bone: {}", bone.bone_name);
    ///     println!("  World: {:?}", bone.world);
    ///     println!("  Component: {:?}", bone.component);
    ///     println!("  Relative: {:?}", bone.relative);
    /// }
    /// ```
    pub fn get_bones_transform(&self) -> WalkerBoneControlOut {
        let mut ffi_bones = self.inner.GetBonesTransformFfi().within_box();
        // ffi_bones is now Pin<Box<FfiWalkerBoneControlOut>>
        WalkerBoneControlOut::from_ffi(ffi_bones.as_mut())
    }

    #[allow(dead_code)]
    pub(crate) fn from_cxx(ptr: SharedPtr<FfiWalker>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }

    #[allow(dead_code)]
    pub(crate) fn cxx_walker(&self) -> SharedPtr<FfiWalker> {
        self.inner.clone()
    }
}

impl ActorBase for Walker {
    fn cxx_actor(&self) -> SharedPtr<FfiActor> {
        // SAFETY: FfiWalker can be safely cast to FfiActor (inheritance)
        unsafe { std::mem::transmute(self.inner.clone()) }
    }
}

impl TryFrom<Actor> for Walker {
    type Error = Actor;

    fn try_from(actor: Actor) -> Result<Self, Self::Error> {
        // Try to cast to walker
        let walker_ptr: SharedPtr<FfiWalker> =
            // SAFETY: We check if the cast is valid by checking if type_id contains "walker"
            unsafe { std::mem::transmute(actor.inner.clone()) };

        if actor.type_id().contains("walker") {
            Ok(Walker { inner: walker_ptr })
        } else {
            Err(actor)
        }
    }
}

assert_impl_all!(Walker: Send, Sync);
