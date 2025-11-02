//! Actor snapshot for efficient state queries.
//!
//! [`ActorSnapshot`] provides a lightweight way to query actor state from a
//! [`WorldSnapshot`](super::WorldSnapshot) without requiring actor lookups.

use crate::{
    geom::{Transform, Vector3D},
    rpc::ActorId,
};
use carla_sys::carla_rust::{client::FfiActorSnapshot, geom::FfiVector3D};
use cxx::UniquePtr;
use derivative::Derivative;
use nalgebra::Vector3;
use static_assertions::assert_impl_all;

/// Snapshot of an actor's state at a specific moment in time.
///
/// Contains position, velocity, acceleration, and angular velocity for an actor
/// at a particular simulation frame. This is more efficient than querying actors
/// directly when you need to examine many actors' states simultaneously.
///
/// Corresponds to [`carla.ActorSnapshot`](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.ActorSnapshot) in the Python API.
///
/// # Examples
///
/// ```no_run
/// use carla::client::Client;
///
/// let client = Client::default();
/// let world = client.world();
///
/// // Get a world snapshot
/// let snapshot = world.wait_for_tick();
///
/// // Iterate over all actor snapshots
/// for actor_snapshot in snapshot.actor_snapshots() {
///     let id = actor_snapshot.id();
///     let velocity = actor_snapshot.velocity();
///     let speed = velocity.norm();
///
///     if speed > 10.0 {
///         println!("Actor {} is moving at {:.1} m/s", id, speed);
///     }
/// }
/// ```
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct ActorSnapshot {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: UniquePtr<FfiActorSnapshot>,
}

impl ActorSnapshot {
    /// Returns the actor's ID.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::actor_snapshot::ActorSnapshot;
    /// # let actor_snapshot: ActorSnapshot = todo!();
    /// let actor_id = actor_snapshot.id();
    /// println!("Actor ID: {}", actor_id);
    /// ```
    pub fn id(&self) -> ActorId {
        self.inner.GetId()
    }

    /// Returns the actor's transform (position and rotation).
    ///
    /// The transform is returned as a nalgebra `Isometry3<f32>`, which combines
    /// translation and rotation in a single type.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::actor_snapshot::ActorSnapshot;
    /// # let actor_snapshot: ActorSnapshot = todo!();
    /// let transform = actor_snapshot.transform();
    /// let position = transform.translation.vector;
    /// println!(
    ///     "Actor position: ({:.1}, {:.1}, {:.1})",
    ///     position.x, position.y, position.z
    /// );
    /// ```
    pub fn transform(&self) -> nalgebra::Isometry3<f32> {
        Transform::from_ffi(self.inner.GetTransform()).to_na()
    }

    /// Returns the actor's velocity vector in meters per second.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::actor_snapshot::ActorSnapshot;
    /// # let actor_snapshot: ActorSnapshot = todo!();
    /// let velocity = actor_snapshot.velocity();
    /// let speed = velocity.norm();
    /// println!("Actor speed: {:.1} m/s", speed);
    /// ```
    pub fn velocity(&self) -> Vector3<f32> {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        unsafe {
            let cpp_vec = self.inner.GetVelocity();
            Vector3D::from_ffi(std::mem::transmute::<
                carla_sys::carla::geom::Vector3D,
                FfiVector3D,
            >(cpp_vec))
            .to_na()
        }
    }

    /// Returns the actor's angular velocity vector in radians per second.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::actor_snapshot::ActorSnapshot;
    /// # let actor_snapshot: ActorSnapshot = todo!();
    /// let angular_velocity = actor_snapshot.angular_velocity();
    /// let rotation_speed = angular_velocity.norm();
    /// println!("Actor rotation speed: {:.2} rad/s", rotation_speed);
    /// ```
    pub fn angular_velocity(&self) -> Vector3<f32> {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        unsafe {
            let cpp_vec = self.inner.GetAngularVelocity();
            Vector3D::from_ffi(std::mem::transmute::<
                carla_sys::carla::geom::Vector3D,
                FfiVector3D,
            >(cpp_vec))
            .to_na()
        }
    }

    /// Returns the actor's acceleration vector in meters per second squared.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::actor_snapshot::ActorSnapshot;
    /// # let actor_snapshot: ActorSnapshot = todo!();
    /// let acceleration = actor_snapshot.acceleration();
    /// let accel_magnitude = acceleration.norm();
    /// println!("Actor acceleration: {:.2} m/s²", accel_magnitude);
    /// ```
    pub fn acceleration(&self) -> Vector3<f32> {
        // SAFETY: carla::geom::Vector3D and FfiVector3D have identical memory layout
        unsafe {
            let cpp_vec = self.inner.GetAcceleration();
            Vector3D::from_ffi(std::mem::transmute::<
                carla_sys::carla::geom::Vector3D,
                FfiVector3D,
            >(cpp_vec))
            .to_na()
        }
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiActorSnapshot>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

assert_impl_all!(ActorSnapshot: Send, Sync);
