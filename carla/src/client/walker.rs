//! Walker (pedestrian) actor type for the CARLA client.

use super::{Actor, ActorBase};
use crate::rpc::WalkerControl;
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
/// Corresponds to [`carla.Walker`](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Walker) in the Python API
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
    /// Returns the last control applied to the walker.
    pub fn control(&self) -> WalkerControl {
        self.inner.GetWalkerControl()
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiWalker>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }

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
