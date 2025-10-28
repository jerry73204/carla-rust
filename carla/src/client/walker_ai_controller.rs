//! Walker AI controller for autonomous pedestrian navigation.

use super::{Actor, ActorBase};
use carla_sys::carla_rust::client::{FfiActor, FfiWalkerAIController};
use cxx::SharedPtr;
use derivative::Derivative;

/// AI controller for autonomous walker navigation.
///
/// [`WalkerAIController`] provides methods for:
/// - Starting/stopping AI control
/// - Navigating to specific locations
/// - Setting maximum walking speed
/// - Getting random navigation targets
///
/// Corresponds to [`carla.WalkerAIController`](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.WalkerAIController) in the Python API
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     client::{ActorBase, Client},
///     geom::Location,
/// };
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// # let bp_lib = world.blueprint_library();
/// # let walker_bp = bp_lib.filter("walker.pedestrian.*").get(0).unwrap();
/// # let ai_controller_bp = bp_lib.find("controller.ai.walker").unwrap();
/// # let spawn_points = world.map().recommended_spawn_points();
/// # let walker = world.spawn_actor(&walker_bp, spawn_points.get(0).unwrap()).unwrap();
/// let ai_controller = world
///     .spawn_actor(&ai_controller_bp, spawn_points.get(0).unwrap())
///     .unwrap();
/// let mut ai: carla::client::WalkerAIController = ai_controller.try_into().unwrap();
///
/// // Start AI control
/// ai.start();
///
/// // Set maximum speed
/// ai.set_max_speed(1.4); // m/s
///
/// // Navigate to a location
/// let destination = Location {
///     x: 100.0,
///     y: 50.0,
///     z: 0.5,
/// };
/// ai.go_to_location(&destination);
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct WalkerAIController {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiWalkerAIController>,
}

impl WalkerAIController {
    /// Starts the AI controller.
    ///
    /// The walker will begin autonomous navigation behavior.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let ai_bp = bp_lib.find("controller.ai.walker").unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let actor = world.spawn_actor(&ai_bp, spawn_points.get(0).unwrap()).unwrap();
    /// # let mut ai: carla::client::WalkerAIController = actor.try_into().unwrap();
    /// ai.start();
    /// ```
    pub fn start(&self) {
        self.inner.Start();
    }

    /// Stops the AI controller.
    ///
    /// The walker will stop autonomous navigation.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let ai_bp = bp_lib.find("controller.ai.walker").unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let actor = world.spawn_actor(&ai_bp, spawn_points.get(0).unwrap()).unwrap();
    /// # let mut ai: carla::client::WalkerAIController = actor.try_into().unwrap();
    /// ai.stop();
    /// ```
    pub fn stop(&self) {
        self.inner.Stop();
    }

    // TODO: Implement get_random_location() once boost::optional FFI wrapper is ready
    // /// Gets a random navigation location.
    // ///
    // /// Returns a random point within the map's navigation mesh,
    // /// or `None` if no valid location is available.
    // pub fn get_random_location(&self) -> Option<Location> {
    //     // Requires boost::optional FFI wrapper
    //     None
    // }

    // TODO: Implement go_to_location() once autocxx properly supports const reference parameters
    // /// Directs the walker to navigate to a specific location.
    // ///
    // /// The AI controller will pathfind to the destination using the navigation mesh.
    // pub fn go_to_location(&self, location: &Location) {
    //     // Requires fixing autocxx const reference parameter handling
    // }

    /// Sets the maximum walking speed.
    ///
    /// # Arguments
    ///
    /// * `speed` - Maximum speed in meters per second (m/s)
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let ai_bp = bp_lib.find("controller.ai.walker").unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let actor = world.spawn_actor(&ai_bp, spawn_points.get(0).unwrap()).unwrap();
    /// # let mut ai: carla::client::WalkerAIController = actor.try_into().unwrap();
    /// // Set walking speed to 1.4 m/s (typical human walking speed)
    /// ai.set_max_speed(1.4);
    /// ```
    pub fn set_max_speed(&self, speed: f32) {
        self.inner.SetMaxSpeed(speed);
    }

    #[allow(dead_code)]
    pub(crate) fn from_cxx(ptr: SharedPtr<FfiWalkerAIController>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }

    #[allow(dead_code)]
    pub(crate) fn cxx_walker_ai_controller(&self) -> SharedPtr<FfiWalkerAIController> {
        self.inner.clone()
    }
}

impl ActorBase for WalkerAIController {
    fn cxx_actor(&self) -> SharedPtr<FfiActor> {
        // SAFETY: FfiWalkerAIController can be safely cast to FfiActor (inheritance)
        unsafe { std::mem::transmute(self.inner.clone()) }
    }
}

impl TryFrom<Actor> for WalkerAIController {
    type Error = Actor;

    fn try_from(actor: Actor) -> Result<Self, Self::Error> {
        // Try to cast to walker AI controller
        let controller_ptr: SharedPtr<FfiWalkerAIController> =
            // SAFETY: We check if the cast is valid by checking if type_id contains "controller.ai.walker"
            unsafe { std::mem::transmute(actor.inner.clone()) };

        if actor.type_id().contains("controller.ai.walker") {
            Ok(WalkerAIController {
                inner: controller_ptr,
            })
        } else {
            Err(actor)
        }
    }
}

// TODO: Re-enable Send/Sync once FFI wrapper properly supports it
// assert_impl_all!(WalkerAIController: Send, Sync);
