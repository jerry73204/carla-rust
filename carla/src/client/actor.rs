use crate::client::{Sensor, Vehicle};
use carla_sys::carla_rust::client::FfiActor;
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

use super::{ActorBase, ActorKind, TrafficLight, TrafficSign};

/// A base actor that represents a movable object in the simulation,
/// Corresponds to [`carla.Actor`](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Actor) in the Python API.
///
/// [`Actor`] is the generic type for all entities in the simulation. For type-specific
/// functionality, convert to specialized types using [`TryFrom`]:
///
/// ```no_run
/// use carla::client::{Client, Sensor, Vehicle};
///
/// let client = Client::default();
/// let mut world = client.world();
/// # let bp_lib = world.blueprint_library();
/// # let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
/// # let spawn_points = world.map().recommended_spawn_points();
/// # let actor = world.spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap()).unwrap();
///
/// // Convert to specific type
/// let vehicle: Vehicle = actor.try_into().unwrap();
/// ```
///
/// All actor types implement [`ActorBase`], providing common functionality for
/// position, physics, and metadata queries.
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct Actor {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: SharedPtr<FfiActor>,
}

impl Actor {
    /// Classifies the actor into a specific variant ([`ActorKind`]).
    ///
    /// This consumes the actor and returns an enum that can be matched to
    /// determine the specific actor type.
    pub fn into_kinds(self) -> ActorKind {
        let me = self;
        let me = match Vehicle::try_from(me) {
            Ok(me) => return me.into(),
            Err(me) => me,
        };
        let me = match Sensor::try_from(me) {
            Ok(me) => return me.into(),
            Err(me) => me,
        };
        let me = match TrafficLight::try_from(me) {
            Ok(me) => return me.into(),
            Err(me) => me,
        };
        let me = match TrafficSign::try_from(me) {
            Ok(me) => return me.into(),
            Err(me) => me,
        };

        me.into()
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiActor>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl ActorBase for Actor {
    fn cxx_actor(&self) -> SharedPtr<FfiActor> {
        self.inner.clone()
    }
}

assert_impl_all!(Actor: Send, Sync);
