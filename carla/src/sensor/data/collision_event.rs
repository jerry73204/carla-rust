//! Collision event sensor data.
//!
//! This module provides the [`CollisionEvent`] type for detecting and measuring collisions
//! between actors in the simulation.

use crate::{client::Actor, sensor::SensorData};
use carla_sys::{carla::geom::Vector3D, carla_rust::sensor::data::FfiCollisionEvent};
use cxx::SharedPtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Collision event data from a collision sensor.
///
/// Generated when an actor with a collision sensor attached collides with another actor.
/// Contains information about the collision force and the actors involved.
///
/// Corresponds to [`carla.CollisionEvent`] in the Python API.
#[cfg_attr(carla_version_0916, doc = "")]
#[cfg_attr(
    carla_version_0916,
    doc = " [`carla.CollisionEvent`]: https://carla.readthedocs.io/en/0.9.16/python_api/#carla.CollisionEvent"
)]
#[cfg_attr(carla_version_0915, doc = "")]
#[cfg_attr(
    carla_version_0915,
    doc = " [`carla.CollisionEvent`]: https://carla.readthedocs.io/en/0.9.15/python_api/#carla.CollisionEvent"
)]
#[cfg_attr(carla_version_0914, doc = "")]
#[cfg_attr(
    carla_version_0914,
    doc = " [`carla.CollisionEvent`]: https://carla.readthedocs.io/en/0.9.14/python_api/#carla.CollisionEvent"
)]
///
/// # Examples
///
/// ```no_run
/// use carla::{
///     client::Client,
///     sensor::{data::CollisionEvent, SensorDataBase},
/// };
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// // Spawn vehicle with collision sensor
/// let bp_lib = world.blueprint_library();
/// let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
/// let collision_bp = bp_lib.find("sensor.other.collision").unwrap();
///
/// let spawn_points = world.map().recommended_spawn_points();
/// let vehicle = world
///     .spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap())
///     .unwrap();
///
/// let collision_sensor = world
///     .spawn_actor_opt(
///         &collision_bp,
///         &nalgebra::Isometry3::identity(),
///         Some(&vehicle),
///         carla::rpc::AttachmentType::Rigid,
///     )
///     .unwrap();
///
/// let sensor: carla::client::Sensor = collision_sensor.try_into().unwrap();
/// sensor.listen(move |data| {
///     if let Ok(collision) = CollisionEvent::try_from(data) {
///         println!("Collision detected!");
///         println!("  Impulse: {:?}", collision.normal_impulse());
///         if let Some(other) = collision.other_actor() {
///             println!("  Hit actor ID: {}", other.id());
///         }
///     }
/// });
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct CollisionEvent {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiCollisionEvent>,
}

impl CollisionEvent {
    /// Returns the actor that owns the collision sensor.
    ///
    /// This is the actor that experienced the collision and has the sensor attached.
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.CollisionEvent.actor](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.CollisionEvent.actor)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.CollisionEvent.actor](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.CollisionEvent.actor)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.CollisionEvent.actor](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.CollisionEvent.actor)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::sensor::data::CollisionEvent;
    /// # let collision: CollisionEvent = todo!();
    /// let actor = collision.actor();
    /// println!("Collision sensor actor ID: {}", actor.id());
    /// ```
    pub fn actor(&self) -> Actor {
        unsafe { Actor::from_cxx(self.inner.GetActor()).unwrap_unchecked() }
    }

    /// Returns the actor that was hit in the collision, if any.
    ///
    /// Returns `None` if the collision was with a static object (like a building or the ground).
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.CollisionEvent.other_actor](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.CollisionEvent.other_actor)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.CollisionEvent.other_actor](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.CollisionEvent.other_actor)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.CollisionEvent.other_actor](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.CollisionEvent.other_actor)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::sensor::data::CollisionEvent;
    /// # let collision: CollisionEvent = todo!();
    /// if let Some(other) = collision.other_actor() {
    ///     println!("Collided with actor ID: {}", other.id());
    /// } else {
    ///     println!("Collided with static object");
    /// }
    /// ```
    pub fn other_actor(&self) -> Option<Actor> {
        Actor::from_cxx(self.inner.GetOtherActor())
    }

    /// Returns the normal impulse resulting from the collision.
    ///
    /// The impulse is a 3D vector representing the force applied during the collision.
    /// The magnitude indicates collision severity, and the direction shows the impact vector.
    ///
    /// Units: Newton-seconds (N·s)
    #[cfg_attr(
        carla_version_0916,
        doc = " See [carla.CollisionEvent.normal_impulse](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.CollisionEvent.normal_impulse)"
    )]
    #[cfg_attr(
        carla_version_0915,
        doc = " See [carla.CollisionEvent.normal_impulse](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.CollisionEvent.normal_impulse)"
    )]
    #[cfg_attr(
        carla_version_0914,
        doc = " See [carla.CollisionEvent.normal_impulse](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.CollisionEvent.normal_impulse)"
    )]
    #[cfg_attr(
        any(carla_version_0916, carla_version_0915, carla_version_0914),
        doc = " in the Python API."
    )]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::sensor::data::CollisionEvent;
    /// # let collision: CollisionEvent = todo!();
    /// let impulse = collision.normal_impulse();
    /// let magnitude = (impulse.x * impulse.x + impulse.y * impulse.y + impulse.z * impulse.z).sqrt();
    /// println!("Collision force: {} N·s", magnitude);
    /// ```
    pub fn normal_impulse(&self) -> &Vector3D {
        self.inner.GetNormalImpulse()
    }

    pub(crate) fn from_cxx(ptr: SharedPtr<FfiCollisionEvent>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl TryFrom<SensorData> for CollisionEvent {
    type Error = SensorData;

    fn try_from(value: SensorData) -> Result<Self, Self::Error> {
        let ptr = value.inner.to_collision_event();
        Self::from_cxx(ptr).ok_or(value)
    }
}

assert_impl_all!(CollisionEvent: Send, Sync);
