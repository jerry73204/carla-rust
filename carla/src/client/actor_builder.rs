use super::{Actor, ActorBase, ActorBlueprint, Sensor, Vehicle, World};
use crate::geom::Transform;
use anyhow::{anyhow, ensure, Context, Result};
use carla_sys::carla::rpc::AttachmentType;
use itertools::chain;

/// The builder is used to construct an actor with customized options.
#[derive(Debug)]
pub struct ActorBuilder<'a> {
    world: &'a mut World,
    blueprint: ActorBlueprint,
}

impl<'a> ActorBuilder<'a> {
    pub fn new(world: &'a mut World, key: &str) -> Result<Self> {
        let lib = world.blueprint_library();
        let blueprint = lib
            .find(key)
            .ok_or_else(|| anyhow!("unable to find blueprint '{}'", key))
            .with_context(|| {
                let blues = lib.iter().map(|blu| format!("- {}", blu.id()));
                itertools::join(
                    chain!(
                        ["available blueprints are:".to_string()],
                        blues,
                        ["".to_string()],
                    ),
                    "\n",
                )
            })?;
        Ok(Self { world, blueprint })
    }

    pub fn set_attribute(mut self, id: &str, value: &str) -> Result<Self> {
        let ok = self.blueprint.set_attribute(id, value);
        ensure!(ok, "Unable to set attribute '{}' to value '{}'", id, value);
        Ok(self)
    }

    /// Spawns an actor at the given transform.
    ///
    /// # Arguments
    ///
    /// * `transform` - Initial position and rotation for the actor
    ///
    /// # Returns
    ///
    /// The spawned actor, or an error if spawning failed.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # use carla::geom::Transform;
    /// let client = Client::default();
    /// let mut world = client.world();
    /// let transform = Transform::default();
    /// let actor = world
    ///     .actor_builder("vehicle.tesla.model3")?
    ///     .spawn(transform)?;
    /// # Ok::<(), anyhow::Error>(())
    /// ```
    pub fn spawn(self, transform: Transform) -> Result<Actor> {
        self.spawn_opt::<Actor, _>(transform, None, None)
    }

    pub fn spawn_opt<A, T>(
        self,
        transform: Transform,
        parent: Option<&A>,
        attachment_type: T,
    ) -> Result<Actor>
    where
        A: ActorBase,
        T: Into<Option<AttachmentType>>,
    {
        self.world
            .spawn_actor_opt(&self.blueprint, &transform, parent, attachment_type)
    }

    /// Spawns a vehicle at the given transform.
    ///
    /// # Arguments
    ///
    /// * `transform` - Initial position and rotation for the vehicle
    ///
    /// # Returns
    ///
    /// The spawned vehicle, or an error if spawning failed or the blueprint is not a vehicle.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # use carla::geom::Transform;
    /// let client = Client::default();
    /// let mut world = client.world();
    /// let transform = Transform::default();
    /// let vehicle = world
    ///     .actor_builder("vehicle.tesla.model3")?
    ///     .spawn_vehicle(transform)?;
    /// # Ok::<(), anyhow::Error>(())
    /// ```
    pub fn spawn_vehicle(self, transform: Transform) -> Result<Vehicle> {
        self.spawn_vehicle_opt::<Actor, _>(transform, None, None)
    }

    pub fn spawn_vehicle_opt<A, T>(
        self,
        transform: Transform,
        parent: Option<&A>,
        attachment_type: T,
    ) -> Result<Vehicle>
    where
        A: ActorBase,
        T: Into<Option<AttachmentType>>,
    {
        let id = self.blueprint.id();
        self.spawn_opt(transform, parent, attachment_type)?
            .try_into()
            .map_err(|_| anyhow!("the blueprint '{}' is not a vehicle", id))
    }

    /// Spawns a sensor at the given transform.
    ///
    /// # Arguments
    ///
    /// * `transform` - Initial position and rotation for the sensor
    ///
    /// # Returns
    ///
    /// The spawned sensor, or an error if spawning failed or the blueprint is not a sensor.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # use carla::geom::Transform;
    /// let client = Client::default();
    /// let mut world = client.world();
    /// let transform = Transform::default();
    /// let sensor = world
    ///     .actor_builder("sensor.camera.rgb")?
    ///     .spawn_sensor(transform)?;
    /// # Ok::<(), anyhow::Error>(())
    /// ```
    pub fn spawn_sensor(self, transform: Transform) -> Result<Sensor> {
        self.spawn_sensor_opt::<Actor, _>(transform, None, None)
    }

    pub fn spawn_sensor_opt<A, T>(
        self,
        transform: Transform,
        parent: Option<&A>,
        attachment_type: T,
    ) -> Result<Sensor>
    where
        A: ActorBase,
        T: Into<Option<AttachmentType>>,
    {
        let id = self.blueprint.id();
        self.spawn_opt(transform, parent, attachment_type)?
            .try_into()
            .map_err(|_| anyhow!("the blueprint '{}' is not a sensor", id))
    }

    pub fn blueprint(&self) -> &ActorBlueprint {
        &self.blueprint
    }

    pub fn blueprint_mut(&mut self) -> &mut ActorBlueprint {
        &mut self.blueprint
    }

    pub fn into_blueprint(self) -> ActorBlueprint {
        self.blueprint
    }
}
