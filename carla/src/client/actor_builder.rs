use super::{Actor, ActorBase, ActorBlueprint, Sensor, Vehicle, World};
use crate::{
    error::{ResourceError, ResourceType, Result, ValidationError},
    geom::Transform,
};
use carla_sys::carla::rpc::AttachmentType;

/// The builder is used to construct an actor with customized options.
#[derive(Debug)]
pub struct ActorBuilder<'a> {
    world: &'a mut World,
    blueprint: ActorBlueprint,
}

impl<'a> ActorBuilder<'a> {
    pub fn new(world: &'a mut World, key: &str) -> Result<Self> {
        let lib = world.blueprint_library();
        let blueprint = lib.find(key).ok_or_else(|| ResourceError::NotFound {
            resource_type: ResourceType::Blueprint,
            identifier: key.to_string(),
            context: Some(format!(
                "Available blueprints:\n{}",
                lib.iter()
                    .map(|bp| format!("- {}", bp.id()))
                    .collect::<Vec<_>>()
                    .join("\n")
            )),
        })?;
        Ok(Self { world, blueprint })
    }

    pub fn set_attribute(mut self, id: &str, value: &str) -> Result<Self> {
        let ok = self.blueprint.set_attribute(id, value);
        if !ok {
            return Err(ValidationError::InvalidConfiguration {
                setting: format!("blueprint attribute '{}'", id),
                reason: format!(
                    "Unable to set attribute '{}' to value '{}'. \
                     The attribute may not exist or the value may be invalid.",
                    id, value
                ),
            }
            .into());
        }
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
    /// # Ok::<(), carla::error::CarlaError>(())
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
    /// # Ok::<(), carla::error::CarlaError>(())
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
            .map_err(|_| {
                ValidationError::InvalidConfiguration {
                    setting: "blueprint type".to_string(),
                    reason: format!(
                        "Blueprint '{}' is not a vehicle. Use spawn() for non-vehicle actors.",
                        id
                    ),
                }
                .into()
            })
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
    /// # Ok::<(), carla::error::CarlaError>(())
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
            .map_err(|_| {
                ValidationError::InvalidConfiguration {
                    setting: "blueprint type".to_string(),
                    reason: format!(
                        "Blueprint '{}' is not a sensor. Use spawn() for non-sensor actors.",
                        id
                    ),
                }
                .into()
            })
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
