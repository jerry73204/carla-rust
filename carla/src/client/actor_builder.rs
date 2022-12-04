use super::{Actor, ActorBase, ActorBlueprint, Sensor, Vehicle, World};
use anyhow::{anyhow, ensure, Context, Result};
use carla_sys::carla::rpc::AttachmentType;
use itertools::chain;
use nalgebra::Isometry3;

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

    pub fn spawn_actor(self, transform: &Isometry3<f32>) -> Result<Actor> {
        self.spawn_actor_opt::<Actor, _>(transform, None, None)
    }

    pub fn spawn_actor_opt<A, T>(
        self,
        transform: &Isometry3<f32>,
        parent: Option<&A>,
        attachment_type: T,
    ) -> Result<Actor>
    where
        A: ActorBase,
        T: Into<Option<AttachmentType>>,
    {
        self.world
            .spawn_actor_opt(&self.blueprint, transform, parent, attachment_type)
    }

    pub fn spawn_vehicle(self, transform: &Isometry3<f32>) -> Result<Vehicle> {
        self.spawn_vehicle_opt::<Actor, _>(transform, None, None)
    }

    pub fn spawn_vehicle_opt<A, T>(
        self,
        transform: &Isometry3<f32>,
        parent: Option<&A>,
        attachment_type: T,
    ) -> Result<Vehicle>
    where
        A: ActorBase,
        T: Into<Option<AttachmentType>>,
    {
        let id = self.blueprint.id();
        self.spawn_actor_opt(transform, parent, attachment_type)?
            .try_into()
            .map_err(|_| anyhow!("the blueprint '{}' is not a vehicle", id))
    }

    pub fn spawn_sensor(self, transform: &Isometry3<f32>) -> Result<Sensor> {
        self.spawn_sensor_opt::<Actor, _>(transform, None, None)
    }

    pub fn spawn_sensor_opt<A, T>(
        self,
        transform: &Isometry3<f32>,
        parent: Option<&A>,
        attachment_type: T,
    ) -> Result<Sensor>
    where
        A: ActorBase,
        T: Into<Option<AttachmentType>>,
    {
        let id = self.blueprint.id();
        self.spawn_actor_opt(transform, parent, attachment_type)?
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
