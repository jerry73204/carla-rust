// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::{
    Actor, ActorBase, ActorBlueprint, ActorBuilder, ActorList, ActorVec, BlueprintLibrary,
    BoundingBoxList, DebugHelper, EnvironmentObjectList, LabelledPointList, Landmark, LightManager,
    Map, Waypoint, WorldSnapshot,
};
use crate::{
    error::{OperationError, Result},
    geom::{Location, Transform, Vector3D},
    road::JuncId,
    rpc::{
        ActorId, AttachmentType, EpisodeSettings, LabelledPoint, MapLayer, VehicleLightStateList,
        WeatherParameters,
    },
    utils::CxxVectorExt,
};
use autocxx::prelude::*;
use carla_sys::carla_rust::{
    client::{FfiActor, FfiWorld},
    geom::FfiVector3D,
};
use cxx::{let_cxx_string, CxxVector, UniquePtr};
use derivative::Derivative;
use static_assertions::assert_impl_all;
use std::{ptr, time::Duration};

const DEFAULT_TICK_TIMEOUT: Duration = Duration::from_secs(60);

/// The world contains the map and assets of a simulation.
///
/// [`World`] is the main interface for interacting with the simulation. It provides:
/// - Access to the simulation [`Map`] and environment
/// - Actor spawning and management
/// - Weather and lighting control
/// - Simulation settings (synchronous mode, fixed time step, etc.)
/// - Traffic light management
/// - World snapshots for state inspection
///
/// Corresponds to [`carla.World`] in the Python API.
///
#[cfg_attr(carla_version_0916, doc = "")]
#[cfg_attr(carla_version_0916, doc = " [`carla.World`]: https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World")]
#[cfg_attr(carla_version_0915, doc = "")]
#[cfg_attr(carla_version_0915, doc = " [`carla.World`]: https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World")]
#[cfg_attr(carla_version_0914, doc = "")]
#[cfg_attr(carla_version_0914, doc = " [`carla.World`]: https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World")]
///
/// # Thread Safety
///
/// [`World`] implements [`Send`] and [`Sync`] and can be cloned cheaply (internally
/// reference-counted).
///
/// # Examples
///
/// ```no_run
/// use carla::client::Client;
///
/// let client = Client::default();
/// let mut world = client.world();
///
/// // Get simulation info
/// println!("World ID: {}", world.id());
/// let map = world.map();
/// println!("Map: {}", map.name());
///
/// // Spawn an actor
/// let bp_lib = world.blueprint_library();
/// let vehicle_bp = bp_lib.filter("vehicle.tesla.model3").get(0).unwrap();
/// let spawn_points = map.recommended_spawn_points();
/// if let Some(spawn_point) = spawn_points.get(0) {
///     let actor = world.spawn_actor(&vehicle_bp, &spawn_point);
/// }
/// ```
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct World {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: UniquePtr<FfiWorld>,
}

impl World {
    /// Returns the unique ID of this world/episode.
    ///
    /// The ID changes whenever the world is reloaded or a new map is loaded.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.id](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.id)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.id](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.id)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.id](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.id)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn id(&self) -> u64 {
        self.inner.GetId()
    }

    /// Returns the map associated with this world.
    ///
    /// The map contains the road network, spawn points, and navigation information.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_map](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_map)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_map](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_map)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_map](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_map)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn map(&self) -> Map {
        let ptr = self.inner.GetMap();
        unsafe { Map::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns the light manager for controlling street lights and vehicle lights.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_lightmanager](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_lightmanager)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_lightmanager](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_lightmanager)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_lightmanager](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_lightmanager)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn light_manager(&self) -> LightManager {
        unsafe { LightManager::from_cxx(self.inner.GetLightManager()).unwrap_unchecked() }
    }

    /// Loads a map layer (e.g., buildings, props, roads).
    ///
    /// Use this to dynamically load/unload parts of the map for performance.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.load_map_layer](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.load_map_layer)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.load_map_layer](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.load_map_layer)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.load_map_layer](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.load_map_layer)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn load_level_layer(&self, map_layers: MapLayer) {
        self.inner.LoadLevelLayer(map_layers as u16);
    }

    /// Unloads a map layer.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.unload_map_layer](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.unload_map_layer)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.unload_map_layer](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.unload_map_layer)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.unload_map_layer](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.unload_map_layer)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn unload_level_layer(&self, map_layers: MapLayer) {
        self.inner.UnloadLevelLayer(map_layers as u16);
    }

    /// Returns the blueprint library containing all available actor blueprints.
    ///
    /// Blueprints are templates for spawning actors (vehicles, sensors, pedestrians, etc.).
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_blueprint_library](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_blueprint_library)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_blueprint_library](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_blueprint_library)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_blueprint_library](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_blueprint_library)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// let world = client.world();
    /// let bp_lib = world.blueprint_library();
    ///
    /// // Find all vehicle blueprints
    /// let vehicles = bp_lib.filter("vehicle.*");
    /// println!("Found {} vehicle types", vehicles.len());
    /// ```
    pub fn blueprint_library(&self) -> BlueprintLibrary {
        let ptr = self.inner.GetBlueprintLibrary();
        unsafe { BlueprintLibrary::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns the light state of all vehicles in the world.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_vehicles_light_states](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_vehicles_light_states)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_vehicles_light_states](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_vehicles_light_states)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_vehicles_light_states](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_vehicles_light_states)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn vehicle_light_states(&self) -> VehicleLightStateList {
        let ptr = self.inner.GetVehiclesLightStates().within_unique_ptr();
        unsafe { VehicleLightStateList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns a random navigable location (on roads/sidewalks).
    ///
    /// Useful for spawning actors at random valid positions.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_random_location_from_navigation](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_random_location_from_navigation)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_random_location_from_navigation](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_random_location_from_navigation)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_random_location_from_navigation](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_random_location_from_navigation)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn random_location_from_navigation(&self) -> Location {
        let cpp_loc = self.inner.GetRandomLocationFromNavigation();
        Location::from_ffi(cpp_loc.as_ref().unwrap().clone())
    }

    /// Returns the spectator actor (the free-flying camera).
    ///
    /// Move the spectator to change the view in the CARLA window.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_spectator](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_spectator)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_spectator](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_spectator)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_spectator](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_spectator)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn spectator(&self) -> Actor {
        let actor = self.inner.GetSpectator();
        unsafe { Actor::from_cxx(actor).unwrap_unchecked() }
    }

    /// Returns the current simulation settings.
    ///
    /// Settings include synchronous mode, fixed time step, rendering options, etc.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_settings](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_settings)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_settings](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_settings)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_settings](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_settings)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn settings(&self) -> EpisodeSettings {
        let ptr = self.inner.GetSettings().within_unique_ptr();
        EpisodeSettings::from_cxx(&ptr)
    }

    /// Returns a snapshot of the current world state.
    ///
    /// Snapshots contain actor transforms, velocities, and simulation timestamp.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_snapshot](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_snapshot)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_snapshot](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_snapshot)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_snapshot](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_snapshot)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn snapshot(&self) -> WorldSnapshot {
        let ptr = self.inner.GetSnapshot();
        unsafe { WorldSnapshot::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns names of all environment objects in the world.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_names_of_all_objects](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_names_of_all_objects)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_names_of_all_objects](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_names_of_all_objects)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_names_of_all_objects](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_names_of_all_objects)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn names_of_all_objects(&self) -> Vec<String> {
        self.inner
            .GetNamesOfAllObjects()
            .iter()
            .map(|s| s.to_string())
            .collect()
    }

    /// Finds an actor by its ID.
    ///
    /// Returns `None` if the actor doesn't exist or has been destroyed.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_actor](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_actor)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_actor](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_actor)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_actor](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_actor)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn actor(&self, actor_id: ActorId) -> Option<Actor> {
        let ptr = self.inner.GetActor(actor_id);
        Actor::from_cxx(ptr)
    }

    /// Returns a list of all actors currently in the world.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_actors](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_actors)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_actors](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_actors)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_actors](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_actors)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// let world = client.world();
    /// let actors = world.actors();
    /// println!("Total actors: {}", actors.len());
    /// ```
    pub fn actors(&self) -> ActorList {
        let ptr = self.inner.GetActors();
        unsafe { ActorList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns a list of actors matching the given IDs.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_actors](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_actors)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_actors](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_actors)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_actors](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_actors)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn actors_by_ids(&self, ids: &[ActorId]) -> ActorList {
        let mut vec = CxxVector::new_typed();
        ids.iter().cloned().for_each(|id| {
            vec.pin_mut().push(id);
        });

        let ptr = self.inner.GetActorsByIds(&vec);
        unsafe { ActorList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns traffic lights affecting a waypoint within a certain distance.
    ///
    /// # Arguments
    /// * `waypoint` - The waypoint to check from
    /// * `distance` - Maximum distance to search for traffic lights
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_traffic_lights_from_waypoint](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_traffic_lights_from_waypoint)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_traffic_lights_from_waypoint](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_traffic_lights_from_waypoint)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_traffic_lights_from_waypoint](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_traffic_lights_from_waypoint)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn traffic_lights_from_waypoint(&self, waypoint: &Waypoint, distance: f64) -> ActorVec {
        let ptr = self
            .inner
            .GetTrafficLightsFromWaypoint(&waypoint.inner, distance)
            .within_unique_ptr();
        unsafe { ActorVec::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns all traffic lights in a junction.
    ///
    /// # Arguments
    /// * `junc_id` - Junction ID to query
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_traffic_lights_in_junction](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_traffic_lights_in_junction)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_traffic_lights_in_junction](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_traffic_lights_in_junction)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_traffic_lights_in_junction](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_traffic_lights_in_junction)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn traffic_lights_in_junction(&self, junc_id: JuncId) -> ActorVec {
        let ptr = self
            .inner
            .GetTrafficLightsInJunction(junc_id)
            .within_unique_ptr();
        unsafe { ActorVec::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Applies new simulation settings.
    ///
    /// Use this to change synchronous mode, time step, rendering options, etc.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.apply_settings](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.apply_settings)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.apply_settings](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.apply_settings)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.apply_settings](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.apply_settings)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    ///
    /// # Arguments
    ///
    /// * `settings` - The new settings to apply
    /// * `timeout` - Maximum time to wait for the operation
    ///
    /// # Returns
    ///
    /// The frame number when the settings were applied.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    /// use std::time::Duration;
    ///
    /// let client = Client::default();
    /// let mut world = client.world();
    ///
    /// let mut settings = world.settings();
    /// settings.synchronous_mode = true;
    /// settings.fixed_delta_seconds = Some(0.05);
    /// world.apply_settings(&settings, Duration::from_secs(2));
    /// ```
    pub fn apply_settings(&mut self, settings: &EpisodeSettings, timeout: Duration) -> u64 {
        let settings = settings.to_cxx();
        let millis = timeout.as_millis() as usize;
        self.inner.pin_mut().ApplySettings(&settings, millis)
    }

    /// Spawns an actor in the world.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.spawn_actor](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.spawn_actor)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.spawn_actor](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.spawn_actor)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.spawn_actor](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.spawn_actor)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    ///
    /// # Arguments
    ///
    /// * `blueprint` - The actor blueprint (from [`blueprint_library()`](Self::blueprint_library))
    /// * `transform` - Initial position and rotation
    ///
    /// # Returns
    ///
    /// The spawned [`Actor`], or an error if spawning failed (e.g., collision with
    /// another object).
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::{ActorBase, Client};
    ///
    /// let client = Client::default();
    /// let mut world = client.world();
    ///
    /// let bp_lib = world.blueprint_library();
    /// let vehicle_bp = bp_lib.filter("vehicle.tesla.model3").get(0).unwrap();
    ///
    /// let spawn_points = world.map().recommended_spawn_points();
    /// if let Some(spawn_point) = spawn_points.get(0) {
    ///     match world.spawn_actor(&vehicle_bp, &spawn_point) {
    ///         Ok(actor) => println!("Spawned actor {}", actor.id()),
    ///         Err(e) => eprintln!("Failed to spawn: {}", e),
    ///     }
    /// }
    /// ```
    pub fn spawn_actor(
        &mut self,
        blueprint: &ActorBlueprint,
        transform: &Transform,
    ) -> Result<Actor> {
        self.spawn_actor_opt::<Actor, _>(blueprint, transform, None, None)
    }

    /// Spawns an actor with optional parent attachment.
    ///
    /// # Arguments
    ///
    /// * `blueprint` - The actor blueprint
    /// * `transform` - Initial position and rotation (relative to parent if attached)
    /// * `parent` - Optional parent actor to attach to
    /// * `attachment_type` - How to attach to parent (Rigid, SpringArm, etc.)
    ///
    /// # Examples
    ///
    /// Attaching a camera to a vehicle:
    ///
    /// ```no_run
    /// use carla::client::Client;
    /// use carla::rpc::AttachmentType;
    /// use nalgebra::{Isometry3, Translation3, UnitQuaternion};
    ///
    /// let client = Client::default();
    /// let mut world = client.world();
    /// # let bp_lib = world.blueprint_library();
    /// # let vehicle_bp = bp_lib.filter("vehicle.*").get(0).unwrap();
    /// # let spawn_points = world.map().recommended_spawn_points();
    /// # let vehicle_actor = world.spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap()).unwrap();
    /// # let vehicle: carla::client::Vehicle = vehicle_actor.try_into().unwrap();
    ///
    /// // Spawn camera attached to vehicle
    /// let camera_bp = bp_lib.filter("sensor.camera.rgb").get(0).unwrap();
    /// let camera_transform = Isometry3::from_parts(
    ///     Translation3::new(0.0, 0.0, 2.0),
    ///     UnitQuaternion::identity()
    /// );
    ///
    /// let camera = world.spawn_actor_opt(
    ///     &camera_bp,
    ///     &camera_transform,
    ///     Some(&vehicle),
    ///     AttachmentType::Rigid
    /// ).unwrap();
    /// ```
    pub fn spawn_actor_opt<A, T>(
        &mut self,
        blueprint: &ActorBlueprint,
        transform: &Transform,
        parent: Option<&A>,
        attachment_type: T,
    ) -> Result<Actor>
    where
        A: ActorBase,
        T: Into<Option<AttachmentType>>,
    {
        let parent = parent.map(|parent| parent.cxx_actor());
        let attachment_type = attachment_type.into().unwrap_or(AttachmentType::Rigid);

        unsafe {
            let parent_ptr: *mut FfiActor = parent
                .as_ref()
                .and_then(|parent| parent.as_ref())
                .map(|ref_| ref_ as *const _ as *mut _)
                .unwrap_or(ptr::null_mut());
            let ffi_transform = transform.clone();
            #[cfg(carla_0916)]
            let actor = {
                use cxx::let_cxx_string;
                let_cxx_string!(socket_name = "");
                self.inner.pin_mut().TrySpawnActor(
                    &blueprint.inner,
                    ffi_transform.as_ffi(),
                    parent_ptr,
                    attachment_type,
                    &socket_name,
                )
            };
            #[cfg(not(carla_0916))]
            let actor = self.inner.pin_mut().TrySpawnActor(
                &blueprint.inner,
                ffi_transform.as_ffi(),
                parent_ptr,
                attachment_type,
            );
            Actor::from_cxx(actor).ok_or_else(|| {
                OperationError::SpawnFailed {
                    blueprint: blueprint.id().to_string(),
                    transform: format!("{:?}", transform),
                    reason: "Spawn returned null (likely collision or invalid location)"
                        .to_string(),
                    source: None,
                }
                .into()
            })
        }
    }

    /// Spawns an actor attached to a parent actor (convenience method).
    ///
    /// This is a convenience wrapper around [`spawn_actor_opt`](Self::spawn_actor_opt) that
    /// makes the API clearer when you specifically want to attach an actor to a parent.
    ///
    /// # Arguments
    ///
    /// * `blueprint` - The actor blueprint
    /// * `transform` - Initial position and rotation (relative to parent)
    /// * `parent` - Parent actor to attach to
    /// * `attachment_type` - How to attach to parent (defaults to [`AttachmentType::Rigid`] if not specified)
    ///
    /// # Examples
    ///
    /// Attaching a camera sensor to a vehicle:
    ///
    /// ```no_run
    /// use carla::{client::Client, rpc::AttachmentType};
    /// use nalgebra::{Isometry3, Translation3, UnitQuaternion};
    ///
    /// let client = Client::default();
    /// let mut world = client.world();
    /// let bp_lib = world.blueprint_library();
    ///
    /// // Spawn a vehicle
    /// let vehicle_bp = bp_lib.filter("vehicle.tesla.model3").get(0).unwrap();
    /// let spawn_points = world.map().recommended_spawn_points();
    /// let vehicle = world
    ///     .spawn_actor(&vehicle_bp, &spawn_points.get(0).unwrap())
    ///     .unwrap();
    ///
    /// // Attach a camera to the vehicle's roof
    /// let camera_bp = bp_lib.find("sensor.camera.rgb").unwrap();
    /// let camera_transform = Isometry3::from_parts(
    ///     Translation3::new(0.0, 0.0, 2.5).into(), // 2.5m above vehicle center
    ///     UnitQuaternion::identity(),
    /// );
    ///
    /// let camera = world
    ///     .spawn_actor_attached(
    ///         &camera_bp,
    ///         &camera_transform,
    ///         &vehicle,
    ///         AttachmentType::Rigid, // Camera moves rigidly with vehicle
    ///     )
    ///     .unwrap();
    /// ```
    ///
    /// See also: [`spawn_actor_opt`](Self::spawn_actor_opt) for spawning with optional parent
    pub fn spawn_actor_attached<A, T>(
        &mut self,
        blueprint: &ActorBlueprint,
        transform: &Transform,
        parent: &A,
        attachment_type: T,
    ) -> Result<Actor>
    where
        A: ActorBase,
        T: Into<Option<AttachmentType>>,
    {
        self.spawn_actor_opt(blueprint, transform, Some(parent), attachment_type)
    }

    /// Waits for the next simulation tick and returns the world snapshot.
    ///
    /// In synchronous mode, the server waits for this call before advancing the simulation.
    /// In asynchronous mode, this returns when the next tick completes.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.wait_for_tick](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.wait_for_tick)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.wait_for_tick](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.wait_for_tick)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.wait_for_tick](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.wait_for_tick)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    ///
    /// # Errors
    ///
    /// Returns an error if the operation times out (default: 60 seconds).
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// let world = client.world();
    ///
    /// // Wait for next tick
    /// let snapshot = world.wait_for_tick().unwrap();
    /// println!("Frame: {}", snapshot.frame());
    /// ```
    pub fn wait_for_tick(&self) -> Result<WorldSnapshot> {
        self.wait_for_tick_or_timeout(DEFAULT_TICK_TIMEOUT)
            .ok_or_else(|| {
                crate::error::ConnectionError::Timeout {
                    operation: "wait_for_tick".to_string(),
                    duration: DEFAULT_TICK_TIMEOUT,
                    source: None,
                }
                .into()
            })
    }

    /// Creates an actor builder for convenient actor spawning with attribute configuration.
    ///
    /// # Arguments
    ///
    /// * `key` - Blueprint ID (e.g., "vehicle.tesla.model3")
    pub fn actor_builder(&mut self, key: &str) -> Result<ActorBuilder<'_>> {
        ActorBuilder::new(self, key)
    }

    /// Waits for the next tick with a custom timeout.
    ///
    /// Returns `None` if the timeout expires.
    #[must_use]
    pub fn wait_for_tick_or_timeout(&self, timeout: Duration) -> Option<WorldSnapshot> {
        let ptr = self.inner.WaitForTick(timeout.as_millis() as usize);
        WorldSnapshot::from_cxx(ptr)
    }

    /// Advances the simulation by one tick (synchronous mode only).
    ///
    /// # Arguments
    ///
    /// * `timeout` - Maximum time to wait for the tick
    ///
    /// # Returns
    ///
    /// The frame number after the tick.
    pub fn tick_or_timeout(&mut self, timeout: Duration) -> u64 {
        self.inner.pin_mut().Tick(timeout.as_millis() as usize)
    }

    /// Advances the simulation by one tick (synchronous mode only).
    ///
    /// Uses the default timeout of 60 seconds.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.tick](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.tick)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.tick](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.tick)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.tick](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.tick)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn tick(&mut self) -> u64 {
        self.tick_or_timeout(DEFAULT_TICK_TIMEOUT)
    }

    /// Sets the percentage of pedestrians that will cross roads.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.set_pedestrians_cross_factor](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.set_pedestrians_cross_factor)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.set_pedestrians_cross_factor](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.set_pedestrians_cross_factor)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.set_pedestrians_cross_factor](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.set_pedestrians_cross_factor)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    ///
    /// # Arguments
    ///
    /// * `percentage` - Value from 0.0 to 1.0 (0% to 100%)
    pub fn set_pedestrians_cross_factor(&mut self, percentage: f32) {
        self.inner.pin_mut().SetPedestriansCrossFactor(percentage);
    }

    /// Sets the random seed for pedestrian behavior.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.set_pedestrians_seed](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.set_pedestrians_seed)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.set_pedestrians_seed](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.set_pedestrians_seed)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.set_pedestrians_seed](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.set_pedestrians_seed)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn set_pedestrians_seed(&mut self, seed: usize) {
        let seed = c_uint(seed as std::os::raw::c_uint);
        self.inner.pin_mut().SetPedestriansSeed(seed);
    }

    /// Returns the traffic sign actor at the given landmark location.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_traffic_sign](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_traffic_sign)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_traffic_sign](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_traffic_sign)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_traffic_sign](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_traffic_sign)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn traffic_sign_at(&self, landmark: &Landmark) -> Option<Actor> {
        // SAFETY: Landmark.inner is guaranteed non-null (see landmark.rs from_cxx())
        let ptr = self
            .inner
            .GetTrafficSign(unsafe { landmark.inner.as_ref().unwrap_unchecked() });
        Actor::from_cxx(ptr)
    }

    /// Returns the traffic light actor at the given landmark location.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_traffic_light](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_traffic_light)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_traffic_light](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_traffic_light)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_traffic_light](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_traffic_light)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn traffic_light_at(&self, landmark: &Landmark) -> Option<Actor> {
        // SAFETY: Landmark.inner is guaranteed non-null (see landmark.rs from_cxx())
        let ptr = self
            .inner
            .GetTrafficLight(unsafe { landmark.inner.as_ref().unwrap_unchecked() });
        Actor::from_cxx(ptr)
    }

    /// Returns the traffic light actor with the given OpenDRIVE sign ID.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_traffic_light_from_opendrive](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_traffic_light_from_opendrive)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_traffic_light_from_opendrive](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_traffic_light_from_opendrive)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_traffic_light_from_opendrive](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_traffic_light_from_opendrive)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn traffic_light_from_open_drive(&self, sign_id: &str) -> Option<Actor> {
        let_cxx_string!(sign_id = sign_id);
        let ptr = self.inner.GetTrafficLightFromOpenDRIVE(&sign_id);
        Actor::from_cxx(ptr)
    }

    /// Freezes or unfreezes all traffic lights in the world.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.freeze_all_traffic_lights](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.freeze_all_traffic_lights)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.freeze_all_traffic_lights](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.freeze_all_traffic_lights)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.freeze_all_traffic_lights](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.freeze_all_traffic_lights)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn freeze_all_traffic_lights(&mut self, frozen: bool) {
        self.inner.pin_mut().FreezeAllTrafficLights(frozen);
    }

    /// Resets all traffic lights to their initial state.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.reset_all_traffic_lights](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.reset_all_traffic_lights)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.reset_all_traffic_lights](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.reset_all_traffic_lights)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.reset_all_traffic_lights](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.reset_all_traffic_lights)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn reset_all_traffic_lights(&mut self) {
        self.inner.pin_mut().ResetAllTrafficLights();
    }

    /// Returns bounding boxes for environment objects matching the queried tag.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_level_bbs](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_level_bbs)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_level_bbs](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_level_bbs)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_level_bbs](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_level_bbs)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn level_bounding_boxes(&self, queried_tag: u8) -> BoundingBoxList {
        let ptr = self.inner.GetLevelBBs(queried_tag);
        unsafe { BoundingBoxList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Returns the current weather parameters.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_weather](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_weather)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_weather](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_weather)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_weather](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_weather)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn weather(&self) -> WeatherParameters {
        self.inner.GetWeather()
    }

    /// Sets the weather parameters (sun, clouds, precipitation, fog, etc.).
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.set_weather](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.set_weather)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.set_weather](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.set_weather)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.set_weather](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.set_weather)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::{client::Client, rpc::WeatherParameters};
    ///
    /// let client = Client::default();
    /// let mut world = client.world();
    ///
    /// let mut weather = world.weather();
    /// weather.cloudiness = 80.0;
    /// weather.precipitation = 50.0;
    /// world.set_weather(&weather);
    /// ```
    pub fn set_weather(&mut self, weather: &WeatherParameters) {
        self.inner.pin_mut().SetWeather(weather)
    }

    /// Returns environment objects matching the queried tag.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.get_environment_objects](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.get_environment_objects)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.get_environment_objects](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.get_environment_objects)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.get_environment_objects](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.get_environment_objects)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn environment_objects(&self, queried_tag: u8) -> EnvironmentObjectList {
        let ptr = self
            .inner
            .GetEnvironmentObjects(queried_tag)
            .within_unique_ptr();
        unsafe { EnvironmentObjectList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Enables or disables environment objects by their IDs.
    ///
    ///
    #[cfg_attr(carla_version_0916, doc = " See [carla.World.enable_environment_objects](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.World.enable_environment_objects)")]
    #[cfg_attr(carla_version_0915, doc = " See [carla.World.enable_environment_objects](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.World.enable_environment_objects)")]
    #[cfg_attr(carla_version_0914, doc = " See [carla.World.enable_environment_objects](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.World.enable_environment_objects)")]
    #[cfg_attr(any(carla_version_0916, carla_version_0915, carla_version_0914), doc = " in the Python API.")]
    pub fn enable_environment_objects(&self, ids: &[u64], enable: bool) {
        let ptr = ids.as_ptr();
        let len = ids.len();
        unsafe {
            self.inner.EnableEnvironmentObjects(ptr, len, enable);
        }
    }

    pub fn project_point(
        &self,
        location: &Location,
        direction: &Vector3D,
        search_distance: f32,
    ) -> Option<LabelledPoint> {
        // SAFETY: FfiVector3D and carla::geom::Vector3D have identical memory layout
        let cpp_dir = unsafe {
            std::mem::transmute::<FfiVector3D, carla_sys::carla::geom::Vector3D>(
                direction.into_ffi(),
            )
        };
        let ptr = self
            .inner
            .ProjectPoint(location.into_ffi(), cpp_dir, search_distance);

        if ptr.is_null() {
            None
        } else {
            Some((*ptr).clone())
        }
    }

    pub fn ground_projection(
        &self,
        location: &Location,
        search_distance: f32,
    ) -> Option<LabelledPoint> {
        let ptr = self
            .inner
            .GroundProjection(location.into_ffi(), search_distance);

        if ptr.is_null() {
            None
        } else {
            Some((*ptr).clone())
        }
    }

    pub fn cast_ray(&self, start: &Location, end: &Location) -> LabelledPointList {
        let ptr = self
            .inner
            .CastRay(start.into_ffi(), end.into_ffi())
            .within_unique_ptr();
        unsafe { LabelledPointList::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Creates a debug helper for drawing visualization primitives.
    ///
    /// Returns a [`DebugHelper`] that can be used to draw points, lines, boxes, arrows,
    /// and text in the simulation for debugging and visualization purposes.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # use carla::geom::Location;
    /// # use carla::rpc::Color;
    /// #
    /// # let mut client = Client::connect("localhost", 2000, None).unwrap();
    /// # let mut world = client.world();
    /// let mut debug = world.debug();
    ///
    /// // Draw a red point at the origin
    /// debug.draw_point(Location::new(0.0, 0.0, 0.0), 0.5, Color::RED, 5.0, false);
    /// ```
    pub fn debug(&mut self) -> DebugHelper {
        let ptr = self.inner.pin_mut().MakeDebugHelper();
        DebugHelper { inner: ptr }
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiWorld>) -> Option<World> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl Clone for World {
    fn clone(&self) -> Self {
        unsafe { Self::from_cxx(self.inner.clone().within_unique_ptr()).unwrap_unchecked() }
    }
}

assert_impl_all!(World: Sync, Send);
