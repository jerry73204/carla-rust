use autocxx::prelude::*;
use carla_sys::{
    carla::client::ActorBlueprint as FfiActorBlueprint, carla_rust::client::copy_actor_blueprint,
};
use cxx::{let_cxx_string, UniquePtr};
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// A template used to construct an actor, corresponding to `carla.ActorBlueprint` in Python API.
///
/// Blueprints define the type and configurable parameters of actors that can be spawned
/// in the simulation. Each blueprint has:
/// - An **ID** (e.g., `"vehicle.tesla.model3"`)
/// - **Tags** for categorization (e.g., `"vehicle"`, `"car"`)
/// - **Attributes** that can be modified before spawning (color, role_name, etc.)
///
/// # Common Attributes
///
/// - `role_name` - Identifier for the actor (useful for tracking specific vehicles)
/// - `color` (vehicles) - RGB color string (e.g., `"255,0,0"` for red)
/// - `driver_id` (vehicles) - Pedestrian blueprint ID for the driver
/// - `image_size_x`, `image_size_y` (cameras) - Resolution
/// - `fov` (cameras) - Field of view in degrees
///
/// # Examples
///
/// ```no_run
/// use carla::client::{ActorBase, Client};
/// use nalgebra::Isometry3;
///
/// let client = Client::default();
/// let mut world = client.world();
/// let library = world.blueprint_library();
///
/// // Get a vehicle blueprint
/// let mut bp = library.find("vehicle.tesla.model3").unwrap();
///
/// // Check blueprint ID and tags
/// println!("Blueprint ID: {}", bp.id());
/// println!("Tags: {:?}", bp.tags());
/// assert!(bp.contains_tag("car"));
///
/// // Modify attributes
/// bp.set_attribute("role_name", "hero");
/// bp.set_attribute("color", "255,0,0"); // Red color
///
/// // Spawn with modified blueprint
/// let spawn_points = world.map().recommended_spawn_points();
/// if let Some(transform) = spawn_points.get(0) {
///     let actor = world.spawn_actor(&bp, &transform).unwrap();
///     println!("Spawned red Tesla with ID {}", actor.id());
/// }
/// ```
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct ActorBlueprint {
    #[derivative(Debug = "ignore")]
    pub(crate) inner: UniquePtr<FfiActorBlueprint>,
}

impl ActorBlueprint {
    /// Returns the blueprint's unique identifier.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let world = client.world();
    /// # let library = world.blueprint_library();
    /// let bp = library.find("vehicle.tesla.model3").unwrap();
    /// assert_eq!(bp.id(), "vehicle.tesla.model3");
    /// ```
    pub fn id(&self) -> String {
        self.inner.GetId().to_string()
    }

    /// Checks if the blueprint has a specific tag.
    ///
    /// # Arguments
    ///
    /// * `tag` - Tag to check for (e.g., `"vehicle"`, `"car"`)
    pub fn contains_tag(&self, tag: &str) -> bool {
        let_cxx_string!(tag = tag);
        self.inner.ContainsTag(&tag)
    }

    /// Checks if the blueprint's tags match a wildcard pattern.
    ///
    /// # Arguments
    ///
    /// * `pattern` - Wildcard pattern (e.g., `"vehicle.*"`)
    pub fn match_tags(&self, pattern: &str) -> bool {
        let_cxx_string!(pattern = pattern);
        self.inner.MatchTags(&pattern)
    }

    /// Returns all tags associated with this blueprint.
    ///
    /// Tags are used for categorization and filtering.
    pub fn tags(&self) -> Vec<String> {
        self.inner
            .GetTags()
            .iter()
            .map(|tag| tag.to_string())
            .collect()
    }

    /// Checks if the blueprint has a specific attribute.
    ///
    /// # Arguments
    ///
    /// * `id` - Attribute name (e.g., `"role_name"`, `"color"`)
    pub fn contains_attribute(&self, id: &str) -> bool {
        let_cxx_string!(id = id);
        self.inner.ContainsAttribute(&id)
    }

    /// Sets an attribute value.
    ///
    /// # Arguments
    ///
    /// * `id` - Attribute name
    /// * `value` - New value as a string
    ///
    /// # Returns
    ///
    /// `true` if the attribute was set successfully, `false` if the attribute doesn't exist.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let world = client.world();
    /// # let library = world.blueprint_library();
    /// let mut bp = library.find("vehicle.tesla.model3").unwrap();
    ///
    /// // Set role name and color
    /// bp.set_attribute("role_name", "hero");
    /// bp.set_attribute("color", "255,0,0"); // Red
    /// ```
    #[must_use]
    pub fn set_attribute(&mut self, id: &str, value: &str) -> bool {
        if !self.contains_attribute(id) {
            return false;
        }
        let_cxx_string!(id = id);
        self.inner.pin_mut().SetAttribute(&id, value);
        true
    }

    /// Returns the number of attributes.
    pub fn len(&self) -> usize {
        self.inner.size()
    }

    /// Returns true if the blueprint has no attributes.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiActorBlueprint>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

impl Clone for ActorBlueprint {
    fn clone(&self) -> Self {
        let clone = copy_actor_blueprint(&self.inner).within_unique_ptr();
        Self { inner: clone }
    }
}

assert_impl_all!(ActorBlueprint: Send, Sync);
