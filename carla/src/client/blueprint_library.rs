// SAFETY: This module uses unwrap_unchecked() for performance on methods guaranteed
// to never return null. See UNWRAP_REPLACEMENTS.md for detailed C++ code audit.

use super::ActorBlueprint;
use autocxx::prelude::*;
use carla_sys::carla_rust::client::{copy_actor_blueprint, FfiBlueprintLibrary};
use cxx::{let_cxx_string, SharedPtr};
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Provides blueprints used to spawn actors, corresponding to `carla.BlueprintLibrary` in Python API.
///
/// The blueprint library contains templates for all spawnable actors in CARLA,
/// including vehicles, pedestrians, sensors, and props. Use blueprints to create
/// actors with [`World::spawn_actor()`](crate::client::World::spawn_actor).
///
/// # Blueprint Categories
///
/// - **Vehicles**: `vehicle.*` (cars, trucks, motorcycles, bicycles)
/// - **Sensors**: `sensor.*` (cameras, LiDAR, GNSS, IMU, collision)
/// - **Walkers**: `walker.pedestrian.*`
/// - **Props**: `static.prop.*`
///
/// # Examples
///
/// ```no_run
/// use carla::client::Client;
///
/// let client = Client::default();
/// let mut world = client.world();
/// let library = world.blueprint_library();
///
/// // Find a specific vehicle by ID
/// if let Some(tesla_bp) = library.find("vehicle.tesla.model3") {
///     println!("Found Tesla Model 3 blueprint");
/// }
///
/// // Filter vehicles by wildcard pattern
/// let all_vehicles = library.filter("vehicle.*");
/// println!("Available vehicles: {}", all_vehicles.len());
///
/// // Filter Tesla vehicles specifically
/// let tesla_vehicles = library.filter("vehicle.tesla.*");
/// for bp in tesla_vehicles.iter() {
///     println!("Tesla: {}", bp.id());
/// }
///
/// // Get blueprint by index
/// if let Some(first_bp) = library.get(0) {
///     println!("First blueprint: {}", first_bp.id());
/// }
/// ```
#[derive(Clone, Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct BlueprintLibrary {
    #[derivative(Debug = "ignore")]
    inner: SharedPtr<FfiBlueprintLibrary>,
}

impl BlueprintLibrary {
    pub(crate) fn from_cxx(ptr: SharedPtr<FfiBlueprintLibrary>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }

    /// Filters blueprints by wildcard pattern.
    ///
    /// Returns a new library containing only blueprints matching the pattern.
    /// Use `*` as a wildcard to match multiple characters.
    ///
    /// # Arguments
    ///
    /// * `pattern` - Wildcard pattern (e.g., `"vehicle.*"`, `"vehicle.tesla.*"`)
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// let world = client.world();
    /// let library = world.blueprint_library();
    ///
    /// // Get all vehicles
    /// let vehicles = library.filter("vehicle.*");
    ///
    /// // Get all sensors
    /// let sensors = library.filter("sensor.*");
    ///
    /// // Get all Tesla vehicles
    /// let teslas = library.filter("vehicle.tesla.*");
    /// ```
    pub fn filter(&self, pattern: &str) -> Self {
        let_cxx_string!(pattern = pattern);
        let ptr = self.inner.filter(&pattern);
        unsafe { Self::from_cxx(ptr).unwrap_unchecked() }
    }

    /// Finds a blueprint by its exact ID.
    ///
    /// # Arguments
    ///
    /// * `key` - Exact blueprint ID (e.g., `"vehicle.tesla.model3"`)
    ///
    /// # Returns
    ///
    /// The blueprint if found, or `None` if the ID doesn't exist.
    pub fn find(&self, key: &str) -> Option<ActorBlueprint> {
        let_cxx_string!(key = key);
        unsafe {
            let actor_bp = self.inner.find(&key).as_ref()?;
            let actor_bp = copy_actor_blueprint(actor_bp).within_unique_ptr();
            Some(ActorBlueprint::from_cxx(actor_bp).unwrap_unchecked())
        }
    }

    /// Gets the blueprint at the given index.
    ///
    /// # Arguments
    ///
    /// * `index` - Index in the library (0-based)
    ///
    /// # Returns
    ///
    /// The blueprint at the index, or `None` if out of bounds.
    pub fn get(&self, index: usize) -> Option<ActorBlueprint> {
        unsafe {
            let actor_bp = self.inner.at(index).as_ref()?;
            let actor_bp = copy_actor_blueprint(actor_bp).within_unique_ptr();
            Some(ActorBlueprint::from_cxx(actor_bp).unwrap_unchecked())
        }
    }

    /// Returns an iterator over all blueprints in the library.
    pub fn iter(&self) -> impl Iterator<Item = ActorBlueprint> + '_ {
        // SAFETY: Index is bounds-checked by (0..self.len()), so get() always returns Some
        (0..self.len()).map(|idx| unsafe { self.get(idx).unwrap_unchecked() })
    }

    /// Returns the number of blueprints in the library.
    pub fn len(&self) -> usize {
        self.inner.size()
    }

    /// Returns true if the library contains no blueprints.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }
}

assert_impl_all!(BlueprintLibrary: Send, Sync);
