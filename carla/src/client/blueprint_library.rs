use super::{ActorBlueprint, RecommendedSpawnPoints};
use crate::{
    geom::Transform,
    utils::{check_carla_error, rust_string_to_c},
};
use anyhow::{anyhow, Result};
use carla_sys::*;
use std::{collections::HashMap, ptr};

/// Provides blueprints used to spawn actors, corresponding to
/// `carla.BlueprintLibrary` in Python API.
#[derive(Clone, Debug)]
pub struct BlueprintLibrary {
    inner: *mut carla_blueprint_library_t,
}

impl BlueprintLibrary {
    /// Create a BlueprintLibrary from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_blueprint_library_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null blueprint library pointer"));
        }
        Ok(Self { inner: ptr })
    }

    /// Filter blueprints by wildcard pattern.
    ///
    /// # Arguments
    /// * `pattern` - Wildcard pattern to match blueprint IDs (e.g., "vehicle.*", "*sensor*")
    ///
    /// # Examples
    /// ```ignore
    /// let vehicles = library.filter("vehicle.*")?;
    /// let sensors = library.filter("sensor.*")?;
    /// ```
    pub fn filter(&self, pattern: &str) -> Result<Vec<ActorBlueprint>> {
        let pattern_cstring = rust_string_to_c(pattern)?;
        let mut count = 0;
        let blueprint_array = unsafe {
            carla_blueprint_library_filter(self.inner, pattern_cstring.as_ptr(), &mut count)
        };

        if blueprint_array.is_null() {
            return Ok(Vec::new());
        }

        let mut blueprints = Vec::with_capacity(count);
        for i in 0..count {
            let blueprint_ptr = unsafe { *blueprint_array.add(i) };
            if let Ok(blueprint) = ActorBlueprint::from_raw_ptr(blueprint_ptr) {
                blueprints.push(blueprint);
            }
        }

        Ok(blueprints)
    }

    /// Filter blueprints by actor type (e.g., "vehicle", "sensor", "walker").
    ///
    /// # Arguments
    /// * `actor_type` - The type of actor to filter for
    ///
    /// # Examples
    /// ```ignore
    /// let vehicles = library.filter_by_type("vehicle")?;
    /// let walkers = library.filter_by_type("walker")?;
    /// ```
    pub fn filter_by_type(&self, actor_type: &str) -> Result<Vec<ActorBlueprint>> {
        let pattern = format!("{}.*", actor_type);
        self.filter(&pattern)
    }

    /// Filter blueprints by multiple criteria.
    ///
    /// # Arguments
    /// * `filter_criteria` - HashMap of criteria (e.g., "type" -> "vehicle", "generation" -> "2")
    ///
    /// # Examples
    /// ```ignore
    /// let mut criteria = HashMap::new();
    /// criteria.insert("type".to_string(), "vehicle".to_string());
    /// criteria.insert("role_name".to_string(), "hero".to_string());
    /// let filtered = library.filter_by_criteria(&criteria)?;
    /// ```
    pub fn filter_by_criteria(
        &self,
        filter_criteria: &HashMap<String, String>,
    ) -> Result<Vec<ActorBlueprint>> {
        let all_blueprints = self.get_all()?;
        let mut filtered = Vec::new();

        for blueprint in all_blueprints {
            let mut matches = true;

            for (key, value) in filter_criteria {
                match key.as_str() {
                    "type" => {
                        if let Ok(id) = blueprint.id() {
                            if !id.starts_with(value) {
                                matches = false;
                                break;
                            }
                        }
                    }
                    "tag" => {
                        if !blueprint.has_tag(value) {
                            matches = false;
                            break;
                        }
                    }
                    "attribute" => {
                        // For attribute filtering, value should be "attr_name:attr_value"
                        if let Some((attr_name, attr_value)) = value.split_once(':') {
                            if let Ok(current_value) = blueprint.get_attribute(attr_name) {
                                if current_value != attr_value {
                                    matches = false;
                                    break;
                                }
                            } else {
                                matches = false;
                                break;
                            }
                        }
                    }
                    _ => {
                        // Try as a direct attribute match
                        if let Ok(current_value) = blueprint.get_attribute(key) {
                            if current_value != *value {
                                matches = false;
                                break;
                            }
                        }
                    }
                }
            }

            if matches {
                filtered.push(blueprint);
            }
        }

        Ok(filtered)
    }

    /// Get blueprints for vehicles only.
    pub fn vehicles(&self) -> Result<Vec<ActorBlueprint>> {
        self.filter_by_type("vehicle")
    }

    /// Get blueprints for sensors only.
    pub fn sensors(&self) -> Result<Vec<ActorBlueprint>> {
        self.filter_by_type("sensor")
    }

    /// Get blueprints for walkers (pedestrians) only.
    pub fn walkers(&self) -> Result<Vec<ActorBlueprint>> {
        self.filter_by_type("walker")
    }

    /// Get blueprints for traffic lights only.
    pub fn traffic_lights(&self) -> Result<Vec<ActorBlueprint>> {
        self.filter("static.prop.trafficlight*")
    }

    /// Get blueprints for traffic signs only.
    pub fn traffic_signs(&self) -> Result<Vec<ActorBlueprint>> {
        self.filter("static.prop.trafficsign*")
    }

    /// Find a blueprint by its exact ID.
    ///
    /// # Arguments
    /// * `key` - The exact blueprint ID to search for
    ///
    /// # Examples
    /// ```ignore
    /// if let Some(mustang) = library.find("vehicle.ford.mustang") {
    ///     // Use the mustang blueprint
    /// }
    /// ```
    pub fn find(&self, key: &str) -> Option<ActorBlueprint> {
        let key_cstring = rust_string_to_c(key).ok()?;
        let blueprint_ptr =
            unsafe { carla_blueprint_library_find(self.inner, key_cstring.as_ptr()) };

        if blueprint_ptr.is_null() {
            None
        } else {
            ActorBlueprint::from_raw_ptr(blueprint_ptr).ok()
        }
    }

    /// Find a blueprint with a fallback option if the first choice is not available.
    ///
    /// # Arguments
    /// * `preferred_id` - The preferred blueprint ID
    /// * `fallback_id` - The fallback blueprint ID if preferred is not found
    pub fn find_with_fallback(
        &self,
        preferred_id: &str,
        fallback_id: &str,
    ) -> Option<ActorBlueprint> {
        self.find(preferred_id).or_else(|| self.find(fallback_id))
    }

    /// Get all blueprints in the library.
    pub fn get_all(&self) -> Result<Vec<ActorBlueprint>> {
        let mut blueprints = Vec::with_capacity(self.len());
        for i in 0..self.len() {
            if let Some(blueprint) = self.get(i) {
                blueprints.push(blueprint);
            }
        }
        Ok(blueprints)
    }

    /// Get a random blueprint from the library.
    pub fn random(&self) -> Option<ActorBlueprint> {
        if self.is_empty() {
            return None;
        }

        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        use std::time::{SystemTime, UNIX_EPOCH};

        let mut hasher = DefaultHasher::new();
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos()
            .hash(&mut hasher);

        let random_index = (hasher.finish() as usize) % self.len();
        self.get(random_index)
    }

    /// Get a random blueprint of a specific type.
    ///
    /// # Arguments
    /// * `actor_type` - The type of actor to get a random blueprint for
    pub fn random_by_type(&self, actor_type: &str) -> Result<Option<ActorBlueprint>> {
        let filtered = self.filter_by_type(actor_type)?;
        if filtered.is_empty() {
            return Ok(None);
        }

        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        use std::time::{SystemTime, UNIX_EPOCH};

        let mut hasher = DefaultHasher::new();
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos()
            .hash(&mut hasher);

        let random_index = (hasher.finish() as usize) % filtered.len();
        Ok(filtered.into_iter().nth(random_index))
    }

    pub fn get(&self, index: usize) -> Option<ActorBlueprint> {
        if index >= self.len() {
            return None;
        }

        let blueprint_ptr = unsafe { carla_blueprint_library_at(self.inner, index) };

        if blueprint_ptr.is_null() {
            None
        } else {
            ActorBlueprint::from_raw_ptr(blueprint_ptr).ok()
        }
    }

    /// Get an iterator over all blueprints in the library.
    pub fn iter(&self) -> impl Iterator<Item = ActorBlueprint> + '_ {
        (0..self.len()).filter_map(|idx| self.get(idx))
    }

    /// Get an iterator over blueprints of a specific type.
    pub fn iter_by_type(&self, actor_type: &str) -> impl Iterator<Item = ActorBlueprint> + '_ {
        let pattern = format!("{}.*", actor_type);
        self.iter().filter(move |blueprint| {
            if let Ok(id) = blueprint.id() {
                id.starts_with(&pattern[..pattern.len() - 2]) // Remove the ".*" suffix
            } else {
                false
            }
        })
    }

    /// Get the number of blueprints in the library.
    pub fn len(&self) -> usize {
        unsafe { carla_blueprint_library_size(self.inner) }
    }

    /// Check if the library is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Get blueprints that match specific tags.
    ///
    /// # Arguments
    /// * `tags` - List of tags that blueprints must have
    /// * `match_all` - If true, blueprint must have ALL tags; if false, ANY tag
    pub fn filter_by_tags(&self, tags: &[&str], match_all: bool) -> Result<Vec<ActorBlueprint>> {
        let all_blueprints = self.get_all()?;
        let mut filtered = Vec::new();

        for blueprint in all_blueprints {
            let blueprint_matches = if match_all {
                tags.iter().all(|tag| blueprint.has_tag(tag))
            } else {
                tags.iter().any(|tag| blueprint.has_tag(tag))
            };

            if blueprint_matches {
                filtered.push(blueprint);
            }
        }

        Ok(filtered)
    }

    /// Get recommended spawn points for vehicle blueprints.
    ///
    /// Note: This method requires access to the world/map to get spawn points.
    /// Consider using `Map::recommended_spawn_points()` directly.
    pub fn get_recommended_spawn_points(&self) -> Result<RecommendedSpawnPoints> {
        // TODO: This requires access to the current world/map
        // For now, return an error directing users to use Map::recommended_spawn_points()
        Err(anyhow!(
            "Recommended spawn points must be accessed through Map::recommended_spawn_points(). \
             BlueprintLibrary does not have direct access to map data."
        ))
    }

    /// Count blueprints by type.
    pub fn count_by_type(&self, actor_type: &str) -> usize {
        self.iter_by_type(actor_type).count()
    }

    /// Get a summary of available blueprint types and their counts.
    pub fn blueprint_summary(&self) -> Result<HashMap<String, usize>> {
        let mut summary = HashMap::new();

        for blueprint in self.iter() {
            if let Ok(id) = blueprint.id() {
                // Extract the main type (e.g., "vehicle" from "vehicle.ford.mustang")
                if let Some(main_type) = id.split('.').next() {
                    *summary.entry(main_type.to_string()).or_insert(0) += 1;
                }
            }
        }

        Ok(summary)
    }

    /// Check if a specific blueprint ID exists in the library.
    pub fn contains(&self, blueprint_id: &str) -> bool {
        self.find(blueprint_id).is_some()
    }

    /// Get blueprints that have a specific attribute with a specific value.
    ///
    /// # Arguments
    /// * `attribute_name` - Name of the attribute to check
    /// * `attribute_value` - Expected value of the attribute
    pub fn filter_by_attribute(
        &self,
        attribute_name: &str,
        attribute_value: &str,
    ) -> Result<Vec<ActorBlueprint>> {
        let all_blueprints = self.get_all()?;
        let mut filtered = Vec::new();

        for blueprint in all_blueprints {
            if let Ok(value) = blueprint.get_attribute(attribute_name) {
                if value == attribute_value {
                    filtered.push(blueprint);
                }
            }
        }

        Ok(filtered)
    }
}

/// Type-safe blueprint selectors for specific actor types.
impl BlueprintLibrary {
    /// Get a blueprint selector for vehicles.
    pub fn vehicle_selector(&self) -> VehicleBlueprintSelector {
        VehicleBlueprintSelector { library: self }
    }

    /// Get a blueprint selector for sensors.
    pub fn sensor_selector(&self) -> SensorBlueprintSelector {
        SensorBlueprintSelector { library: self }
    }

    /// Get a blueprint selector for walkers.
    pub fn walker_selector(&self) -> WalkerBlueprintSelector {
        WalkerBlueprintSelector { library: self }
    }
}

/// Type-safe blueprint selector for vehicles.
pub struct VehicleBlueprintSelector<'a> {
    library: &'a BlueprintLibrary,
}

impl<'a> VehicleBlueprintSelector<'a> {
    /// Get all vehicle blueprints.
    pub fn all(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.vehicles()
    }

    /// Filter vehicles by manufacturer (e.g., "audi", "bmw").
    pub fn by_manufacturer(&self, manufacturer: &str) -> Result<Vec<ActorBlueprint>> {
        let pattern = format!("vehicle.{}.*", manufacturer);
        self.library.filter(&pattern)
    }

    /// Get vehicles by specific model (e.g., "mustang", "a2").
    pub fn by_model(&self, model: &str) -> Result<Vec<ActorBlueprint>> {
        let pattern = format!("vehicle.*.{}", model);
        self.library.filter(&pattern)
    }

    /// Get vehicles by generation number.
    pub fn by_generation(&self, generation: &str) -> Result<Vec<ActorBlueprint>> {
        self.library.filter_by_attribute("generation", generation)
    }

    /// Get only four-wheeled vehicles (cars).
    pub fn cars(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter_by_attribute("number_of_wheels", "4")
    }

    /// Get only two-wheeled vehicles (motorcycles, bicycles).
    pub fn motorcycles(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter_by_attribute("number_of_wheels", "2")
    }

    /// Get electric vehicles.
    pub fn electric(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter_by_tags(&["electric"], false)
    }

    /// Get a random vehicle blueprint.
    pub fn random(&self) -> Result<Option<ActorBlueprint>> {
        self.library.random_by_type("vehicle")
    }

    /// Find a specific vehicle by manufacturer and model.
    pub fn find(&self, manufacturer: &str, model: &str) -> Option<ActorBlueprint> {
        let blueprint_id = format!("vehicle.{}.{}", manufacturer, model);
        self.library.find(&blueprint_id)
    }
}

/// Type-safe blueprint selector for sensors.
pub struct SensorBlueprintSelector<'a> {
    library: &'a BlueprintLibrary,
}

impl<'a> SensorBlueprintSelector<'a> {
    /// Get all sensor blueprints.
    pub fn all(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.sensors()
    }

    /// Get camera sensors.
    pub fn cameras(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter("sensor.camera.*")
    }

    /// Get LiDAR sensors.
    pub fn lidars(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter("sensor.lidar.*")
    }

    /// Get radar sensors.
    pub fn radars(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter("sensor.other.radar")
    }

    /// Get IMU sensors.
    pub fn imus(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter("sensor.other.imu")
    }

    /// Get GNSS/GPS sensors.
    pub fn gnss(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter("sensor.other.gnss")
    }

    /// Get collision sensors.
    pub fn collision(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter("sensor.other.collision")
    }

    /// Get lane invasion sensors.
    pub fn lane_invasion(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter("sensor.other.lane_invasion")
    }

    /// Get obstacle detection sensors.
    pub fn obstacle_detection(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter("sensor.other.obstacle")
    }

    /// Get RGB camera sensor.
    pub fn rgb_camera(&self) -> Option<ActorBlueprint> {
        self.library.find("sensor.camera.rgb")
    }

    /// Get depth camera sensor.
    pub fn depth_camera(&self) -> Option<ActorBlueprint> {
        self.library.find("sensor.camera.depth")
    }

    /// Get semantic segmentation camera sensor.
    pub fn semantic_camera(&self) -> Option<ActorBlueprint> {
        self.library.find("sensor.camera.semantic_segmentation")
    }

    /// Get instance segmentation camera sensor.
    pub fn instance_camera(&self) -> Option<ActorBlueprint> {
        self.library.find("sensor.camera.instance_segmentation")
    }

    /// Get DVS (Dynamic Vision Sensor) camera.
    pub fn dvs_camera(&self) -> Option<ActorBlueprint> {
        self.library.find("sensor.camera.dvs")
    }

    /// Get optical flow camera sensor.
    pub fn optical_flow_camera(&self) -> Option<ActorBlueprint> {
        self.library.find("sensor.camera.optical_flow")
    }

    /// Get a random sensor blueprint.
    pub fn random(&self) -> Result<Option<ActorBlueprint>> {
        self.library.random_by_type("sensor")
    }
}

/// Type-safe blueprint selector for walkers (pedestrians).
pub struct WalkerBlueprintSelector<'a> {
    library: &'a BlueprintLibrary,
}

impl<'a> WalkerBlueprintSelector<'a> {
    /// Get all walker blueprints.
    pub fn all(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.walkers()
    }

    /// Get adult walker blueprints.
    pub fn adults(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter_by_tags(&["adult"], false)
    }

    /// Get child walker blueprints.
    pub fn children(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter_by_tags(&["child"], false)
    }

    /// Get male walker blueprints.
    pub fn males(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter_by_tags(&["male"], false)
    }

    /// Get female walker blueprints.
    pub fn females(&self) -> Result<Vec<ActorBlueprint>> {
        self.library.filter_by_tags(&["female"], false)
    }

    /// Get a random walker blueprint.
    pub fn random(&self) -> Result<Option<ActorBlueprint>> {
        self.library.random_by_type("walker")
    }

    /// Find a specific walker by ID.
    pub fn find(&self, walker_id: &str) -> Option<ActorBlueprint> {
        let blueprint_id = if walker_id.starts_with("walker.") {
            walker_id.to_string()
        } else {
            format!("walker.pedestrian.{}", walker_id)
        };
        self.library.find(&blueprint_id)
    }
}

impl Drop for BlueprintLibrary {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_blueprint_library_free(self.inner);
            }
            self.inner = ptr::null_mut();
        }
    }
}

// SAFETY: BlueprintLibrary wraps a thread-safe C API
unsafe impl Send for BlueprintLibrary {}
unsafe impl Sync for BlueprintLibrary {}
