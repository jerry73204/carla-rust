use anyhow::{anyhow, Result};
use carla_sys::*;
use std::ffi::{CStr, CString};

/// OpenDRIVE generation parameters for creating road networks.
/// Provides functionality to generate OpenDRIVE map data from various sources.
#[derive(Clone, Debug)]
pub struct OpenDriveGenerator {
    inner: *mut carla_opendrive_generator_t,
}

impl OpenDriveGenerator {
    /// Create a new OpenDRIVE generator.
    pub fn new() -> Result<Self> {
        let generator_ptr = unsafe { carla_opendrive_generator_create() };
        if generator_ptr.is_null() {
            return Err(anyhow!("Failed to create OpenDRIVE generator"));
        }
        Ok(Self {
            inner: generator_ptr,
        })
    }

    /// Generate OpenDRIVE data from OSM (OpenStreetMap) data.
    ///
    /// # Arguments
    /// * `osm_data` - The OpenStreetMap XML data as a string
    /// * `settings` - Generation settings and parameters
    ///
    /// # Returns
    /// Generated OpenDRIVE XML data as a string
    pub fn generate_from_osm(
        &self,
        osm_data: &str,
        settings: &OpenDriveGenerationSettings,
    ) -> Result<String> {
        let osm_cstring = CString::new(osm_data).map_err(|_| anyhow!("Invalid OSM data string"))?;
        let c_settings = settings.to_c_settings();

        let result_ptr = unsafe {
            carla_opendrive_generate_from_osm(self.inner, osm_cstring.as_ptr(), &c_settings)
        };

        if result_ptr.is_null() {
            return Err(anyhow!("Failed to generate OpenDRIVE from OSM data"));
        }

        let c_str = unsafe { CStr::from_ptr(result_ptr) };
        let result = c_str.to_string_lossy().into_owned();

        // Free the C string allocated by the generator
        unsafe {
            carla_opendrive_free_string(result_ptr);
        }

        Ok(result)
    }

    /// Generate OpenDRIVE data from a list of waypoints.
    ///
    /// # Arguments
    /// * `waypoints` - List of waypoints defining the road path
    /// * `settings` - Generation settings and parameters
    ///
    /// # Returns
    /// Generated OpenDRIVE XML data as a string
    pub fn generate_from_waypoints(
        &self,
        waypoints: &[OpenDriveWaypoint],
        settings: &OpenDriveGenerationSettings,
    ) -> Result<String> {
        let c_waypoints: Vec<carla_opendrive_waypoint_t> =
            waypoints.iter().map(|wp| wp.to_c_waypoint()).collect();
        let c_settings = settings.to_c_settings();

        let result_ptr = unsafe {
            carla_opendrive_generate_from_waypoints(
                self.inner,
                c_waypoints.as_ptr(),
                c_waypoints.len(),
                &c_settings,
            )
        };

        if result_ptr.is_null() {
            return Err(anyhow!("Failed to generate OpenDRIVE from waypoints"));
        }

        let c_str = unsafe { CStr::from_ptr(result_ptr) };
        let result = c_str.to_string_lossy().into_owned();

        // Free the C string allocated by the generator
        unsafe {
            carla_opendrive_free_string(result_ptr);
        }

        Ok(result)
    }

    /// Validate an OpenDRIVE XML string.
    ///
    /// # Arguments
    /// * `opendrive_xml` - The OpenDRIVE XML data to validate
    ///
    /// # Returns
    /// True if the OpenDRIVE data is valid, false otherwise
    pub fn validate_opendrive(&self, opendrive_xml: &str) -> bool {
        let xml_cstring = match CString::new(opendrive_xml) {
            Ok(s) => s,
            Err(_) => return false,
        };

        unsafe { carla_opendrive_validate(self.inner, xml_cstring.as_ptr()) }
    }

    /// Get information about roads in an OpenDRIVE file.
    ///
    /// # Arguments
    /// * `opendrive_xml` - The OpenDRIVE XML data to analyze
    ///
    /// # Returns
    /// Vector of road information structures
    pub fn get_road_info(&self, opendrive_xml: &str) -> Result<Vec<OpenDriveRoadInfo>> {
        let xml_cstring =
            CString::new(opendrive_xml).map_err(|_| anyhow!("Invalid OpenDRIVE XML string"))?;

        let mut count = 0;
        let info_array_ptr =
            unsafe { carla_opendrive_get_road_info(self.inner, xml_cstring.as_ptr(), &mut count) };

        if info_array_ptr.is_null() {
            return Ok(Vec::new());
        }

        let mut road_infos = Vec::with_capacity(count);
        for i in 0..count {
            let c_info = unsafe { *info_array_ptr.add(i) };
            road_infos.push(OpenDriveRoadInfo::from_c_info(c_info));
        }

        // Free the array allocated by C
        unsafe {
            carla_opendrive_free_road_info_array(info_array_ptr);
        }

        Ok(road_infos)
    }
}

impl Drop for OpenDriveGenerator {
    fn drop(&mut self) {
        if !self.inner.is_null() {
            unsafe {
                carla_opendrive_generator_destroy(self.inner);
            }
            self.inner = std::ptr::null_mut();
        }
    }
}

/// Settings for OpenDRIVE generation.
#[derive(Clone, Debug)]
pub struct OpenDriveGenerationSettings {
    /// Whether to enable debugging output
    pub debug: bool,
    /// Default lane width in meters
    pub default_lane_width: f64,
    /// Whether to generate elevation data
    pub generate_elevation: bool,
    /// Elevation sampling distance in meters
    pub elevation_sampling_distance: f64,
    /// Whether to generate lane markings
    pub generate_lane_markings: bool,
    /// Default speed limit in km/h
    pub default_speed_limit: f64,
    /// Center map coordinates (latitude, longitude)
    pub center_map: bool,
}

impl Default for OpenDriveGenerationSettings {
    fn default() -> Self {
        Self {
            debug: false,
            default_lane_width: 3.5,
            generate_elevation: true,
            elevation_sampling_distance: 2.0,
            generate_lane_markings: true,
            default_speed_limit: 50.0,
            center_map: true,
        }
    }
}

impl OpenDriveGenerationSettings {
    pub(crate) fn to_c_settings(&self) -> carla_opendrive_generation_settings_t {
        carla_opendrive_generation_settings_t {
            debug: self.debug,
            default_lane_width: self.default_lane_width,
            generate_elevation: self.generate_elevation,
            elevation_sampling_distance: self.elevation_sampling_distance,
            generate_lane_markings: self.generate_lane_markings,
            default_speed_limit: self.default_speed_limit,
            center_map: self.center_map,
        }
    }
}

/// A waypoint used for OpenDRIVE generation.
#[derive(Clone, Debug)]
pub struct OpenDriveWaypoint {
    /// X coordinate in meters
    pub x: f64,
    /// Y coordinate in meters
    pub y: f64,
    /// Z coordinate (elevation) in meters
    pub z: f64,
    /// Heading angle in radians
    pub heading: f64,
    /// Lane width in meters
    pub lane_width: f64,
}

impl OpenDriveWaypoint {
    /// Create a new OpenDRIVE waypoint.
    pub fn new(x: f64, y: f64, z: f64, heading: f64, lane_width: f64) -> Self {
        Self {
            x,
            y,
            z,
            heading,
            lane_width,
        }
    }

    pub(crate) fn to_c_waypoint(&self) -> carla_opendrive_waypoint_t {
        carla_opendrive_waypoint_t {
            x: self.x,
            y: self.y,
            z: self.z,
            heading: self.heading,
            lane_width: self.lane_width,
        }
    }
}

/// Information about a road in an OpenDRIVE file.
#[derive(Clone, Debug)]
pub struct OpenDriveRoadInfo {
    /// Road ID
    pub id: i32,
    /// Road length in meters
    pub length: f64,
    /// Number of lanes
    pub lane_count: u32,
    /// Number of junctions this road connects to
    pub junction_count: u32,
}

impl OpenDriveRoadInfo {
    pub(crate) fn from_c_info(info: carla_opendrive_road_info_t) -> Self {
        Self {
            id: info.id,
            length: info.length,
            lane_count: info.lane_count,
            junction_count: info.junction_count,
        }
    }
}

// SAFETY: OpenDriveGenerator wraps a thread-safe C API
unsafe impl Send for OpenDriveGenerator {}
unsafe impl Sync for OpenDriveGenerator {}
