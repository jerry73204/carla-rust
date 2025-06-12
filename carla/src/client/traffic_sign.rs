use super::{Actor, ActorBase};
use crate::{
    geom::{Transform, Vector3D},
    stubs::{
        carla_actor_is_traffic_sign, carla_traffic_sign_get_trigger_volume,
        carla_traffic_sign_get_sign_id, carla_traffic_sign_t,
    },
};
use anyhow::{anyhow, Result};
use carla_sys::*;

/// A traffic sign actor in the simulation.
/// This is a newtype wrapper around Actor that provides traffic sign-specific functionality.
#[derive(Clone, Debug)]
pub struct TrafficSign(pub Actor);

impl TrafficSign {
    /// Create a TrafficSign from a raw C pointer.
    ///
    /// # Safety
    /// The pointer must be valid and not null, and must point to a traffic sign actor.
    pub(crate) fn from_raw_ptr(ptr: *mut carla_actor_t) -> Result<Self> {
        if ptr.is_null() {
            return Err(anyhow!("Null traffic sign pointer"));
        }

        // Verify it's actually a traffic sign
        if !unsafe { carla_actor_is_traffic_sign(ptr) } {
            return Err(anyhow!("Actor is not a traffic sign"));
        }

        // Create the base Actor first
        let actor = Actor::from_raw_ptr(ptr)?;
        Ok(Self(actor))
    }

    /// Convert this TrafficSign back into a generic Actor.
    pub fn into_actor(self) -> Actor {
        self.0
    }

    /// Get access to the underlying Actor.
    pub fn actor(&self) -> &Actor {
        &self.0
    }

    /// Get mutable access to the underlying Actor.
    pub fn actor_mut(&mut self) -> &mut Actor {
        &mut self.0
    }

    pub(crate) fn raw_traffic_sign_ptr(&self) -> *mut carla_traffic_sign_t {
        self.0.raw_ptr() as *mut carla_traffic_sign_t
    }

    /// Get the traffic sign ID.
    pub fn get_sign_id(&self) -> String {
        let c_sign_id = unsafe { carla_traffic_sign_get_sign_id(self.raw_traffic_sign_ptr()) };
        // TODO: Convert C string to Rust string when C API is available
        "unknown".to_string()
    }

    /// Get the trigger volume of the traffic sign.
    pub fn get_trigger_volume(&self) -> TrafficSignTriggerVolume {
        let c_volume = unsafe { carla_traffic_sign_get_trigger_volume(self.raw_traffic_sign_ptr()) };
        TrafficSignTriggerVolume::from_c_volume(c_volume)
    }

    /// Check if a vehicle is within the trigger volume.
    pub fn is_vehicle_in_trigger_volume(&self, vehicle_location: Vector3D) -> bool {
        let trigger_volume = self.get_trigger_volume();
        trigger_volume.contains_point(vehicle_location)
    }

    /// Get traffic sign information.
    pub fn get_sign_info(&self) -> TrafficSignInfo {
        TrafficSignInfo {
            sign_id: self.get_sign_id(),
            sign_type: TrafficSignType::Unknown, // TODO: Implement when C API is available
            trigger_volume: self.get_trigger_volume(),
            transform: self.get_transform(),
        }
    }

    /// Get traffic sign properties.
    pub fn get_properties(&self) -> TrafficSignProperties {
        // TODO: Implement when C API provides more traffic sign properties
        TrafficSignProperties {
            speed_limit: None,
            priority: TrafficSignPriority::Normal,
            affects_lanes: Vec::new(),
        }
    }

    /// Destroy this traffic sign.
    pub fn destroy(self) -> Result<()> {
        self.0.destroy()
    }
}

/// Traffic sign trigger volume definition.
#[derive(Debug, Clone)]
pub struct TrafficSignTriggerVolume {
    /// Center location of the trigger volume.
    pub center: Vector3D,
    /// Size of the trigger volume (width, height, depth).
    pub extent: Vector3D,
}

impl TrafficSignTriggerVolume {
    /// Create from C trigger volume.
    /// TODO: Implement when C API provides proper trigger volume structure
    pub(crate) fn from_c_volume(_c_volume: *mut std::ffi::c_void) -> Self {
        // TODO: Implement conversion when C API is available
        Self {
            center: Vector3D::default(),
            extent: Vector3D { x: 5.0, y: 5.0, z: 2.0 }, // Default trigger volume
        }
    }

    /// Check if a point is within the trigger volume.
    pub fn contains_point(&self, point: Vector3D) -> bool {
        let half_extent = Vector3D {
            x: self.extent.x / 2.0,
            y: self.extent.y / 2.0,
            z: self.extent.z / 2.0,
        };

        point.x >= (self.center.x - half_extent.x) &&
        point.x <= (self.center.x + half_extent.x) &&
        point.y >= (self.center.y - half_extent.y) &&
        point.y <= (self.center.y + half_extent.y) &&
        point.z >= (self.center.z - half_extent.z) &&
        point.z <= (self.center.z + half_extent.z)
    }
}

/// Traffic sign type enumeration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TrafficSignType {
    /// Stop sign.
    Stop,
    /// Yield sign.
    Yield,
    /// Speed limit sign.
    SpeedLimit,
    /// No entry sign.
    NoEntry,
    /// One way sign.
    OneWay,
    /// Parking sign.
    Parking,
    /// Warning sign.
    Warning,
    /// Information sign.
    Information,
    /// Unknown or unsupported sign type.
    Unknown,
}

/// Traffic sign priority level.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TrafficSignPriority {
    /// Low priority sign.
    Low,
    /// Normal priority sign.
    Normal,
    /// High priority sign (e.g., stop, yield).
    High,
    /// Critical priority sign (e.g., danger warnings).
    Critical,
}

/// Complete traffic sign information.
#[derive(Debug, Clone)]
pub struct TrafficSignInfo {
    /// Unique sign identifier.
    pub sign_id: String,
    /// Type of traffic sign.
    pub sign_type: TrafficSignType,
    /// Trigger volume for vehicle detection.
    pub trigger_volume: TrafficSignTriggerVolume,
    /// World transform of the sign.
    pub transform: Transform,
}

/// Traffic sign properties and effects.
#[derive(Debug, Clone)]
pub struct TrafficSignProperties {
    /// Speed limit imposed by the sign (if applicable).
    pub speed_limit: Option<f32>,
    /// Priority level of the sign.
    pub priority: TrafficSignPriority,
    /// Lane IDs affected by this sign.
    pub affects_lanes: Vec<u32>,
}

// Implement ActorBase trait for TrafficSign
impl ActorBase for TrafficSign {
    fn raw_ptr(&self) -> *mut carla_actor_t {
        self.0.raw_ptr()
    }

    fn id(&self) -> u32 {
        self.0.id()
    }

    fn type_id(&self) -> String {
        self.0.type_id()
    }

    fn get_transform(&self) -> Transform {
        self.0.get_transform()
    }

    fn is_alive(&self) -> bool {
        self.0.is_alive()
    }
}

// Note: TrafficSign doesn't implement Drop because it's a newtype wrapper around Actor,
// and the underlying carla_actor_t will be freed when the Actor is dropped

// SAFETY: TrafficSign wraps a thread-safe C API
unsafe impl Send for TrafficSign {}
unsafe impl Sync for TrafficSign {}
