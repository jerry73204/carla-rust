//! World interaction utilities for ray casting and queries.

use crate::ffi::bridge::{
    SimpleActorList, SimpleLabelledPoint, SimpleLocation, SimpleOptionalLabelledPoint,
    SimpleOptionalLocation, SimpleVector3D,
};

/// City object labels for ray casting results
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CityObjectLabel {
    None = 0,
    Roads = 1,
    Sidewalks = 2,
    Buildings = 3,
    Walls = 4,
    Fences = 5,
    Poles = 6,
    TrafficLight = 7,
    TrafficSigns = 8,
    Vegetation = 9,
    Terrain = 10,
    Sky = 11,
    Pedestrians = 12,
    Rider = 13,
    Car = 14,
    Truck = 15,
    Bus = 16,
    Train = 17,
    Motorcycle = 18,
    Bicycle = 19,
    Static = 20,
    Dynamic = 21,
    Other = 22,
    Water = 23,
    RoadLine = 24,
    Ground = 25,
    Bridge = 26,
    RailTrack = 27,
    GuardRail = 28,
}

impl From<u8> for CityObjectLabel {
    fn from(value: u8) -> Self {
        match value {
            0 => CityObjectLabel::None,
            1 => CityObjectLabel::Roads,
            2 => CityObjectLabel::Sidewalks,
            3 => CityObjectLabel::Buildings,
            4 => CityObjectLabel::Walls,
            5 => CityObjectLabel::Fences,
            6 => CityObjectLabel::Poles,
            7 => CityObjectLabel::TrafficLight,
            8 => CityObjectLabel::TrafficSigns,
            9 => CityObjectLabel::Vegetation,
            10 => CityObjectLabel::Terrain,
            11 => CityObjectLabel::Sky,
            12 => CityObjectLabel::Pedestrians,
            13 => CityObjectLabel::Rider,
            14 => CityObjectLabel::Car,
            15 => CityObjectLabel::Truck,
            16 => CityObjectLabel::Bus,
            17 => CityObjectLabel::Train,
            18 => CityObjectLabel::Motorcycle,
            19 => CityObjectLabel::Bicycle,
            20 => CityObjectLabel::Static,
            21 => CityObjectLabel::Dynamic,
            22 => CityObjectLabel::Other,
            23 => CityObjectLabel::Water,
            24 => CityObjectLabel::RoadLine,
            25 => CityObjectLabel::Ground,
            26 => CityObjectLabel::Bridge,
            27 => CityObjectLabel::RailTrack,
            28 => CityObjectLabel::GuardRail,
            _ => CityObjectLabel::Other,
        }
    }
}

impl From<CityObjectLabel> for u8 {
    fn from(label: CityObjectLabel) -> u8 {
        label as u8
    }
}

/// High-level labelled point with typed label
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LabelledPoint {
    pub location: SimpleLocation,
    pub label: CityObjectLabel,
}

impl From<SimpleLabelledPoint> for LabelledPoint {
    fn from(simple: SimpleLabelledPoint) -> Self {
        LabelledPoint {
            location: simple.location,
            label: CityObjectLabel::from(simple.label),
        }
    }
}

impl From<LabelledPoint> for SimpleLabelledPoint {
    fn from(labelled: LabelledPoint) -> Self {
        SimpleLabelledPoint {
            location: labelled.location,
            label: labelled.label.into(),
        }
    }
}

/// Extension trait for SimpleOptionalLabelledPoint
pub trait OptionalLabelledPointExt {
    /// Get the labelled point if it exists
    fn get(&self) -> Option<LabelledPoint>;

    /// Check if the optional point has a value
    fn is_some(&self) -> bool;

    /// Check if the optional point is empty
    fn is_none(&self) -> bool;
}

impl OptionalLabelledPointExt for SimpleOptionalLabelledPoint {
    fn get(&self) -> Option<LabelledPoint> {
        if self.has_value {
            Some(LabelledPoint::from(self.value))
        } else {
            None
        }
    }

    fn is_some(&self) -> bool {
        self.has_value
    }

    fn is_none(&self) -> bool {
        !self.has_value
    }
}

/// Extension trait for SimpleOptionalLocation
pub trait OptionalLocationExt {
    /// Get the location if it exists
    fn get(&self) -> Option<SimpleLocation>;

    /// Check if the optional location has a value
    fn is_some(&self) -> bool;

    /// Check if the optional location is empty
    fn is_none(&self) -> bool;
}

impl OptionalLocationExt for SimpleOptionalLocation {
    fn get(&self) -> Option<SimpleLocation> {
        if self.has_value {
            Some(self.value)
        } else {
            None
        }
    }

    fn is_some(&self) -> bool {
        self.has_value
    }

    fn is_none(&self) -> bool {
        !self.has_value
    }
}

/// Extension trait for SimpleActorList
pub trait ActorListExt {
    /// Get the list of actor IDs
    fn get_ids(&self) -> &[u32];

    /// Check if the list is empty
    fn is_empty(&self) -> bool;

    /// Get the number of actors
    fn len(&self) -> usize;
}

impl ActorListExt for SimpleActorList {
    fn get_ids(&self) -> &[u32] {
        &self.actor_ids
    }

    fn is_empty(&self) -> bool {
        self.actor_ids.is_empty()
    }

    fn len(&self) -> usize {
        self.actor_ids.len()
    }
}

/// Helper functions for creating ray casting queries
pub mod ray_casting {
    use super::*;

    /// Create a downward ray for ground projection
    pub fn downward_ray(_location: SimpleLocation, distance: f32) -> SimpleVector3D {
        SimpleVector3D {
            x: 0.0,
            y: 0.0,
            z: -distance as f64,
        }
    }

    /// Create a forward ray from a location
    pub fn forward_ray(distance: f32) -> SimpleVector3D {
        SimpleVector3D {
            x: distance as f64,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Create a ray in a specific direction
    pub fn directional_ray(x: f64, y: f64, z: f64) -> SimpleVector3D {
        SimpleVector3D { x, y, z }
    }

    /// Create a normalized ray direction
    pub fn normalized_ray(x: f64, y: f64, z: f64) -> SimpleVector3D {
        let length = (x * x + y * y + z * z).sqrt();
        if length > 0.0 {
            SimpleVector3D {
                x: x / length,
                y: y / length,
                z: z / length,
            }
        } else {
            SimpleVector3D {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            }
        }
    }
}

/// Default search distances for different queries
pub mod defaults {
    /// Default search distance for ground projection (10 meters)
    pub const GROUND_PROJECTION_DISTANCE: f32 = 10.0;

    /// Default search distance for point projection (100 meters)
    pub const POINT_PROJECTION_DISTANCE: f32 = 100.0;

    /// Default distance for traffic light queries (50 meters)
    pub const TRAFFIC_LIGHT_QUERY_DISTANCE: f64 = 50.0;

    /// Default pedestrian cross factor (100% compliance)
    pub const DEFAULT_PEDESTRIAN_CROSS_FACTOR: f32 = 1.0;
}
