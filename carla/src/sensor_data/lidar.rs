//! LiDAR sensor implementations.

use crate::{
    geom::{Location, Transform},
    sensor_data::SensorData,
    time::Timestamp,
};

/// LiDAR point cloud data.
#[derive(Debug, Clone)]
pub struct LiDARData {
    /// Sensor timestamp
    pub timestamp: Timestamp,
    /// Sensor transform when captured
    pub transform: Transform,
    /// Sensor ID
    pub sensor_id: u32,
    /// Number of channels
    pub channels: u32,
    /// Horizontal field of view in degrees
    pub horizontal_fov: f32,
    /// Points per second
    pub points_per_second: u32,
    /// Point cloud data (x, y, z, intensity)
    pub points: Vec<LiDARPoint>,
}

/// Individual LiDAR point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiDARPoint {
    /// X coordinate
    pub x: f32,
    /// Y coordinate  
    pub y: f32,
    /// Z coordinate
    pub z: f32,
    /// Intensity value
    pub intensity: f32,
}

impl LiDARPoint {
    /// Create a new LiDAR point.
    pub fn new(x: f32, y: f32, z: f32, intensity: f32) -> Self {
        Self { x, y, z, intensity }
    }

    /// Get the distance from origin.
    pub fn distance(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Get the 2D distance (ignoring Z).
    pub fn distance_2d(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// Convert to Location.
    pub fn to_location(&self) -> Location {
        Location::new(self.x as f64, self.y as f64, self.z as f64)
    }

    /// Get azimuth angle in radians.
    pub fn azimuth(&self) -> f32 {
        self.y.atan2(self.x)
    }

    /// Get elevation angle in radians.
    pub fn elevation(&self) -> f32 {
        self.z.atan2(self.distance_2d())
    }
}

impl SensorData for LiDARData {
    fn timestamp(&self) -> Timestamp {
        self.timestamp
    }

    fn transform(&self) -> Transform {
        self.transform
    }

    fn sensor_id(&self) -> u32 {
        self.sensor_id
    }

    fn size(&self) -> usize {
        self.points.len() * std::mem::size_of::<LiDARPoint>()
    }
}

impl LiDARData {
    /// Create LiDARData from carla-sys LiDARDataFull
    pub fn from_cxx(cxx_data: carla_sys::sensor::LiDARDataFull) -> Self {
        let points = cxx_data
            .points
            .iter()
            .map(|p| LiDARPoint::new(p.x, p.y, p.z, p.intensity))
            .collect();

        let transform = crate::geom::Transform::from(cxx_data.transform);
        let timestamp = crate::time::Timestamp::from(cxx_data.timestamp);

        Self {
            timestamp,
            transform,
            sensor_id: cxx_data.sensor_id,
            channels: cxx_data.channels,
            horizontal_fov: cxx_data.horizontal_fov,
            points_per_second: cxx_data.points_per_second,
            points,
        }
    }

    /// Get point cloud as location array.
    pub fn get_locations(&self) -> Vec<Location> {
        self.points
            .iter()
            .map(|p| Location::new(p.x as f64, p.y as f64, p.z as f64))
            .collect()
    }

    /// Filter points by distance range.
    pub fn filter_by_distance(&self, min_distance: f32, max_distance: f32) -> Vec<LiDARPoint> {
        self.points
            .iter()
            .filter(|p| {
                let dist = p.distance();
                dist >= min_distance && dist <= max_distance
            })
            .copied()
            .collect()
    }

    /// Filter points by intensity range.
    pub fn filter_by_intensity(&self, min_intensity: f32, max_intensity: f32) -> Vec<LiDARPoint> {
        self.points
            .iter()
            .filter(|p| p.intensity >= min_intensity && p.intensity <= max_intensity)
            .copied()
            .collect()
    }

    /// Filter points by height (Z coordinate) range.
    pub fn filter_by_height(&self, min_height: f32, max_height: f32) -> Vec<LiDARPoint> {
        self.points
            .iter()
            .filter(|p| p.z >= min_height && p.z <= max_height)
            .copied()
            .collect()
    }

    /// Get points within a 2D radius.
    pub fn filter_by_radius_2d(
        &self,
        center_x: f32,
        center_y: f32,
        radius: f32,
    ) -> Vec<LiDARPoint> {
        let radius_sq = radius * radius;
        self.points
            .iter()
            .filter(|p| {
                let dx = p.x - center_x;
                let dy = p.y - center_y;
                dx * dx + dy * dy <= radius_sq
            })
            .copied()
            .collect()
    }

    /// Get points within a 3D sphere.
    pub fn filter_by_sphere(&self, center: &Location, radius: f32) -> Vec<LiDARPoint> {
        let radius_sq = radius * radius;
        let cx = center.x as f32;
        let cy = center.y as f32;
        let cz = center.z as f32;

        self.points
            .iter()
            .filter(|p| {
                let dx = p.x - cx;
                let dy = p.y - cy;
                let dz = p.z - cz;
                dx * dx + dy * dy + dz * dz <= radius_sq
            })
            .copied()
            .collect()
    }

    /// Get the bounding box of all points.
    pub fn get_bounding_box(&self) -> Option<crate::geom::BoundingBox> {
        if self.points.is_empty() {
            return None;
        }

        let mut min_x = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut min_y = f32::INFINITY;
        let mut max_y = f32::NEG_INFINITY;
        let mut min_z = f32::INFINITY;
        let mut max_z = f32::NEG_INFINITY;

        for point in &self.points {
            min_x = min_x.min(point.x);
            max_x = max_x.max(point.x);
            min_y = min_y.min(point.y);
            max_y = max_y.max(point.y);
            min_z = min_z.min(point.z);
            max_z = max_z.max(point.z);
        }

        let center = Location::new(
            ((min_x + max_x) / 2.0) as f64,
            ((min_y + max_y) / 2.0) as f64,
            ((min_z + max_z) / 2.0) as f64,
        );

        let extent = crate::geom::Vector3D::new(
            (max_x - min_x) / 2.0,
            (max_y - min_y) / 2.0,
            (max_z - min_z) / 2.0,
        );

        Some(crate::geom::BoundingBox::new(center, extent))
    }

    /// Convert to numpy-style array format (for Python interop).
    pub fn to_array(&self) -> Vec<[f32; 4]> {
        self.points
            .iter()
            .map(|p| [p.x, p.y, p.z, p.intensity])
            .collect()
    }

    /// Get statistics about the point cloud.
    pub fn statistics(&self) -> LiDARStatistics {
        if self.points.is_empty() {
            return LiDARStatistics::default();
        }

        let mut total_distance = 0.0;
        let mut total_intensity = 0.0;
        let mut min_distance = f32::INFINITY;
        let mut max_distance: f32 = 0.0;
        let mut min_intensity = f32::INFINITY;
        let mut max_intensity: f32 = 0.0;

        for point in &self.points {
            let distance = point.distance();
            total_distance += distance;
            total_intensity += point.intensity;

            min_distance = min_distance.min(distance);
            max_distance = max_distance.max(distance);
            min_intensity = min_intensity.min(point.intensity);
            max_intensity = max_intensity.max(point.intensity);
        }

        let count = self.points.len();
        LiDARStatistics {
            point_count: count,
            avg_distance: total_distance / count as f32,
            min_distance,
            max_distance,
            avg_intensity: total_intensity / count as f32,
            min_intensity,
            max_intensity,
        }
    }
}

/// LiDAR point cloud statistics.
#[derive(Debug, Clone, PartialEq)]
pub struct LiDARStatistics {
    /// Number of points
    pub point_count: usize,
    /// Average distance from sensor
    pub avg_distance: f32,
    /// Minimum distance
    pub min_distance: f32,
    /// Maximum distance
    pub max_distance: f32,
    /// Average intensity
    pub avg_intensity: f32,
    /// Minimum intensity
    pub min_intensity: f32,
    /// Maximum intensity
    pub max_intensity: f32,
}

impl Default for LiDARStatistics {
    fn default() -> Self {
        Self {
            point_count: 0,
            avg_distance: 0.0,
            min_distance: 0.0,
            max_distance: 0.0,
            avg_intensity: 0.0,
            min_intensity: 0.0,
            max_intensity: 0.0,
        }
    }
}

/// Semantic LiDAR sensor data.
#[derive(Debug, Clone)]
pub struct SemanticLiDARData {
    /// Sensor timestamp
    pub timestamp: Timestamp,
    /// Sensor transform when captured
    pub transform: Transform,
    /// Sensor ID
    pub sensor_id: u32,
    /// Current horizontal rotation angle
    pub horizontal_angle: f32,
    /// Number of laser channels
    pub channel_count: u32,
    /// Semantic detections
    pub detections: Vec<SemanticLiDARDetection>,
}

/// Individual semantic LiDAR detection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SemanticLiDARDetection {
    /// 3D point location
    pub point: Location,
    /// Cosine of incidence angle
    pub cos_inc_angle: f32,
    /// Object instance index
    pub object_idx: u32,
    /// Semantic tag/label
    pub object_tag: u32,
}

impl SensorData for SemanticLiDARData {
    fn timestamp(&self) -> Timestamp {
        self.timestamp
    }

    fn transform(&self) -> Transform {
        self.transform
    }

    fn sensor_id(&self) -> u32 {
        self.sensor_id
    }

    fn size(&self) -> usize {
        self.detections.len() * std::mem::size_of::<SemanticLiDARDetection>()
    }
}

impl SemanticLiDARData {
    /// Create SemanticLiDARData from carla-sys SemanticLidarData
    pub fn from_cxx(cxx_data: carla_sys::sensor::SemanticLidarData) -> Self {
        let detections = cxx_data
            .detections
            .iter()
            .map(|d| SemanticLiDARDetection {
                point: Location::new(d.point.x, d.point.y, d.point.z),
                cos_inc_angle: d.cos_inc_angle,
                object_idx: d.object_idx,
                object_tag: d.object_tag,
            })
            .collect();

        Self {
            timestamp: cxx_data.timestamp.into(),
            transform: Transform::from(cxx_data.transform),
            sensor_id: cxx_data.sensor_id,
            horizontal_angle: cxx_data.horizontal_angle,
            channel_count: cxx_data.channel_count,
            detections,
        }
    }

    /// Filter detections by semantic tag/label.
    pub fn filter_by_tag(&self, tag: u32) -> Vec<SemanticLiDARDetection> {
        self.detections
            .iter()
            .filter(|d| d.object_tag == tag)
            .copied()
            .collect()
    }

    /// Filter detections by object instance.
    pub fn filter_by_instance(&self, instance: u32) -> Vec<SemanticLiDARDetection> {
        self.detections
            .iter()
            .filter(|d| d.object_idx == instance)
            .copied()
            .collect()
    }

    /// Get all unique semantic tags in the data.
    pub fn get_unique_tags(&self) -> Vec<u32> {
        let mut tags: Vec<u32> = self.detections.iter().map(|d| d.object_tag).collect();
        tags.sort_unstable();
        tags.dedup();
        tags
    }
}
