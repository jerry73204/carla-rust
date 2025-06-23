//! Bounding box type for collision detection and spatial queries.

use super::{FromCxx, Location, ToCxx, Transform, Vector3D};

/// Represents a 3D axis-aligned bounding box.
///
/// This is equivalent to `carla::geom::BoundingBox` in the C++ API.
/// The bounding box is defined by its center location and extent (half-dimensions).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingBox {
    /// Center location of the bounding box
    pub location: Location,
    /// Half-dimensions (extent) of the bounding box
    pub extent: Vector3D,
}

impl BoundingBox {
    /// Create a new bounding box from center location and extent.
    ///
    /// # Arguments
    /// * `location` - Center position of the bounding box
    /// * `extent` - Half-dimensions (width/2, length/2, height/2)
    pub fn new(location: Location, extent: Vector3D) -> Self {
        Self { location, extent }
    }

    /// Create a bounding box at the origin with given extent.
    pub fn from_extent(extent: Vector3D) -> Self {
        Self::new(Location::zero(), extent)
    }

    /// Create a bounding box from minimum and maximum points.
    pub fn from_min_max(min: Location, max: Location) -> Self {
        let center = Location::new(
            (min.x + max.x) / 2.0,
            (min.y + max.y) / 2.0,
            (min.z + max.z) / 2.0,
        );
        let extent = Vector3D::new(
            ((max.x - min.x) / 2.0) as f32,
            ((max.y - min.y) / 2.0) as f32,
            ((max.z - min.z) / 2.0) as f32,
        );
        Self::new(center, extent)
    }

    /// Create a unit cube bounding box centered at origin.
    pub fn unit_cube() -> Self {
        Self::from_extent(Vector3D::new(0.5, 0.5, 0.5))
    }

    /// Get the minimum corner of the bounding box.
    pub fn min(&self) -> Location {
        Location::new(
            self.location.x - self.extent.x as f64,
            self.location.y - self.extent.y as f64,
            self.location.z - self.extent.z as f64,
        )
    }

    /// Get the maximum corner of the bounding box.
    pub fn max(&self) -> Location {
        Location::new(
            self.location.x + self.extent.x as f64,
            self.location.y + self.extent.y as f64,
            self.location.z + self.extent.z as f64,
        )
    }

    /// Get all 8 vertices of the bounding box.
    pub fn vertices(&self) -> [Location; 8] {
        let min = self.min();
        let max = self.max();

        [
            Location::new(min.x, min.y, min.z), // 0: min corner
            Location::new(max.x, min.y, min.z), // 1: +x
            Location::new(min.x, max.y, min.z), // 2: +y
            Location::new(max.x, max.y, min.z), // 3: +x+y
            Location::new(min.x, min.y, max.z), // 4: +z
            Location::new(max.x, min.y, max.z), // 5: +x+z
            Location::new(min.x, max.y, max.z), // 6: +y+z
            Location::new(max.x, max.y, max.z), // 7: max corner
        ]
    }

    /// Get the width (X extent * 2).
    pub fn width(&self) -> f32 {
        self.extent.x * 2.0
    }

    /// Get the length (Y extent * 2).
    pub fn length(&self) -> f32 {
        self.extent.y * 2.0
    }

    /// Get the height (Z extent * 2).
    pub fn height(&self) -> f32 {
        self.extent.z * 2.0
    }

    /// Get the volume of the bounding box.
    pub fn volume(&self) -> f32 {
        self.width() * self.length() * self.height()
    }

    /// Get the surface area of the bounding box.
    pub fn surface_area(&self) -> f32 {
        let w = self.width();
        let l = self.length();
        let h = self.height();
        2.0 * (w * l + w * h + l * h)
    }

    /// Check if a point is inside this bounding box.
    pub fn contains(&self, point: &Location) -> bool {
        let min = self.min();
        let max = self.max();

        point.x >= min.x
            && point.x <= max.x
            && point.y >= min.y
            && point.y <= max.y
            && point.z >= min.z
            && point.z <= max.z
    }

    /// Check if this bounding box intersects with another.
    pub fn intersects(&self, other: &BoundingBox) -> bool {
        let self_min = self.min();
        let self_max = self.max();
        let other_min = other.min();
        let other_max = other.max();

        self_max.x >= other_min.x
            && self_min.x <= other_max.x
            && self_max.y >= other_min.y
            && self_min.y <= other_max.y
            && self_max.z >= other_min.z
            && self_min.z <= other_max.z
    }

    /// Get the intersection of this bounding box with another.
    /// Returns None if they don't intersect.
    pub fn intersection(&self, other: &BoundingBox) -> Option<BoundingBox> {
        if !self.intersects(other) {
            return None;
        }

        let self_min = self.min();
        let self_max = self.max();
        let other_min = other.min();
        let other_max = other.max();

        let intersection_min = Location::new(
            self_min.x.max(other_min.x),
            self_min.y.max(other_min.y),
            self_min.z.max(other_min.z),
        );

        let intersection_max = Location::new(
            self_max.x.min(other_max.x),
            self_max.y.min(other_max.y),
            self_max.z.min(other_max.z),
        );

        Some(BoundingBox::from_min_max(
            intersection_min,
            intersection_max,
        ))
    }

    /// Get the union of this bounding box with another.
    pub fn union(&self, other: &BoundingBox) -> BoundingBox {
        let self_min = self.min();
        let self_max = self.max();
        let other_min = other.min();
        let other_max = other.max();

        let union_min = Location::new(
            self_min.x.min(other_min.x),
            self_min.y.min(other_min.y),
            self_min.z.min(other_min.z),
        );

        let union_max = Location::new(
            self_max.x.max(other_max.x),
            self_max.y.max(other_max.y),
            self_max.z.max(other_max.z),
        );

        BoundingBox::from_min_max(union_min, union_max)
    }

    /// Transform this bounding box by a given transform.
    ///
    /// Note: This creates an axis-aligned bounding box that contains
    /// the transformed oriented bounding box.
    pub fn transform(&self, transform: &Transform) -> BoundingBox {
        let vertices = self.vertices();
        let transformed_vertices: Vec<Location> = vertices
            .iter()
            .map(|v| transform.transform_location(v))
            .collect();

        let mut min_x = transformed_vertices[0].x;
        let mut max_x = transformed_vertices[0].x;
        let mut min_y = transformed_vertices[0].y;
        let mut max_y = transformed_vertices[0].y;
        let mut min_z = transformed_vertices[0].z;
        let mut max_z = transformed_vertices[0].z;

        for vertex in &transformed_vertices[1..] {
            min_x = min_x.min(vertex.x);
            max_x = max_x.max(vertex.x);
            min_y = min_y.min(vertex.y);
            max_y = max_y.max(vertex.y);
            min_z = min_z.min(vertex.z);
            max_z = max_z.max(vertex.z);
        }

        BoundingBox::from_min_max(
            Location::new(min_x, min_y, min_z),
            Location::new(max_x, max_y, max_z),
        )
    }

    /// Calculate the distance from a point to this bounding box.
    /// Returns 0 if the point is inside the box.
    pub fn distance_to_point(&self, point: &Location) -> f64 {
        let min = self.min();
        let max = self.max();

        let dx = (min.x - point.x).max(0.0).max(point.x - max.x);
        let dy = (min.y - point.y).max(0.0).max(point.y - max.y);
        let dz = (min.z - point.z).max(0.0).max(point.z - max.z);

        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Get the closest point on the bounding box to a given point.
    pub fn closest_point(&self, point: &Location) -> Location {
        let min = self.min();
        let max = self.max();

        Location::new(
            point.x.clamp(min.x, max.x),
            point.y.clamp(min.y, max.y),
            point.z.clamp(min.z, max.z),
        )
    }

    /// Expand the bounding box by a given amount in all directions.
    pub fn expand(&self, amount: f32) -> BoundingBox {
        BoundingBox::new(
            self.location,
            Vector3D::new(
                self.extent.x + amount,
                self.extent.y + amount,
                self.extent.z + amount,
            ),
        )
    }

    /// Scale the bounding box by a factor.
    pub fn scale(&self, factor: f32) -> BoundingBox {
        BoundingBox::new(
            self.location,
            Vector3D::new(
                self.extent.x * factor,
                self.extent.y * factor,
                self.extent.z * factor,
            ),
        )
    }

    /// Check if this bounding box is approximately equal to another.
    pub fn approx_eq(
        &self,
        other: &BoundingBox,
        location_epsilon: f64,
        extent_epsilon: f32,
    ) -> bool {
        self.location.approx_eq(&other.location, location_epsilon)
            && self.extent.approx_eq(&other.extent, extent_epsilon)
    }
}

impl Default for BoundingBox {
    fn default() -> Self {
        Self::unit_cube()
    }
}

impl From<Vector3D> for BoundingBox {
    fn from(extent: Vector3D) -> Self {
        Self::from_extent(extent)
    }
}

impl From<(Location, Vector3D)> for BoundingBox {
    fn from((location, extent): (Location, Vector3D)) -> Self {
        Self::new(location, extent)
    }
}

impl From<BoundingBox> for (Location, Vector3D) {
    fn from(bbox: BoundingBox) -> Self {
        (bbox.location, bbox.extent)
    }
}

impl std::fmt::Display for BoundingBox {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "BoundingBox(center={}, extent={})",
            self.location, self.extent
        )
    }
}

// Conversion to/from carla-sys types
impl FromCxx<carla_sys::SimpleBoundingBox> for BoundingBox {
    fn from_cxx(value: carla_sys::SimpleBoundingBox) -> Self {
        Self::new(
            Location::from_cxx(value.location),
            Vector3D::from_cxx(value.extent),
        )
    }
}

impl ToCxx<carla_sys::SimpleBoundingBox> for BoundingBox {
    fn to_cxx(&self) -> carla_sys::SimpleBoundingBox {
        carla_sys::SimpleBoundingBox {
            location: self.location.to_cxx(),
            extent: self.extent.to_cxx(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Rotation;
    use approx::assert_relative_eq;

    // Tests corresponding to C++ test_geom.cpp

    #[test]
    fn test_bbox_get_local_vertices_get_world_vertices_coherence() {
        // Corresponds to C++ TEST(geom, bbox_get_local_vertices_get_world_vertices_coherence)

        // Create a bounding box centered at origin with extent (1,2,3)
        let bbox = BoundingBox::new(Location::new(0.0, 0.0, 0.0), Vector3D::new(1.0, 2.0, 3.0));

        // Create a transform with rotation and translation
        let transform = Transform::new(
            Location::new(10.0, 20.0, 30.0),
            Rotation::new(10.0, 25.0, 17.0),
        );

        // Get local vertices
        let local_vertices = bbox.vertices();

        // Transform each vertex manually
        let mut manually_transformed_vertices = Vec::new();
        for vertex in &local_vertices {
            manually_transformed_vertices.push(transform.transform_location(vertex));
        }

        // Get world vertices using the transform method
        let transformed_bbox = bbox.transform(&transform);
        let _world_vertices = transformed_bbox.vertices();

        // The axis-aligned bounding box after transformation should contain all transformed vertices
        for transformed_vertex in &manually_transformed_vertices {
            // Check that each manually transformed vertex is within the world bounding box
            let min = transformed_bbox.min();
            let max = transformed_bbox.max();

            assert!(transformed_vertex.x >= min.x - 0.001);
            assert!(transformed_vertex.x <= max.x + 0.001);
            assert!(transformed_vertex.y >= min.y - 0.001);
            assert!(transformed_vertex.y <= max.y + 0.001);
            assert!(transformed_vertex.z >= min.z - 0.001);
            assert!(transformed_vertex.z <= max.z + 0.001);
        }
    }

    #[test]
    fn test_bounding_box_creation() {
        // Test creating bounding box from center and extent
        let bbox = BoundingBox::new(Location::new(5.0, 10.0, 15.0), Vector3D::new(1.0, 2.0, 3.0));

        assert_relative_eq!(bbox.location.x, 5.0, epsilon = 0.001);
        assert_relative_eq!(bbox.location.y, 10.0, epsilon = 0.001);
        assert_relative_eq!(bbox.location.z, 15.0, epsilon = 0.001);
        assert_relative_eq!(bbox.extent.x, 1.0, epsilon = 0.001);
        assert_relative_eq!(bbox.extent.y, 2.0, epsilon = 0.001);
        assert_relative_eq!(bbox.extent.z, 3.0, epsilon = 0.001);
    }

    #[test]
    fn test_bounding_box_min_max() {
        // Test min/max corners
        let bbox = BoundingBox::new(
            Location::new(10.0, 20.0, 30.0),
            Vector3D::new(5.0, 10.0, 15.0),
        );

        let min = bbox.min();
        assert_relative_eq!(min.x, 5.0, epsilon = 0.001);
        assert_relative_eq!(min.y, 10.0, epsilon = 0.001);
        assert_relative_eq!(min.z, 15.0, epsilon = 0.001);

        let max = bbox.max();
        assert_relative_eq!(max.x, 15.0, epsilon = 0.001);
        assert_relative_eq!(max.y, 30.0, epsilon = 0.001);
        assert_relative_eq!(max.z, 45.0, epsilon = 0.001);
    }

    #[test]
    fn test_bounding_box_vertices() {
        // Test all 8 vertices
        let bbox = BoundingBox::new(Location::new(0.0, 0.0, 0.0), Vector3D::new(1.0, 1.0, 1.0));

        let vertices = bbox.vertices();
        assert_eq!(vertices.len(), 8);

        // Check that we have all 8 unique corners of a cube
        let expected_vertices = vec![
            Location::new(-1.0, -1.0, -1.0),
            Location::new(1.0, -1.0, -1.0),
            Location::new(-1.0, 1.0, -1.0),
            Location::new(1.0, 1.0, -1.0),
            Location::new(-1.0, -1.0, 1.0),
            Location::new(1.0, -1.0, 1.0),
            Location::new(-1.0, 1.0, 1.0),
            Location::new(1.0, 1.0, 1.0),
        ];

        for expected in &expected_vertices {
            let found = vertices.iter().any(|v| v.approx_eq(expected, 0.001));
            assert!(found);
        }
    }

    #[test]
    fn test_bounding_box_contains() {
        // Test point containment
        let bbox = BoundingBox::new(
            Location::new(0.0, 0.0, 0.0),
            Vector3D::new(10.0, 10.0, 10.0),
        );

        // Points inside
        assert!(bbox.contains(&Location::new(0.0, 0.0, 0.0)));
        assert!(bbox.contains(&Location::new(5.0, 5.0, 5.0)));
        assert!(bbox.contains(&Location::new(-5.0, -5.0, -5.0)));

        // Points on boundary
        assert!(bbox.contains(&Location::new(10.0, 0.0, 0.0)));
        assert!(bbox.contains(&Location::new(0.0, 10.0, 0.0)));
        assert!(bbox.contains(&Location::new(0.0, 0.0, 10.0)));

        // Points outside
        assert!(!bbox.contains(&Location::new(11.0, 0.0, 0.0)));
        assert!(!bbox.contains(&Location::new(0.0, 11.0, 0.0)));
        assert!(!bbox.contains(&Location::new(0.0, 0.0, 11.0)));
    }

    #[test]
    fn test_bounding_box_transform() {
        // Test transforming a bounding box
        let bbox = BoundingBox::new(Location::new(0.0, 0.0, 0.0), Vector3D::new(1.0, 1.0, 1.0));

        // Pure translation
        let transform = Transform::from_location(Location::new(10.0, 20.0, 30.0));
        let transformed = bbox.transform(&transform);

        assert_relative_eq!(transformed.location.x, 10.0, epsilon = 0.001);
        assert_relative_eq!(transformed.location.y, 20.0, epsilon = 0.001);
        assert_relative_eq!(transformed.location.z, 30.0, epsilon = 0.001);
        assert_relative_eq!(transformed.extent.x, 1.0, epsilon = 0.001);
        assert_relative_eq!(transformed.extent.y, 1.0, epsilon = 0.001);
        assert_relative_eq!(transformed.extent.z, 1.0, epsilon = 0.001);
    }
}
