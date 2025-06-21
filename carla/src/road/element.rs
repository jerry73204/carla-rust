//! Road element types for OpenDrive parsing.
//!
//! This module contains types that represent various OpenDrive road elements
//! like elevation profiles, geometry, road marks, etc.

/// Road elevation information at a specific s-coordinate.
#[derive(Debug, Clone, PartialEq)]
pub struct RoadElevation {
    /// Distance along the road reference line
    pub s: f64,
    /// Elevation at s
    pub a: f64,
    /// Slope at s
    pub b: f64,
    /// Vertical curvature at s
    pub c: f64,
    /// Derivative of vertical curvature at s
    pub d: f64,
}

impl RoadElevation {
    /// Create a new road elevation record.
    pub fn new(s: f64, a: f64, b: f64, c: f64, d: f64) -> Self {
        Self { s, a, b, c, d }
    }

    /// Calculate elevation at a given s-coordinate.
    pub fn elevation_at(&self, s: f64) -> f64 {
        let ds = s - self.s;
        self.a + self.b * ds + self.c * ds * ds + self.d * ds * ds * ds
    }
}

/// Road geometry types.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GeometryType {
    /// Straight line
    Line,
    /// Circular arc
    Arc,
    /// Spiral/clothoid
    Spiral,
    /// Polynomial curve
    Poly3,
    /// Parametric cubic curve
    ParamPoly3,
}

/// Road geometry information.
#[derive(Debug, Clone, PartialEq)]
pub struct RoadGeometry {
    /// Start position along the road
    pub s: f64,
    /// Start position in x
    pub x: f64,
    /// Start position in y
    pub y: f64,
    /// Start orientation (heading)
    pub hdg: f64,
    /// Length of the geometry
    pub length: f64,
    /// Type of geometry
    pub geometry_type: GeometryType,
}

impl RoadGeometry {
    /// Create a new road geometry record.
    pub fn new(s: f64, x: f64, y: f64, hdg: f64, length: f64, geometry_type: GeometryType) -> Self {
        Self {
            s,
            x,
            y,
            hdg,
            length,
            geometry_type,
        }
    }
}

/// Road mark types.
#[derive(Debug, Clone, PartialEq)]
pub enum RoadMarkType {
    None,
    Solid,
    Broken,
    SolidSolid,
    SolidBroken,
    BrokenSolid,
    BrokenBroken,
    BottsDots,
    Grass,
    Curb,
    Custom(String),
}

impl RoadMarkType {
    /// Parse from string.
    pub fn from_str(s: &str) -> Self {
        match s {
            "none" => Self::None,
            "solid" => Self::Solid,
            "broken" => Self::Broken,
            "solid solid" => Self::SolidSolid,
            "solid broken" => Self::SolidBroken,
            "broken solid" => Self::BrokenSolid,
            "broken broken" => Self::BrokenBroken,
            "botts dots" => Self::BottsDots,
            "grass" => Self::Grass,
            "curb" => Self::Curb,
            other => Self::Custom(other.to_string()),
        }
    }

    /// Convert to string.
    pub fn as_str(&self) -> &str {
        match self {
            Self::None => "none",
            Self::Solid => "solid",
            Self::Broken => "broken",
            Self::SolidSolid => "solid solid",
            Self::SolidBroken => "solid broken",
            Self::BrokenSolid => "broken solid",
            Self::BrokenBroken => "broken broken",
            Self::BottsDots => "botts dots",
            Self::Grass => "grass",
            Self::Curb => "curb",
            Self::Custom(s) => s,
        }
    }
}

/// Road mark colors.
#[derive(Debug, Clone, PartialEq)]
pub enum RoadMarkColor {
    Standard,
    Blue,
    Green,
    Red,
    White,
    Yellow,
    Orange,
    Custom(String),
}

impl RoadMarkColor {
    /// Parse from string.
    pub fn from_str(s: &str) -> Self {
        match s {
            "standard" => Self::Standard,
            "blue" => Self::Blue,
            "green" => Self::Green,
            "red" => Self::Red,
            "white" => Self::White,
            "yellow" => Self::Yellow,
            "orange" => Self::Orange,
            other => Self::Custom(other.to_string()),
        }
    }

    /// Convert to string.
    pub fn as_str(&self) -> &str {
        match self {
            Self::Standard => "standard",
            Self::Blue => "blue",
            Self::Green => "green",
            Self::Red => "red",
            Self::White => "white",
            Self::Yellow => "yellow",
            Self::Orange => "orange",
            Self::Custom(s) => s,
        }
    }
}

/// Road mark lane change permissions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RoadMarkLaneChange {
    None,
    Increase,
    Decrease,
    Both,
}

impl RoadMarkLaneChange {
    /// Parse from string.
    pub fn from_str(s: &str) -> Self {
        match s {
            "none" => Self::None,
            "increase" => Self::Increase,
            "decrease" => Self::Decrease,
            "both" => Self::Both,
            _ => Self::None,
        }
    }

    /// Convert to string.
    pub fn as_str(&self) -> &str {
        match self {
            Self::None => "none",
            Self::Increase => "increase",
            Self::Decrease => "decrease",
            Self::Both => "both",
        }
    }
}

/// Road mark information.
#[derive(Debug, Clone, PartialEq)]
pub struct RoadMark {
    /// Start position along the lane
    pub s_offset: f64,
    /// Type of road mark
    pub mark_type: RoadMarkType,
    /// Material (standard, etc.)
    pub material: String,
    /// Color of the mark
    pub color: RoadMarkColor,
    /// Lane change permissions
    pub lane_change: RoadMarkLaneChange,
}

impl RoadMark {
    /// Create a new road mark.
    pub fn new(
        s_offset: f64,
        mark_type: RoadMarkType,
        material: String,
        color: RoadMarkColor,
        lane_change: RoadMarkLaneChange,
    ) -> Self {
        Self {
            s_offset,
            mark_type,
            material,
            color,
            lane_change,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_road_elevation() {
        // Test elevation calculation
        let elevation = RoadElevation::new(0.0, 10.0, 0.1, 0.01, 0.001);

        // At s = 0, elevation should be 'a'
        assert_eq!(elevation.elevation_at(0.0), 10.0);

        // At s = 10, elevation = a + b*10 + c*100 + d*1000
        let expected = 10.0 + 0.1 * 10.0 + 0.01 * 100.0 + 0.001 * 1000.0;
        assert_eq!(elevation.elevation_at(10.0), expected);
    }

    #[test]
    fn test_road_mark_types() {
        assert_eq!(RoadMarkType::from_str("solid"), RoadMarkType::Solid);
        assert_eq!(RoadMarkType::from_str("broken"), RoadMarkType::Broken);
        assert_eq!(
            RoadMarkType::from_str("unknown"),
            RoadMarkType::Custom("unknown".to_string())
        );

        assert_eq!(RoadMarkType::Solid.as_str(), "solid");
        assert_eq!(RoadMarkType::Broken.as_str(), "broken");
    }

    #[test]
    fn test_road_mark_colors() {
        assert_eq!(RoadMarkColor::from_str("white"), RoadMarkColor::White);
        assert_eq!(RoadMarkColor::from_str("yellow"), RoadMarkColor::Yellow);
        assert_eq!(
            RoadMarkColor::from_str("custom"),
            RoadMarkColor::Custom("custom".to_string())
        );

        assert_eq!(RoadMarkColor::White.as_str(), "white");
        assert_eq!(RoadMarkColor::Yellow.as_str(), "yellow");
    }

    #[test]
    fn test_road_mark_lane_change() {
        assert_eq!(
            RoadMarkLaneChange::from_str("none"),
            RoadMarkLaneChange::None
        );
        assert_eq!(
            RoadMarkLaneChange::from_str("both"),
            RoadMarkLaneChange::Both
        );
        assert_eq!(
            RoadMarkLaneChange::from_str("invalid"),
            RoadMarkLaneChange::None
        );

        assert_eq!(RoadMarkLaneChange::None.as_str(), "none");
        assert_eq!(RoadMarkLaneChange::Both.as_str(), "both");
    }

    #[test]
    fn test_geometry_types() {
        // Test all geometry type variants
        let line = GeometryType::Line;
        let arc = GeometryType::Arc;
        let spiral = GeometryType::Spiral;
        let poly3 = GeometryType::Poly3;
        let param_poly3 = GeometryType::ParamPoly3;

        // Verify they are distinct
        assert_ne!(line, arc);
        assert_ne!(arc, spiral);
        assert_ne!(spiral, poly3);
        assert_ne!(poly3, param_poly3);
    }

    #[test]
    fn test_road_geometry_creation() {
        let geom = RoadGeometry::new(
            10.0,   // s
            100.0,  // x
            200.0,  // y
            1.5708, // hdg (90 degrees)
            50.0,   // length
            GeometryType::Line,
        );

        assert_eq!(geom.s, 10.0);
        assert_eq!(geom.x, 100.0);
        assert_eq!(geom.y, 200.0);
        assert_eq!(geom.hdg, 1.5708);
        assert_eq!(geom.length, 50.0);
        assert_eq!(geom.geometry_type, GeometryType::Line);
    }

    #[test]
    fn test_road_mark_creation() {
        let mark = RoadMark::new(
            5.0,
            RoadMarkType::Broken,
            "standard".to_string(),
            RoadMarkColor::Yellow,
            RoadMarkLaneChange::Both,
        );

        assert_eq!(mark.s_offset, 5.0);
        assert_eq!(mark.mark_type, RoadMarkType::Broken);
        assert_eq!(mark.material, "standard");
        assert_eq!(mark.color, RoadMarkColor::Yellow);
        assert_eq!(mark.lane_change, RoadMarkLaneChange::Both);
    }

    #[test]
    fn test_elevation_polynomial() {
        // Test elevation calculation with different polynomial coefficients
        let elev1 = RoadElevation::new(0.0, 10.0, 0.0, 0.0, 0.0);
        assert_eq!(elev1.elevation_at(5.0), 10.0); // Constant elevation

        let elev2 = RoadElevation::new(0.0, 0.0, 1.0, 0.0, 0.0);
        assert_eq!(elev2.elevation_at(10.0), 10.0); // Linear slope

        let elev3 = RoadElevation::new(0.0, 0.0, 0.0, 0.1, 0.0);
        assert_eq!(elev3.elevation_at(10.0), 10.0); // Quadratic

        let elev4 = RoadElevation::new(10.0, 20.0, 0.5, 0.01, 0.001);
        let s = 20.0;
        let ds = s - 10.0;
        let expected = 20.0 + 0.5 * ds + 0.01 * ds * ds + 0.001 * ds * ds * ds;
        assert_eq!(elev4.elevation_at(s), expected);
    }
}
