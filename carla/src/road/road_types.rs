//! Road-related enums and types.

/// Lane change permissions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneChange {
    /// No lane change allowed
    None,
    /// Lane change to the right allowed
    Right,
    /// Lane change to the left allowed
    Left,
    /// Lane change in both directions allowed
    Both,
}

/// Lane types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneType {
    /// None
    None,
    /// Driving lane
    Driving,
    /// Stop lane
    Stop,
    /// Shoulder
    Shoulder,
    /// Biking lane
    Biking,
    /// Sidewalk
    Sidewalk,
    /// Border
    Border,
    /// Restricted
    Restricted,
    /// Parking
    Parking,
    /// Bidirectional
    Bidirectional,
    /// Median
    Median,
    /// Special1
    Special1,
    /// Special2
    Special2,
    /// Special3
    Special3,
    /// RoadWorks
    RoadWorks,
    /// Tram
    Tram,
    /// Rail
    Rail,
    /// Entry
    Entry,
    /// Exit
    Exit,
    /// OffRamp
    OffRamp,
    /// OnRamp
    OnRamp,
    /// Any
    Any,
}

/// Lane marking types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneMarkingType {
    /// None
    None,
    /// Other
    Other,
    /// Broken
    Broken,
    /// Solid
    Solid,
    /// SolidSolid
    SolidSolid,
    /// SolidBroken
    SolidBroken,
    /// BrokenSolid
    BrokenSolid,
    /// BrokenBroken
    BrokenBroken,
    /// BottsDots
    BottsDots,
    /// Grass
    Grass,
    /// Curb
    Curb,
}

/// Lane marking colors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneMarkingColor {
    /// Standard (white)
    Standard,
    /// Blue
    Blue,
    /// Green
    Green,
    /// Red
    Red,
    /// White
    White,
    /// Yellow
    Yellow,
    /// Other
    Other,
}
