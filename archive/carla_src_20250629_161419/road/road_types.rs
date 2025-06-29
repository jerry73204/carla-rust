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

impl LaneChange {
    /// Convert from u8 value
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => LaneChange::None,
            1 => LaneChange::Right,
            2 => LaneChange::Left,
            3 => LaneChange::Both,
            _ => LaneChange::None,
        }
    }
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

impl LaneType {
    /// Convert from u8 value
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => LaneType::None,
            1 => LaneType::Driving,
            2 => LaneType::Stop,
            3 => LaneType::Shoulder,
            4 => LaneType::Biking,
            5 => LaneType::Sidewalk,
            6 => LaneType::Border,
            7 => LaneType::Restricted,
            8 => LaneType::Parking,
            9 => LaneType::Bidirectional,
            10 => LaneType::Median,
            11 => LaneType::Special1,
            12 => LaneType::Special2,
            13 => LaneType::Special3,
            14 => LaneType::RoadWorks,
            15 => LaneType::Tram,
            16 => LaneType::Rail,
            17 => LaneType::Entry,
            18 => LaneType::Exit,
            19 => LaneType::OffRamp,
            20 => LaneType::OnRamp,
            21 => LaneType::Any,
            _ => LaneType::None,
        }
    }
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

impl LaneMarkingType {
    /// Convert from u32 value
    pub fn from_u32(value: u32) -> Self {
        match value {
            0 => LaneMarkingType::None,
            1 => LaneMarkingType::Other,
            2 => LaneMarkingType::Broken,
            3 => LaneMarkingType::Solid,
            4 => LaneMarkingType::SolidSolid,
            5 => LaneMarkingType::SolidBroken,
            6 => LaneMarkingType::BrokenSolid,
            7 => LaneMarkingType::BrokenBroken,
            8 => LaneMarkingType::BottsDots,
            9 => LaneMarkingType::Grass,
            10 => LaneMarkingType::Curb,
            _ => LaneMarkingType::Other,
        }
    }
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
