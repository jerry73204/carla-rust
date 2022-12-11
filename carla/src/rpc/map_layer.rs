#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[repr(u16)]
pub enum MapLayer {
    None = 0,
    Buildings = 0x1,
    Decals = 0x1 << 1,
    Foliage = 0x1 << 2,
    Ground = 0x1 << 3,
    ParkedVehicles = 0x1 << 4,
    Particles = 0x1 << 5,
    Props = 0x1 << 6,
    StreetLights = 0x1 << 7,
    Walls = 0x1 << 8,
    All = 0xFFFF,
}
