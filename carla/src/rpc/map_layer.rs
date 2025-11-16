/// Map layer flags for toggling visual elements in the simulation.
///
/// Map layers allow selective loading/unloading of visual elements to optimize
/// performance or focus on specific aspects of the environment. Layers can be
/// combined using bitwise OR operations.
#[cfg_attr(
    carla_version_0916,
    doc = " See [carla.MapLayer](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.MapLayer) in the Python API."
)]
#[cfg_attr(
    carla_version_0915,
    doc = " See [carla.MapLayer](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.MapLayer) in the Python API."
)]
#[cfg_attr(
    carla_version_0914,
    doc = " See [carla.MapLayer](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.MapLayer) in the Python API."
)]
///
/// # Examples
///
/// ```no_run
/// use carla::{client::Client, rpc::MapLayer};
///
/// let client = Client::default();
/// let world = client.world();
///
/// // Load only buildings and ground for performance
/// world.load_map_layer(MapLayer::Buildings as u16 | MapLayer::Ground as u16);
///
/// // Unload foliage for better visibility
/// world.unload_map_layer(MapLayer::Foliage as u16);
/// ```
///
/// # Performance Considerations
///
/// - Disabling layers can significantly improve performance, especially `Foliage` and `Particles`
/// - Useful for headless simulations where visuals aren't needed
/// - Can help identify which visual elements impact frame rate
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[repr(u16)]
pub enum MapLayer {
    /// No layers (empty mask).
    None = 0,
    /// Building structures.
    Buildings = 0x1,
    /// Road decals and markings.
    Decals = 0x1 << 1,
    /// Trees, grass, and vegetation.
    Foliage = 0x1 << 2,
    /// Ground surface.
    Ground = 0x1 << 3,
    /// Static parked vehicles in the environment.
    ParkedVehicles = 0x1 << 4,
    /// Particle effects (smoke, dust, etc.).
    Particles = 0x1 << 5,
    /// Props and street furniture.
    Props = 0x1 << 6,
    /// Street lights and lamp posts.
    StreetLights = 0x1 << 7,
    /// Walls and barriers.
    Walls = 0x1 << 8,
    /// All layers enabled (full visual fidelity).
    All = 0xFFFF,
}
