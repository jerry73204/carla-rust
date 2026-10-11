# CARLA Version Differences (0.9.14, 0.9.15, 0.9.16)

This document describes API differences between CARLA versions 0.9.14, 0.9.15, and 0.9.16 as exposed in the carla-rust bindings. The library uses conditional compilation (`cfg` attributes) to support all three versions.

## Build Configuration

The CARLA version is specified using the `CARLA_VERSION` environment variable during build:

```bash
export CARLA_VERSION=0.9.14  # or 0.9.15, or 0.9.16
cargo build
```

The build system automatically sets the following cfg flags:
- `#[cfg(carla_0916)]` - Only enabled for CARLA 0.9.16
- `#[cfg(carla_0915)]` - Only enabled for CARLA 0.9.15 or newer (TBD)

## API Differences by Module

### carla::geom::BoundingBox

#### Field: `actor_id` (0.9.16 only)

**Added in:** 0.9.16
**Availability:** `#[cfg(carla_0916)]`

```rust
// 0.9.16 only
pub struct BoundingBox {
    pub location: Location,
    pub rotation: Rotation,
    pub extent: Vector3D,
    #[cfg(carla_0916)]
    pub actor_id: u32,  // Only available in 0.9.16
}
```

**Migration guide:**
- For 0.9.14/0.9.15: The `actor_id` field is not available. If you need to associate bounding boxes with actors, maintain your own mapping.
- For 0.9.16: You can directly access `bbox.actor_id` to identify which actor the bounding box belongs to.

### carla::traffic_manager::TrafficManager

#### Method: `set_keep_right_percentage()` (0.9.14/0.9.15 only)

**Removed in:** 0.9.16
**Availability:** `#[cfg(not(carla_0916))]`

```rust
// 0.9.14 and 0.9.15 only
impl TrafficManager {
    #[cfg(not(carla_0916))]
    pub fn set_keep_right_percentage<A: ActorBase>(&mut self, actor: &A, percentage: f32);
}
```

**Migration guide:**
- For 0.9.14/0.9.15: Use `set_keep_right_percentage()` to control vehicle lane-keeping behavior.
- For 0.9.16: This method was removed from the CARLA C++ API. Use other Traffic Manager methods to control vehicle behavior.

### carla::client::World

#### Method: `spawn_actor()` - socket_name parameter (0.9.16 only)

**Added in:** 0.9.16
**Availability:** Internal implementation detail, automatically handled

```rust
// Internal implementation - users don't see this difference
impl World {
    pub fn spawn_actor<T: Into<Transform>>(
        &mut self,
        blueprint: &ActorBlueprint,
        transform: T,
    ) -> Option<Actor> {
        #[cfg(carla_0916)]
        // Calls TrySpawnActor with socket_name parameter

        #[cfg(not(carla_0916))]
        // Calls TrySpawnActor without socket_name parameter
    }
}
```

**Migration guide:**
- No migration needed - this is handled internally by the bindings.

### carla::client::Vehicle (0.9.16 additions)

The following methods were added in CARLA 0.9.16 for advanced wheel and telemetry control:

#### Method: `get_telemetry_data()` (0.9.16 only)

**Added in:** 0.9.16
**Availability:** `#[cfg(carla_0916)]`

Returns detailed telemetry data for the vehicle including wheel states, suspension compression, and other physics information.

```rust
#[cfg(carla_0916)]
impl Vehicle {
    pub fn get_telemetry_data(&self) -> TelemetryData;
}
```

**Migration guide:**
- For 0.9.14/0.9.15: Telemetry data is not available. Use other sensor types or vehicle state queries.
- For 0.9.16: Access detailed vehicle physics through `get_telemetry_data()`.

#### Method: `set_wheel_pitch_angle()` (0.9.16 only)

**Added in:** 0.9.16
**Availability:** `#[cfg(carla_0916)]`

Sets the pitch angle (rotation) of a specific wheel. This affects the visual bone animation, not the physics simulation.

```rust
#[cfg(carla_0916)]
impl Vehicle {
    pub fn set_wheel_pitch_angle(&self, location: WheelLocation, angle_deg: f32);
}
```

#### Method: `get_wheel_pitch_angle()` (0.9.16 only)

**Added in:** 0.9.16
**Availability:** `#[cfg(carla_0916)]`

Returns the pitch angle of a specific wheel based on vehicle physics.

```rust
#[cfg(carla_0916)]
impl Vehicle {
    pub fn get_wheel_pitch_angle(&self, location: WheelLocation) -> f32;
}
```

#### Method: `get_vehicle_bone_world_transforms()` (0.9.16 only)

**Added in:** 0.9.16
**Availability:** `#[cfg(carla_0916)]`

Returns the world transforms of all vehicle bones (skeleton).

```rust
#[cfg(carla_0916)]
impl Vehicle {
    pub fn get_vehicle_bone_world_transforms(&self) -> Vec<Transform>;
}
```

#### Method: `restore_phys_x_physics()` (0.9.16 only)

**Added in:** 0.9.16
**Availability:** `#[cfg(carla_0916)]`

Restores PhysX physics simulation for the vehicle.

```rust
#[cfg(carla_0916)]
impl Vehicle {
    pub fn restore_phys_x_physics(&self);
}
```

**Migration guide for all 0.9.16 Vehicle methods:**
- For 0.9.14/0.9.15: These methods are not available. The API will not compile if you try to use them.
- For 0.9.16: Use these methods for advanced vehicle control and introspection.

### carla::client::Client

#### Method: `set_replayer_ignore_spectator()` (0.9.15+)

**Added in:** 0.9.15
**Availability:** Planned for `#[cfg(any(carla_0915, carla_0916))]` (not yet implemented)

Controls whether the spectator camera position is replayed during recording playback.

```rust
// Planned for 0.9.15+
impl Client {
    pub fn set_replayer_ignore_spectator(&self, ignore: bool);
}
```

### carla::client::WorldSettings

#### Field: `spectator_as_ego` (0.9.15+)

**Added in:** 0.9.15
**Availability:** Planned for `#[cfg(any(carla_0915, carla_0916))]` (not yet implemented)

Controls whether the spectator camera is used as the ego vehicle for tile loading in large maps.

## CARLA 0.10 behaviour that differs at run time

Measured against a 0.10.0 server (and, where marked, CARLA's own 0.10 Python client). Regression
tests: `carla/tests/ffi_layout_0100.rs`, which run on every version.

- **Geolocation sign.** CARLA's frame is left-handed (x east, y south). 0.10 maps +y to a lower
  latitude; LibCarla 0.9.x raises the latitude with +y, mirroring north-south (CARLA's 0.9.16
  Python client agrees). GNSS sensors follow the same convention as `Map::transform_to_geolocation`,
  so code that turns a fix into map coordinates flips latitude on 0.9.x only.
- **Exceptions are sliced.** 0.10's `carla::throw_exception(const std::exception&)` rethrows by
  value, so every LibCarla exception arrives as a bare `std::exception` whose `what()` is
  `"std::exception"`: no message, no type. carla-sys maps a bare exception raised once the full
  timeout has elapsed in `WaitForTick` to `Ok(None)` and in `Tick` to a timeout error; every other
  0.10 error reads "std::exception".
- **Wheel positions.** `WheelPhysicsControl::offset`, `location` and `old_location` read zero for
  every wheel (Python too); derive axle positions from the vehicle config instead.
- **Physics control layout.** `VehiclePhysicsControl` and `WheelPhysicsControl` have new layouts.
  autocxx parses 0.9.x-layout shims in `carla-sys/csrc_compat_0100/`; the C++ compile defines
  `CARLA_RUST_NATIVE_HEADERS` so that everything handed to libcarla_client (physics controls and
  `rpc::Command`, whose variant holds one) has the real layout. Before that, `physics_control()`
  returned defaults, `apply_physics_control()` failed with "container size overflow", and
  `rpc::Command` was 184 bytes against libcarla's 264.

## Version-Specific Features Summary

### CARLA 0.9.14
- Base API support
- TrafficManager includes `set_keep_right_percentage()`
- BoundingBox without `actor_id` field

### CARLA 0.9.15
- All 0.9.14 APIs
- TrafficManager still has `set_keep_right_percentage()`
- BoundingBox without `actor_id` field
- Client includes `set_replayer_ignore_spectator()` (planned)
- WorldSettings includes `spectator_as_ego` field (planned)
- Support for empty actors
- Improved pedestrian performance (10x faster with disabled collisions)

### CARLA 0.9.16
- Removed: TrafficManager `set_keep_right_percentage()`
- Added: BoundingBox `actor_id` field
- Added: Vehicle telemetry methods (`get_telemetry_data()`)
- Added: Vehicle wheel pitch control (`set/get_wheel_pitch_angle()`)
- Added: Vehicle bone transforms (`get_vehicle_bone_world_transforms()`)
- Added: Vehicle physics restoration (`restore_phys_x_physics()`)
- Modified: World spawn_actor with socket_name parameter (internal)

## Testing Across Versions

To test your code across all supported versions:

```bash
# Test with 0.9.14
CARLA_VERSION=0.9.14 cargo test

# Test with 0.9.15
CARLA_VERSION=0.9.15 cargo test

# Test with 0.9.16
CARLA_VERSION=0.9.16 cargo test
```

## Writing Version-Compatible Code

If you need your code to work across multiple CARLA versions, use conditional compilation:

```rust
use carla::client::Vehicle;

fn setup_vehicle(vehicle: &Vehicle) {
    // Common code for all versions
    vehicle.set_autopilot(true);

    #[cfg(carla_0916)]
    {
        // Only runs on 0.9.16
        let telemetry = vehicle.get_telemetry_data();
        println!("Telemetry: {:?}", telemetry);
    }

    #[cfg(not(carla_0916))]
    {
        // Runs on 0.9.14 and 0.9.15
        println!("Telemetry not available in this version");
    }
}
```

## References

- [CARLA 0.9.14 Documentation](https://carla.readthedocs.io/en/0.9.14/)
- [CARLA 0.9.15 Documentation](https://carla.readthedocs.io/en/0.9.15/)
- [CARLA 0.9.16 Documentation](https://carla.readthedocs.io/en/latest/)
- [CARLA CHANGELOG.md](https://github.com/carla-simulator/carla/blob/master/CHANGELOG.md)
