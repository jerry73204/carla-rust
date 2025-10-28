# CARLA Rust Examples

This directory contains examples demonstrating how to use the CARLA Rust client library.

## Prerequisites

Before running any example, you must have:

1. **CARLA simulator running** on your system
   - Default connection: `localhost:2000`
   - Download CARLA from: https://github.com/carla-simulator/carla/releases
   - Start CARLA: `./CarlaUE4.sh` (Linux) or `CarlaUE4.exe` (Windows)

2. **Rust toolchain installed**
   - Install from: https://rustup.rs

## Running Examples

Run any example using:

```bash
cargo run --example <example_name>
```

For example:
```bash
cargo run --example connect
```

### World State Management

Most examples (except `connect.rs`) automatically load the **Town03** map when they start. This provides a consistent, clean world state for each example. This means you can:

- Run examples sequentially without restarting CARLA
- Have a predictable starting environment (Town03 map with no actors)
- Avoid conflicts from actors spawned by previous examples

**Note:** Loading a map clears all spawned actors from the previous session. The map loading takes a few seconds and you'll see progress messages in the console output.

## Available Examples

### Basic Examples

These examples cover fundamental operations:

- **connect.rs** - Connect to CARLA and get server version
  ```bash
  cargo run --example connect
  ```

- **world_info.rs** - Get world information (map name, spawn points, actors)
  ```bash
  cargo run --example world_info
  ```

- **blueprints.rs** - Query and filter the blueprint library
  ```bash
  cargo run --example blueprints
  ```

### Vehicle Examples

Learn how to work with vehicles:

- **spawn_vehicle.rs** - Spawn a single vehicle
  ```bash
  cargo run --example spawn_vehicle
  ```

- **multiple_vehicles.rs** - Spawn multiple vehicles at different locations
  ```bash
  cargo run --example multiple_vehicles
  ```

- **vehicle_transform.rs** - Get vehicle location, transform, and velocity
  ```bash
  cargo run --example vehicle_transform
  ```

- **vehicle_attributes.rs** - Query vehicle attributes
  ```bash
  cargo run --example vehicle_attributes
  ```

### Walker (Pedestrian) Examples

Learn how to work with pedestrians:

- **spawn_walker.rs** - Spawn a single walker/pedestrian
  ```bash
  cargo run --example spawn_walker
  ```

- **walker_control.rs** - Control walker movement
  ```bash
  cargo run --example walker_control
  ```

- **walker_directions.rs** - Test different movement directions
  ```bash
  cargo run --example walker_directions
  ```

- **multiple_walkers.rs** - Spawn multiple walkers
  ```bash
  cargo run --example multiple_walkers
  ```

## Common Issues & Troubleshooting

### "Failed to connect to CARLA simulator"

**Cause:** CARLA simulator is not running or not accessible.

**Solutions:**
- Ensure CARLA is running: `./CarlaUE4.sh` (Linux) or `CarlaUE4.exe` (Windows)
- Check CARLA is listening on port 2000 (default)
- If using a different port, modify the example's connection code:
  ```rust
  let client = Client::connect("localhost", YOUR_PORT, None)
  ```

### "Failed to spawn vehicle/walker"

**Cause:** Spawn point is occupied or invalid.

**Solutions:**
- Restart CARLA to clear all actors
- Try a different spawn point (modify the example to use a different index)
- Some spawn points may be invalid depending on the map

### Actors remain in simulation after example exits

This is expected behavior. The examples spawn actors but don't destroy them.

**Solutions:**
- **Run another example** - Most examples load Town03 at startup, which clears all actors
- **Restart CARLA** - Completely resets the simulator
- **Manual cleanup** - Connect and manually destroy actors (actor destruction API coming soon)

**Note:** Because examples automatically load a map at startup, spawned actors are automatically cleaned up when you run the next example.

### "No spawn points available"

**Cause:** The map doesn't have recommended spawn points.

**Solution:** Load a different map in CARLA. Most town maps have spawn points:
```python
# In CARLA Python API or via UE4 console
client.load_world('Town01')
```

## Learning Path

Recommended order for learning:

1. **Start with basics**: `connect.rs` → `world_info.rs` → `blueprints.rs`
2. **Learn vehicles**: `spawn_vehicle.rs` → `vehicle_transform.rs` → `multiple_vehicles.rs`
3. **Learn walkers**: `spawn_walker.rs` → `walker_control.rs` → `walker_directions.rs`

## Example Structure

Each example is self-contained and includes:
- Clear comments explaining what it does
- Minimal error handling (expect/unwrap with descriptive messages)
- Console output showing the results
- No external dependencies beyond the CARLA library

## Next Steps

After exploring these examples:

- Read the [API documentation](https://docs.rs/carla)
- Check out the main [README](../README.md) for advanced features
- Explore the [CARLA Python documentation](https://carla.readthedocs.io) for concepts that apply to all client libraries

## Contributing

Found an issue or want to add an example? Please open an issue or pull request on GitHub.
