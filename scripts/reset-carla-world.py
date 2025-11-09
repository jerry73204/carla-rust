#!/usr/bin/env python3
"""
Reset CARLA world state between example runs.

This script connects to a running CARLA server and resets the world to a clean state
by destroying all actors. If the requested map differs from the current map, it will
reload the map (default: Town01). Map reloading is skipped when already on the target
map to avoid crashes. Used by run-examples.sh to clean up between examples.

Modes:
  --verify: Only verify CARLA is ready (connect, get world, tick)
  default: Full reset (destroy actors, conditionally reload map)

Options:
  --map MAP: Specify which map to load (default: Town01)
             Map reload is skipped if already on this map
  --no-rendering: Enable no-rendering mode (disables all graphics)
                  Note: Cameras and GPU sensors will return empty data

Exit codes:
  0 - Success: World reset/verified successfully
  1 - Connection failed: CARLA server is not responding (needs restart)
  2 - Reset failed: Connected but reset operation failed
"""

import argparse
import sys
import time


def reset_world(host: str, port: int, timeout: float = 10.0, map_name: str = "Town01", no_rendering: bool = False) -> int:
    """
    Reset CARLA world to clean state.

    Args:
        host: CARLA server hostname
        port: CARLA server port
        timeout: Connection timeout in seconds
        map_name: Map to load (default: Town01). Only reloads if different from current map.
        no_rendering: Enable no-rendering mode (disables all graphics, cameras return empty data)

    Returns:
        Exit code (0=success, 1=connection failed, 2=reset failed)
    """
    try:
        # Import CARLA Python API
        # Note: This assumes CARLA Python package is in PYTHONPATH
        import carla
    except ImportError:
        print("ERROR: Could not import carla Python module", file=sys.stderr)
        print("Make sure CARLA Python API is in PYTHONPATH", file=sys.stderr)
        return 2

    try:
        # Connect to CARLA server
        print(f"Connecting to CARLA at {host}:{port}...", file=sys.stderr)
        client = carla.Client(host, port)
        client.set_timeout(timeout)

        # Test connection by getting server version
        version = client.get_server_version()
        print(f"Connected to CARLA {version}", file=sys.stderr)

        # Get world
        world = client.get_world()

        # Apply no-rendering mode if requested
        if no_rendering:
            print("Enabling no-rendering mode...", file=sys.stderr)
            settings = world.get_settings()
            settings.no_rendering_mode = True
            world.apply_settings(settings)
            print("No-rendering mode enabled", file=sys.stderr)

        # Destroy all actors (except sensor.other.* and controller.ai.*)
        print("Destroying actors...", file=sys.stderr)
        actor_list = world.get_actors()
        destroyed_count = 0

        for actor in actor_list:
            # Skip infrastructure actors
            type_id = actor.type_id
            if type_id.startswith("sensor.other.") or type_id.startswith("controller.ai."):
                continue

            # Destroy actor
            actor.destroy()
            destroyed_count += 1

        print(f"Destroyed {destroyed_count} actors", file=sys.stderr)

        # Only reload map if it's different from current map
        current_map = world.get_map()
        current_map_name = current_map.name.split('/')[-1]  # Extract map name (e.g., "Town03" from "/Game/Carla/Maps/Town03")

        if current_map_name != map_name:
            print(f"Reloading from {current_map_name} to {map_name}...", file=sys.stderr)
            try:
                client.load_world(map_name)

                # Wait for world to be ready after reload
                world = client.get_world()
                world.tick()

                print(f"World reset complete (reloaded to {map_name})", file=sys.stderr)
            except RuntimeError as e:
                print(f"WARNING: Map reload failed: {e}", file=sys.stderr)
                print(f"Continuing with current map ({current_map_name})", file=sys.stderr)
        else:
            # Same map, just tick to apply actor destruction
            print(f"Already on {map_name}, skipping reload", file=sys.stderr)
            world.tick()
            print(f"World reset complete (actors destroyed, same map)", file=sys.stderr)
        return 0

    except RuntimeError as e:
        # Connection or timeout errors
        error_msg = str(e)
        if "timeout" in error_msg.lower() or "connection" in error_msg.lower():
            print(f"ERROR: Connection failed: {e}", file=sys.stderr)
            return 1
        else:
            print(f"ERROR: Reset operation failed: {e}", file=sys.stderr)
            return 2

    except Exception as e:
        print(f"ERROR: Unexpected error: {e}", file=sys.stderr)
        return 2


def verify_world(host: str, port: int, timeout: float = 10.0) -> int:
    """
    Verify CARLA is ready by connecting and performing basic operations.

    Args:
        host: CARLA server hostname
        port: CARLA server port
        timeout: Connection timeout in seconds

    Returns:
        Exit code (0=success, 1=connection failed, 2=verification failed)
    """
    try:
        # Import CARLA Python API
        import carla
    except ImportError:
        print("ERROR: Could not import carla Python module", file=sys.stderr)
        print("Make sure CARLA Python API is in PYTHONPATH", file=sys.stderr)
        return 2

    try:
        # Connect to CARLA server
        print(f"Verifying CARLA at {host}:{port}...", file=sys.stderr)
        client = carla.Client(host, port)
        client.set_timeout(timeout)

        # Test connection by getting server version
        version = client.get_server_version()
        print(f"Connected to CARLA {version}", file=sys.stderr)

        # Get world and perform a tick to ensure rendering is ready
        world = client.get_world()

        # Get world settings to verify full initialization
        settings = world.get_settings()
        print(f"World synchronous mode: {settings.synchronous_mode}", file=sys.stderr)

        # Try to tick the world (this will fail if rendering not ready)
        world.tick()
        print("World tick successful", file=sys.stderr)

        # Try to get map (this ensures map data is loaded)
        world_map = world.get_map()
        print(f"Map loaded: {world_map.name}", file=sys.stderr)

        print("CARLA verification successful", file=sys.stderr)
        return 0

    except RuntimeError as e:
        # Connection or timeout errors
        error_msg = str(e)
        if "timeout" in error_msg.lower() or "connection" in error_msg.lower():
            print(f"ERROR: Connection failed: {e}", file=sys.stderr)
            return 1
        else:
            print(f"ERROR: Verification failed: {e}", file=sys.stderr)
            return 2

    except Exception as e:
        print(f"ERROR: Unexpected error: {e}", file=sys.stderr)
        return 2


def main():
    parser = argparse.ArgumentParser(
        description="Reset CARLA world state between example runs"
    )
    parser.add_argument(
        "--host",
        default="localhost",
        help="CARLA server hostname (default: localhost)",
    )
    parser.add_argument(
        "--port", type=int, default=2000, help="CARLA server port (default: 2000)"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Connection timeout in seconds (default: 10.0)",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Only verify CARLA is ready (don't reset)",
    )
    parser.add_argument(
        "--map",
        default="Town01",
        help="Map to load when resetting (default: Town01)",
    )
    parser.add_argument(
        "--no-rendering",
        action="store_true",
        help="Enable no-rendering mode (disables graphics, cameras return empty data)",
    )

    args = parser.parse_args()

    if args.verify:
        exit_code = verify_world(args.host, args.port, args.timeout)
    else:
        exit_code = reset_world(args.host, args.port, args.timeout, args.map, args.no_rendering)

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
