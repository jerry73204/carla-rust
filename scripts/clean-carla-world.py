#!/usr/bin/env python3
"""
Clean CARLA world state without reloading the map.

This script performs a comprehensive cleanup of the CARLA world:
- Destroys all spawned actors (vehicles, walkers, sensors, etc.)
- Resets weather to default (clear day)
- Resets time of day to noon
- Resets all traffic lights
- Re-enables all environment objects
- Does NOT reload the map (keeps current map)

This is faster than reloading the map and useful for quick resets during development.

Exit codes:
  0 - Success: World cleaned successfully
  1 - Connection failed: CARLA server is not responding
  2 - Cleanup failed: Connected but cleanup operation failed
"""

import argparse
import sys
import time


def clean_world(host: str, port: int, timeout: float = 10.0, verbose: bool = False) -> int:
    """
    Clean CARLA world without reloading map.

    Args:
        host: CARLA server hostname
        port: CARLA server port
        timeout: Connection timeout in seconds
        verbose: Print detailed progress messages

    Returns:
        Exit code (0=success, 1=connection failed, 2=cleanup failed)
    """
    try:
        # Import CARLA Python API
        import carla
    except ImportError:
        print("ERROR: Could not import carla Python module", file=sys.stderr)
        print("Make sure CARLA Python API is in PYTHONPATH", file=sys.stderr)
        return 2

    def log(msg):
        if verbose:
            print(msg, file=sys.stderr)

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
        current_map = world.get_map()
        map_name = current_map.name.split('/')[-1]
        log(f"Current map: {map_name}")

        # 1. Destroy all actors
        print("Destroying actors...", file=sys.stderr)
        actor_list = world.get_actors()
        destroyed_count = 0
        destroyed_by_type = {}

        for actor in actor_list:
            type_id = actor.type_id

            # Skip infrastructure actors (spectator, weather, etc.)
            if type_id.startswith("sensor.other.") or type_id.startswith("controller.ai."):
                continue

            # Track types for reporting
            actor_type = type_id.split('.')[0] if '.' in type_id else 'unknown'
            destroyed_by_type[actor_type] = destroyed_by_type.get(actor_type, 0) + 1

            # Destroy actor
            actor.destroy()
            destroyed_count += 1

        print(f"Destroyed {destroyed_count} actors", file=sys.stderr)
        if verbose and destroyed_by_type:
            for actor_type, count in sorted(destroyed_by_type.items()):
                log(f"  - {actor_type}: {count}")

        # Tick to apply actor destruction
        world.tick()

        # 2. Reset weather to clear day
        print("Resetting weather...", file=sys.stderr)
        weather = carla.WeatherParameters.ClearNoon
        world.set_weather(weather)
        log(f"  Weather set to: ClearNoon")
        log(f"    Cloudiness: {weather.cloudiness}%")
        log(f"    Precipitation: {weather.precipitation}%")
        log(f"    Sun altitude: {weather.sun_altitude_angle}°")

        # 3. Reset all traffic lights
        print("Resetting traffic lights...", file=sys.stderr)
        traffic_lights = world.get_actors().filter('traffic.traffic_light*')
        if len(traffic_lights) > 0:
            world.reset_all_traffic_lights()
            log(f"  Reset {len(traffic_lights)} traffic lights")
        else:
            log("  No traffic lights found")

        # 4. Re-enable all environment objects (in case some were disabled)
        print("Re-enabling environment objects...", file=sys.stderr)
        env_objs = world.get_environment_objects(carla.CityObjectLabel.Any)
        if len(env_objs) > 0:
            # Get all object IDs
            obj_ids = [obj.id for obj in env_objs]
            # Enable all objects
            world.enable_environment_objects(obj_ids, True)
            log(f"  Ensured {len(env_objs)} environment objects are enabled")
        else:
            log("  No environment objects found")

        # 5. Reset world settings to default
        print("Resetting world settings...", file=sys.stderr)
        settings = world.get_settings()

        # Store original rendering mode
        original_no_rendering = settings.no_rendering_mode

        # Reset to sensible defaults
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        # Keep no_rendering_mode as it was

        world.apply_settings(settings)
        log(f"  Synchronous mode: {settings.synchronous_mode}")
        log(f"  No rendering mode: {settings.no_rendering_mode}")

        # Final tick to ensure everything is applied
        world.tick()

        print(f"World cleanup complete (map: {map_name})", file=sys.stderr)
        return 0

    except RuntimeError as e:
        # Connection or timeout errors
        error_msg = str(e)
        if "timeout" in error_msg.lower() or "connection" in error_msg.lower():
            print(f"ERROR: Connection failed: {e}", file=sys.stderr)
            return 1
        else:
            print(f"ERROR: Cleanup operation failed: {e}", file=sys.stderr)
            return 2

    except Exception as e:
        print(f"ERROR: Unexpected error: {e}", file=sys.stderr)
        return 2


def main():
    parser = argparse.ArgumentParser(
        description="Clean CARLA world state without reloading map",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                    # Clean world with defaults
  %(prog)s --verbose          # Show detailed progress
  %(prog)s --port 2001        # Clean world on custom port

This script performs comprehensive cleanup:
  - Destroys all actors (vehicles, walkers, sensors, etc.)
  - Resets weather to ClearNoon
  - Resets all traffic lights
  - Re-enables all environment objects
  - Resets world settings to defaults
  - Does NOT reload the map (keeps current map)
        """
    )
    parser.add_argument(
        "--host",
        default="localhost",
        help="CARLA server hostname (default: localhost)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=2000,
        help="CARLA server port (default: 2000)"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Connection timeout in seconds (default: 10.0)",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Show detailed progress messages",
    )

    args = parser.parse_args()

    exit_code = clean_world(args.host, args.port, args.timeout, args.verbose)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
