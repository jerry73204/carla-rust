#include <carla_c/actor.h>
#include <carla_c/client.h>
#include <carla_c/map.h>
#include <carla_c/world.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

void print_available_maps(carla_client_t *client) {
  printf("Available maps:\n");
  carla_string_list_t *maps = carla_client_get_available_maps(client);
  if (maps) {
    size_t count = carla_string_list_size(maps);
    for (size_t i = 0; i < count; i++) {
      const char *map_name = carla_string_list_get(maps, i);
      printf("  - %s\n", map_name);
    }
    carla_free_string_list(maps);
  }
}

void print_spawn_points(carla_world_t *world) {
  carla_map_t *map = carla_world_get_map(world);
  if (!map) {
    printf("Failed to get map\n");
    return;
  }

  carla_transform_list_t *spawn_points = carla_map_get_spawn_points(map);
  if (spawn_points) {
    size_t count = carla_transform_list_size(spawn_points);
    printf("Found %zu spawn points:\n", count);

    for (size_t i = 0; i < count && i < 5; i++) { // Show first 5
      carla_transform_t transform = carla_transform_list_get(spawn_points, i);
      printf("  [%zu] Location: (%.2f, %.2f, %.2f)\n", i, transform.location.x,
             transform.location.y, transform.location.z);
    }

    if (count > 5) {
      printf("  ... and %zu more\n", count - 5);
    }

    carla_transform_list_free(spawn_points);
  }

  carla_map_free(map);
}

void spawn_vehicle_example(carla_world_t *world) {
  printf("\nSpawning a vehicle:\n");

  // Get blueprint library
  carla_blueprint_library_t *bp_lib = carla_world_get_blueprint_library(world);
  if (!bp_lib) {
    printf("Failed to get blueprint library\n");
    return;
  }

  // Find a vehicle blueprint
  carla_actor_blueprint_t *vehicle_bp =
      carla_blueprint_library_find(bp_lib, "vehicle.tesla.model3");
  if (!vehicle_bp) {
    printf("Tesla Model 3 not found, trying another vehicle...\n");
    // Try to find any vehicle
    size_t filter_count = 0;
    carla_actor_blueprint_t **vehicles =
        carla_blueprint_library_filter(bp_lib, "vehicle.*", &filter_count);
    if (vehicles && filter_count > 0) {
      vehicle_bp = vehicles[0];
      const char *bp_id = carla_actor_blueprint_get_id(vehicle_bp);
      printf("Using blueprint: %s\n", bp_id);

      // Free the filter results array (but keep the first blueprint)
      for (size_t i = 1; i < filter_count; i++) {
        // Note: Don't free individual blueprints as they're managed by the
        // library
      }
      free(vehicles);
    }
  }

  if (!vehicle_bp) {
    printf("No vehicle blueprints found\n");
    carla_blueprint_library_free(bp_lib);
    return;
  }

  // Get spawn points
  carla_map_t *map = carla_world_get_map(world);
  carla_transform_list_t *spawn_points = carla_map_get_spawn_points(map);

  if (spawn_points && carla_transform_list_size(spawn_points) > 0) {
    carla_transform_t spawn_transform =
        carla_transform_list_get(spawn_points, 0);

    // Try to spawn the vehicle
    carla_spawn_result_t result =
        carla_world_try_spawn_actor(world, vehicle_bp, &spawn_transform, NULL);

    if (result.error == CARLA_ERROR_NONE && result.actor) {
      uint32_t actor_id = carla_actor_get_id(result.actor);
      const char *type_id = carla_actor_get_type_id(result.actor);
      printf("Successfully spawned vehicle with ID %u (type: %s)\n", actor_id,
             type_id);

      // Get vehicle location
      carla_vector3d_t location = carla_actor_get_location(result.actor);
      printf("Vehicle location: (%.2f, %.2f, %.2f)\n", location.x, location.y,
             location.z);

      // Clean up after a few seconds
      printf("Vehicle will be destroyed in 3 seconds...\n");
      sleep(3);

      carla_actor_destroy(result.actor);
      printf("Vehicle destroyed\n");
    } else {
      printf("Failed to spawn vehicle: %s\n",
             carla_error_to_string(result.error));
    }

    carla_transform_list_free(spawn_points);
  }

  carla_map_free(map);
  carla_blueprint_library_free(bp_lib);
}

void print_usage(const char *program_name) {
  printf("Usage: %s [OPTIONS]\n", program_name);
  printf("\nOptions:\n");
  printf("  -h, --host HOST      CARLA server host (default: localhost)\n");
  printf("  -p, --port PORT      CARLA server port (default: 2000)\n");
  printf("      --help           Show this help message\n");
  printf("      --version        Show version information\n");
  printf("\nExamples:\n");
  printf("  %s                           # Connect to localhost:2000\n",
         program_name);
  printf("  %s -h 192.168.1.100         # Connect to remote host\n",
         program_name);
  printf("  %s --host 192.168.1.100 --port 2001  # Full specification\n",
         program_name);
}

int main(int argc, char *argv[]) {
  printf("LibCarla C Wrapper Example\n");
  printf("==========================\n\n");

  // Default values
  const char *host = "localhost";
  int port = 2000;

  // Define long options
  static struct option long_options[] = {{"host", required_argument, 0, 'h'},
                                         {"port", required_argument, 0, 'p'},
                                         {"help", no_argument, 0, 'H'},
                                         {"version", no_argument, 0, 'V'},
                                         {0, 0, 0, 0}};

  // Parse command line arguments
  int option_index = 0;
  int c;

  while ((c = getopt_long(argc, argv, "h:p:", long_options, &option_index)) !=
         -1) {
    switch (c) {
    case 'h':
      host = optarg;
      break;
    case 'p':
      port = atoi(optarg);
      if (port <= 0 || port > 65535) {
        fprintf(stderr,
                "Error: Invalid port number '%s'. Port must be between 1 and "
                "65535.\n",
                optarg);
        return 1;
      }
      break;
    case 'H':
      print_usage(argv[0]);
      return 0;
    case 'V':
      printf("LibCarla C Wrapper Example v0.10.0\n");
      printf("Compatible with CARLA 0.10.0\n");
      return 0;
    case '?':
      // getopt_long already printed an error message
      fprintf(stderr, "Try '%s --help' for more information.\n", argv[0]);
      return 1;
    default:
      fprintf(stderr, "Error: Unknown option.\n");
      print_usage(argv[0]);
      return 1;
    }
  }

  // Check for unexpected positional arguments
  if (optind < argc) {
    fprintf(stderr, "Error: Unexpected arguments:");
    while (optind < argc) {
      fprintf(stderr, " %s", argv[optind++]);
    }
    fprintf(stderr, "\n");
    fprintf(stderr, "Try '%s --help' for more information.\n", argv[0]);
    return 1;
  }

  // Connect to CARLA server
  printf("Connecting to CARLA server at %s:%d...\n", host, port);
  carla_client_t *client = carla_client_new(host, (uint16_t)port, 0);
  if (!client) {
    printf("Failed to connect to CARLA server\n");
    printf("Make sure CARLA simulator is running on %s:%d\n", host, port);
    return 1;
  }

  // Set timeout
  carla_client_set_timeout(client, 10000); // 10 seconds

  // Get version info
  char *client_version = carla_client_get_client_version(client);
  char *server_version = carla_client_get_server_version(client);
  printf("Client version: %s\n", client_version);
  printf("Server version: %s\n", server_version);
  carla_free_string(client_version);
  carla_free_string(server_version);

  // List available maps
  print_available_maps(client);

  // Get current world
  printf("\nGetting current world...\n");
  carla_world_t *world = carla_client_get_world(client);
  if (!world) {
    printf("Failed to get world\n");
    carla_client_free(client);
    return 1;
  }

  uint64_t world_id = carla_world_get_id(world);
  printf("World ID: %lu\n", world_id);

  // Print spawn points
  print_spawn_points(world);

  // Spawn vehicle example
  spawn_vehicle_example(world);

  // Weather example
  printf("\nWeather example:\n");
  carla_weather_parameters_t weather = carla_world_get_weather(world);
  printf("Current weather - Cloudiness: %.2f, Precipitation: %.2f\n",
         weather.cloudiness, weather.precipitation);

  // Set sunny weather
  weather.cloudiness = 0.0f;
  weather.precipitation = 0.0f;
  weather.sun_altitude_angle = 45.0f;
  carla_world_set_weather(world, &weather);
  printf("Set sunny weather\n");

  // Clean up
  carla_world_free(world);
  carla_client_free(client);

  printf("\nExample completed successfully!\n");
  return 0;
}
