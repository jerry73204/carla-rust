#include "carla_c/actor.h"
#include "carla_c/client.h"
#include "carla_c/sensor.h"
#include "carla_c/world.h"
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Simple example: Basic sensor spawning and data collection
void example_basic_sensors(const char *host, uint16_t port) {
  printf("=== Basic Sensor Example ===\n");

  // Connect to CARLA server
  printf("Connecting to CARLA server at %s:%d...\n", host, port);
  carla_client_t *client = carla_client_new(host, port, 0);
  if (!client) {
    printf("Failed to connect to CARLA server at %s:%d\n", host, port);
    return;
  }

  // Set timeout
  carla_client_set_timeout(client, 10000);

  // Get world
  carla_world_t *world = carla_client_get_world(client);
  if (!world) {
    printf("Failed to get world\n");
    carla_client_free(client);
    return;
  }

  // Get blueprint library
  carla_blueprint_library_t *blueprints =
      carla_world_get_blueprint_library(world);
  if (!blueprints) {
    printf("Failed to get blueprint library\n");
    carla_world_free(world);
    carla_client_free(client);
    return;
  }

  // Spawn a vehicle
  carla_actor_blueprint_t *vehicle_bp =
      carla_blueprint_library_find(blueprints, "vehicle.tesla.model3");
  if (!vehicle_bp) {
    printf("Failed to find vehicle blueprint\n");
    goto cleanup;
  }

  // Get spawn points
  carla_map_t *map = carla_world_get_map(world);
  carla_transform_list_t *spawn_points = carla_map_get_spawn_points(map);
  if (!spawn_points || carla_transform_list_size(spawn_points) == 0) {
    printf("No spawn points available\n");
    goto cleanup;
  }

  carla_transform_t spawn_point = carla_transform_list_get(spawn_points, 0);

  carla_spawn_result_t vehicle_result =
      carla_world_try_spawn_actor(world, vehicle_bp, &spawn_point, NULL);
  if (vehicle_result.error != CARLA_ERROR_NONE) {
    printf("Failed to spawn vehicle: %s\n",
           carla_error_to_string(vehicle_result.error));
    goto cleanup;
  }
  carla_actor_t *vehicle = vehicle_result.actor;
  printf("Vehicle spawned successfully!\n");

  // Create sensor transforms relative to vehicle location
  carla_transform_t lidar_transform = {
      .location = {spawn_point.location.x, spawn_point.location.y,
                   spawn_point.location.z + 2.5f},
      .rotation = {0.0f, 0.0f, 0.0f}};

  carla_transform_t camera_transform = {
      .location = {spawn_point.location.x, spawn_point.location.y,
                   spawn_point.location.z + 2.0f},
      .rotation = {-15.0f, 0.0f, 0.0f}};

  // Spawn LiDAR sensor
  carla_actor_blueprint_t *lidar_bp =
      carla_blueprint_library_find(blueprints, "sensor.lidar.ray_cast");
  if (!lidar_bp) {
    printf("Failed to find LiDAR blueprint\n");
    goto cleanup;
  }

  carla_spawn_result_t lidar_result =
      carla_world_try_spawn_actor(world, lidar_bp, &lidar_transform, NULL);
  if (lidar_result.error != CARLA_ERROR_NONE) {
    printf("Failed to spawn LiDAR sensor: %s\n",
           carla_error_to_string(lidar_result.error));
    goto cleanup;
  }
  carla_actor_t *lidar = lidar_result.actor;
  printf("LiDAR sensor spawned successfully!\n");

  // Spawn RGB camera
  carla_actor_blueprint_t *camera_bp =
      carla_blueprint_library_find(blueprints, "sensor.camera.rgb");
  if (!camera_bp) {
    printf("Failed to find camera blueprint\n");
    goto cleanup;
  }

  carla_spawn_result_t camera_result =
      carla_world_try_spawn_actor(world, camera_bp, &camera_transform, NULL);
  if (camera_result.error != CARLA_ERROR_NONE) {
    printf("Failed to spawn camera sensor: %s\n",
           carla_error_to_string(camera_result.error));
    goto cleanup;
  }
  carla_actor_t *camera = camera_result.actor;
  printf("Camera sensor spawned successfully!\n");

  printf("Sensors active for 5 seconds...\n");
  sleep(5);

cleanup:
  // Clean up
  if (lidar)
    carla_actor_destroy(lidar);
  if (camera)
    carla_actor_destroy(camera);
  if (vehicle)
    carla_actor_destroy(vehicle);

  if (spawn_points)
    carla_transform_list_free(spawn_points);
  if (map)
    carla_map_free(map);
  carla_blueprint_library_free(blueprints);
  carla_world_free(world);
  carla_client_free(client);

  printf("Basic sensor example completed!\n");
}

// Example: Multi-sensor object tracking (placeholder)
void example_multi_sensor_tracking() {
  printf("\n=== Multi-Sensor Object Tracking Example ===\n");
  printf("This example would demonstrate:\n");
  printf("1. Fusing camera semantic segmentation with LiDAR clustering\n");
  printf("2. Tracking objects across multiple sensors\n");
  printf("3. Predicting future object positions\n");
  printf("Implementation not available yet.\n");
}

// Example: IMU-GNSS-Odometry fusion for localization (placeholder)
void example_localization_fusion() {
  printf("\n=== IMU-GNSS-Odometry Fusion Example ===\n");
  printf("This example would demonstrate:\n");
  printf("1. IMU data integration for motion estimation\n");
  printf("2. GNSS position updates for absolute localization\n");
  printf("3. Sensor fusion using Extended Kalman Filter\n");
  printf("Implementation not available yet.\n");
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
  printf("CARLA Basic Sensor Examples\n");
  printf("===========================\n\n");

  // Default values
  const char *host = "localhost";
  uint16_t port = 2000;

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
    case 'p': {
      int parsed_port = atoi(optarg);
      if (parsed_port <= 0 || parsed_port > 65535) {
        fprintf(stderr,
                "Error: Invalid port number '%s'. Port must be between 1 and "
                "65535.\n",
                optarg);
        return 1;
      }
      port = (uint16_t)parsed_port;
      break;
    }
    case 'H':
      print_usage(argv[0]);
      return 0;
    case 'V':
      printf("CARLA Basic Sensor Examples v0.10.0\n");
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

  // Run examples
  example_basic_sensors(host, port);
  example_multi_sensor_tracking();
  example_localization_fusion();

  return 0;
}
