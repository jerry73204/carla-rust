#include "carla_c/actor.h"
#include "carla_c/client.h"
#include "carla_c/lidar.h"
#include "carla_c/sensor.h"
#include "carla_c/world.h"
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Global variables for demonstration
static int lidar_data_received = 0;
static carla_lidar_detection_t *filtered_points = NULL;
static size_t filtered_count = 0;

// LiDAR sensor callback function
void lidar_callback(carla_sensor_data_t *data, void *user_data) {
  printf("LiDAR data received!\n");

  // Get basic sensor data info
  carla_sensor_data_info_t info = carla_sensor_data_get_info(data);
  printf("Frame: %lu, Timestamp: %.3f\n", info.frame, info.timestamp);

  // Get LiDAR-specific data
  carla_lidar_data_t lidar_data = carla_sensor_data_get_lidar(data);
  printf("Point count: %zu, Channels: %u, Horizontal angle: %u\n",
         lidar_data.point_count, lidar_data.channels,
         lidar_data.horizontal_angle);

  // Calculate statistics
  carla_lidar_stats_t stats;
  carla_error_t error = carla_lidar_calculate_stats(data, &stats);
  if (error == CARLA_ERROR_NONE) {
    printf("Statistics:\n");
    printf("  Total points: %zu\n", stats.total_points);
    printf("  Intensity range: %.3f - %.3f (avg: %.3f)\n", stats.min_intensity,
           stats.max_intensity, stats.avg_intensity);
    printf("  Distance range: %.3f - %.3f (avg: %.3f)\n", stats.min_distance,
           stats.max_distance, stats.avg_distance);
    printf("  Bounds: (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)\n",
           stats.bounds.min_point.x, stats.bounds.min_point.y,
           stats.bounds.min_point.z, stats.bounds.max_point.x,
           stats.bounds.max_point.y, stats.bounds.max_point.z);
  }

  // Filter points by distance (remove points closer than 2m and farther than
  // 50m)
  if (filtered_points) {
    free(filtered_points);
  }

  size_t max_points = lidar_data.point_count;
  filtered_points = malloc(max_points * sizeof(carla_lidar_detection_t));

  if (filtered_points) {
    error = carla_lidar_filter_by_distance(data, 2.0f, 50.0f, filtered_points,
                                           max_points, &filtered_count);
    if (error == CARLA_ERROR_NONE) {
      printf("Filtered points (2m-50m range): %zu / %zu (%.1f%%)\n",
             filtered_count, lidar_data.point_count,
             (float)filtered_count / lidar_data.point_count * 100.0f);
    }
  }

  // Save point cloud to file (every 10th frame to avoid too many files)
  if (info.frame % 10 == 0) {
    char filename[256];
    snprintf(filename, sizeof(filename), "lidar_frame_%06lu.ply", info.frame);

    error = carla_lidar_save_to_file(data, filename, CARLA_LIDAR_FORMAT_PLY);
    if (error == CARLA_ERROR_NONE) {
      printf("Saved point cloud to %s\n", filename);
    }

    // Also save filtered points
    if (filtered_points && filtered_count > 0) {
      snprintf(filename, sizeof(filename), "lidar_filtered_%06lu.ply",
               info.frame);
      error = carla_lidar_save_points_to_file(filtered_points, filtered_count,
                                              filename, CARLA_LIDAR_FORMAT_PLY);
      if (error == CARLA_ERROR_NONE) {
        printf("Saved filtered point cloud to %s\n", filename);
      }
    }
  }

  lidar_data_received++;

  // Clean up sensor data
  carla_sensor_data_destroy(data);
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
  printf("CARLA LiDAR Processing Example\n");
  printf("=============================\n\n");

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
      printf("CARLA LiDAR Processing Example v0.10.0\n");
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

  printf("Connecting to CARLA server at %s:%d...\n", host, port);

  // Connect to CARLA
  carla_client_t *client = carla_client_new(host, port, 0);
  if (!client) {
    fprintf(stderr, "Failed to create CARLA client\n");
    return 1;
  }
  
  // Set timeout
  carla_client_set_timeout(client, 10000);

  carla_world_t *world = carla_client_get_world(client);
  if (!world) {
    fprintf(stderr, "Failed to get world\n");
    carla_client_free(client);
    return 1;
  }

  printf("Connected successfully!\n");

  // Get blueprint library
  carla_blueprint_library_t *bp_lib = carla_world_get_blueprint_library(world);
  if (!bp_lib) {
    fprintf(stderr, "Failed to get blueprint library\n");
    carla_world_free(world);
    carla_client_free(client);
    return 1;
  }

  // Find LiDAR sensor blueprint
  carla_actor_blueprint_t *lidar_bp =
      carla_blueprint_library_find(bp_lib, "sensor.lidar.ray_cast");
  if (!lidar_bp) {
    fprintf(stderr, "Failed to find LiDAR sensor blueprint\n");
    carla_blueprint_library_free(bp_lib);
    carla_world_free(world);
    carla_client_free(client);
    return 1;
  }

  printf("Found LiDAR sensor blueprint\n");

  // Set LiDAR attributes for better point cloud
  // Note: In a real implementation, you'd want to check if setting attributes
  // succeeded
  printf("Configuring LiDAR sensor...\n");

  // Spawn LiDAR sensor at a fixed location
  carla_transform_t sensor_transform = {
      .location = {0.0f, 0.0f, 2.0f}, // 2 meters above ground
      .rotation = {0.0f, 0.0f, 0.0f}  // No rotation
  };

  carla_spawn_result_t result =
      carla_world_try_spawn_actor(world, lidar_bp, &sensor_transform, NULL);
  if (result.error != CARLA_ERROR_NONE || !result.actor) {
    fprintf(stderr, "Failed to spawn LiDAR sensor: %s\n", carla_error_to_string(result.error));
    carla_blueprint_library_free(bp_lib);
    carla_world_free(world);
    carla_client_free(client);
    return 1;
  }

  printf("LiDAR sensor spawned successfully\n");

  // Convert actor to sensor
  carla_sensor_t *lidar_sensor = result.actor; // Sensors are actors
  if (!lidar_sensor) {
    fprintf(stderr, "Failed to get sensor\n");
    carla_actor_destroy(result.actor);
    carla_blueprint_library_free(bp_lib);
    carla_world_free(world);
    carla_client_free(client);
    return 1;
  }

  // Start listening for sensor data
  carla_error_t error = carla_sensor_listen(lidar_sensor, lidar_callback, NULL);
  if (error != CARLA_ERROR_NONE) {
    fprintf(stderr, "Failed to start sensor listening: %s\n", carla_error_to_string(error));
    carla_actor_destroy(result.actor);
    carla_blueprint_library_free(bp_lib);
    carla_world_free(world);
    carla_client_free(client);
    return 1;
  }

  printf("Listening for LiDAR data... (will process 50 frames)\n");
  printf("Check the console output for processing results.\n");
  printf("Point cloud files will be saved as *.ply files.\n\n");

  // Process LiDAR data for a while
  int max_frames = 50;
  while (lidar_data_received < max_frames) {
    // Tick the world to advance simulation
    carla_world_tick(world, 1000); // 1 second timeout

    // Sleep a bit to avoid overwhelming the system
    usleep(100000); // 100ms

    if (lidar_data_received % 10 == 0 && lidar_data_received > 0) {
      printf("Processed %d frames...\n", lidar_data_received);
    }
  }

  printf("\nProcessing complete! Received %d LiDAR frames.\n",
         lidar_data_received);

  // Demonstrate LiDAR filter initialization
  printf("\nDemonstrating LiDAR filter configuration:\n");
  carla_lidar_filter_t filter;
  carla_lidar_filter_init(&filter);

  // Configure filter for indoor/close-range scanning
  filter.use_distance_filter = true;
  filter.min_distance = 0.5f;  // 50cm minimum
  filter.max_distance = 20.0f; // 20m maximum
  filter.use_intensity_filter = true;
  filter.min_intensity = 0.1f; // Filter out very low intensity points
  filter.max_intensity = 1.0f;
  filter.use_height_filter = true;
  filter.min_height = -2.0f; // 2m below sensor
  filter.max_height = 5.0f;  // 5m above sensor

  printf("Filter configured:\n");
  printf("  Distance: %.1fm - %.1fm\n", filter.min_distance,
         filter.max_distance);
  printf("  Intensity: %.2f - %.2f\n", filter.min_intensity,
         filter.max_intensity);
  printf("  Height: %.1fm - %.1fm\n", filter.min_height, filter.max_height);

  // Clean up
  printf("\nCleaning up...\n");
  if (lidar_sensor) {
    carla_actor_destroy(result.actor);
  }
  carla_blueprint_library_free(bp_lib);
  carla_world_free(world);
  carla_client_free(client);

  if (filtered_points) {
    free(filtered_points);
  }

  printf("Example completed successfully!\n");
  return 0;
}
