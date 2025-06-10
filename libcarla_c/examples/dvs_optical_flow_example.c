#include "carla_c/camera.h"
#include "carla_c/client.h"
#include "carla_c/map.h"
#include "carla_c/sensor.h"
#include "carla_c/world.h"
#include "carla_c/actor.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// Callback function for DVS sensor data
void dvs_callback(carla_sensor_data_t *data, void *user_data) {
  printf("DVS callback triggered!\n");
  
  carla_sensor_data_type_t data_type = carla_sensor_data_get_type(data);
  if (data_type == CARLA_SENSOR_DATA_DVS_EVENT_ARRAY) {
    size_t event_count;
    const carla_dvs_event_t *events = carla_dvs_get_events(data, &event_count);
    
    printf("DVS Events: %zu total events\n", event_count);
    
    if (events && event_count > 0) {
      // Analyze events
      carla_dvs_analysis_t analysis;
      carla_error_t error = carla_dvs_analyze_events(data, &analysis);
      
      if (error == CARLA_ERROR_NONE) {
        printf("  Positive events: %u\n", analysis.positive_events);
        printf("  Negative events: %u\n", analysis.negative_events);
        printf("  Event rate: %.2f\n", analysis.event_rate);
        printf("  Activity density: %.4f\n", analysis.activity_density);
      }
      
      // Convert events to image
      uint8_t *image_data;
      size_t image_size;
      float time_window = 0.1f; // 100ms time window
      error = carla_dvs_events_to_image(data, time_window, &image_data, &image_size);
      
      if (error == CARLA_ERROR_NONE && image_data) {
        printf("  Converted to image: %zu bytes\n", image_size);
        carla_image_free_data(image_data);
      }
      
      // Filter events by polarity
      carla_dvs_event_t *positive_events;
      size_t positive_count;
      error = carla_dvs_filter_events_by_polarity(data, true, &positive_events, &positive_count);
      
      if (error == CARLA_ERROR_NONE && positive_events) {
        printf("  Filtered positive events: %zu\n", positive_count);
        carla_dvs_free_events(positive_events);
      }
    }
  }
}

// Callback function for optical flow sensor data
void optical_flow_callback(carla_sensor_data_t *data, void *user_data) {
  printf("Optical flow callback triggered!\n");
  
  carla_sensor_data_type_t data_type = carla_sensor_data_get_type(data);
  if (data_type == CARLA_SENSOR_DATA_OPTICAL_FLOW_IMAGE) {
    carla_optical_flow_data_t flow_data = carla_sensor_data_get_optical_flow(data);
    
    printf("Optical Flow: %ux%u pixels\n", flow_data.width, flow_data.height);
    
    if (flow_data.flow_data) {
      // Analyze optical flow
      carla_optical_flow_analysis_t analysis;
      carla_error_t error = carla_image_analyze_optical_flow(data, &analysis);
      
      if (error == CARLA_ERROR_NONE) {
        printf("  Average flow: (%.3f, %.3f)\n", 
               analysis.average_flow.x, analysis.average_flow.y);
        printf("  Motion magnitude avg: %.3f, max: %.3f\n", 
               analysis.magnitude_avg, analysis.magnitude_max);
        printf("  Moving pixels: %u (%.2f%%)\n", 
               analysis.moving_pixels, analysis.motion_density * 100.0f);
        printf("  Dominant direction: (%.3f, %.3f)\n",
               analysis.dominant_direction.x, analysis.dominant_direction.y);
      }
      
      // Get magnitude data
      float *magnitude_data;
      size_t data_size;
      error = carla_image_optical_flow_magnitude(data, &magnitude_data, &data_size);
      
      if (error == CARLA_ERROR_NONE && magnitude_data) {
        printf("  Magnitude data: %zu pixels\n", data_size);
        carla_optical_flow_free_data(magnitude_data);
      }
      
      // Get direction data
      float *direction_data;
      error = carla_image_optical_flow_direction(data, &direction_data, &data_size);
      
      if (error == CARLA_ERROR_NONE && direction_data) {
        printf("  Direction data: %zu pixels\n", data_size);
        carla_optical_flow_free_data(direction_data);
      }
    }
  }
}

int main(int argc, char *argv[]) {
  const char *host = "localhost";
  uint16_t port = 2000;
  
  printf("=== DVS and Optical Flow Example ===\n");
  
  // Connect to CARLA server
  printf("Connecting to CARLA server at %s:%d...\n", host, port);
  carla_client_t *client = carla_client_new(host, port, 0);
  if (!client) {
    printf("Failed to connect to CARLA server\n");
    return 1;
  }
  
  carla_client_set_timeout(client, 10000);
  
  // Get world
  carla_world_t *world = carla_client_get_world(client);
  if (!world) {
    printf("Failed to get world\n");
    carla_client_free(client);
    return 1;
  }
  
  // Get blueprint library
  carla_blueprint_library_t *blueprints = carla_world_get_blueprint_library(world);
  if (!blueprints) {
    printf("Failed to get blueprint library\n");
    carla_world_free(world);
    carla_client_free(client);
    return 1;
  }
  
  // Get spawn points
  carla_map_t *map = carla_world_get_map(world);
  carla_transform_list_t *spawn_points = carla_map_get_spawn_points(map);
  if (!spawn_points || carla_transform_list_size(spawn_points) == 0) {
    printf("No spawn points available\n");
    goto cleanup;
  }
  
  carla_transform_t spawn_point = carla_transform_list_get(spawn_points, 0);
  
  // Adjust spawn point for camera mounting
  spawn_point.location.z += 2.0f; // Mount camera higher
  
  printf("Spawn point: (%.2f, %.2f, %.2f)\n", 
         spawn_point.location.x, spawn_point.location.y, spawn_point.location.z);
  
  // Create DVS sensor
  carla_actor_blueprint_t *dvs_bp = carla_blueprint_library_find(blueprints, "sensor.camera.dvs");
  if (dvs_bp) {
    printf("Creating DVS sensor...\n");
    carla_spawn_result_t dvs_result = carla_world_spawn_actor(world, dvs_bp, &spawn_point, NULL);
    
    if (dvs_result.error == CARLA_ERROR_NONE && dvs_result.actor) {
      printf("DVS sensor spawned successfully\n");
      
      // Set up DVS callback
      carla_sensor_t *dvs_sensor = carla_actor_as_sensor(dvs_result.actor);
      if (dvs_sensor) {
        carla_sensor_listen(dvs_sensor, dvs_callback, NULL);
        printf("DVS sensor listening...\n");
      }
    } else {
      printf("Failed to spawn DVS sensor\n");
    }
  } else {
    printf("DVS sensor blueprint not found\n");
  }
  
  // Create optical flow sensor
  spawn_point.location.y += 1.0f; // Offset slightly
  carla_actor_blueprint_t *flow_bp = carla_blueprint_library_find(blueprints, "sensor.camera.optical_flow");
  if (flow_bp) {
    printf("Creating optical flow sensor...\n");
    carla_spawn_result_t flow_result = carla_world_spawn_actor(world, flow_bp, &spawn_point, NULL);
    
    if (flow_result.error == CARLA_ERROR_NONE && flow_result.actor) {
      printf("Optical flow sensor spawned successfully\n");
      
      // Set up optical flow callback
      carla_sensor_t *flow_sensor = carla_actor_as_sensor(flow_result.actor);
      if (flow_sensor) {
        carla_sensor_listen(flow_sensor, optical_flow_callback, NULL);
        printf("Optical flow sensor listening...\n");
      }
    } else {
      printf("Failed to spawn optical flow sensor\n");
    }
  } else {
    printf("Optical flow sensor blueprint not found\n");
  }
  
  // Run simulation for a few seconds to collect data
  printf("Collecting sensor data for 10 seconds...\n");
  for (int i = 0; i < 10; i++) {
    carla_world_tick(world, 5000); // 5 second timeout
    sleep(1);
    printf("Tick %d/10\n", i + 1);
  }
  
  printf("Example completed!\n");
  
cleanup:
  if (spawn_points) {
    carla_transform_list_free(spawn_points);
  }
  carla_map_free(map);
  carla_world_free(world);
  carla_client_free(client);
  
  return 0;
}