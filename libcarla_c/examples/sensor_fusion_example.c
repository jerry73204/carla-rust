#include "carla_c/actor.h"
#include "carla_c/client.h"
#include "carla_c/sensor.h"
#include "carla_c/sensor_fusion.h"
#include "carla_c/world.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Example: Camera-LiDAR fusion for colored point cloud generation
void example_camera_lidar_fusion() {
  printf("=== Camera-LiDAR Fusion Example ===\n");

  // Connect to CARLA server
  carla_client_t *client = carla_client_connect("localhost", 2000, 5000);
  if (!client) {
    printf("Failed to connect to CARLA server\n");
    return;
  }

  // Get world
  carla_world_t *world = carla_client_get_world(client);

  // Get blueprint library
  carla_blueprint_library_t *blueprints =
      carla_world_get_blueprint_library(world);

  // Spawn a vehicle
  carla_actor_blueprint_t *vehicle_bp =
      carla_blueprint_library_find(blueprints, "vehicle.tesla.model3");
  carla_transform_list_t *spawn_points = carla_world_get_spawn_points(world);
  carla_transform_t spawn_point = carla_transform_list_get(spawn_points, 0);

  carla_spawn_result_t vehicle_result =
      carla_world_try_spawn_actor(world, vehicle_bp, &spawn_point, NULL);
  if (vehicle_result.error != CARLA_ERROR_NONE) {
    printf("Failed to spawn vehicle\n");
    goto cleanup;
  }
  carla_actor_t *vehicle = vehicle_result.actor;

  // Create sensor transforms relative to vehicle
  carla_transform_t lidar_transform = {
      .location = {0.0f, 0.0f, 2.5f}, // On top of vehicle
      .rotation = {0.0f, 0.0f, 0.0f}};

  carla_transform_t camera_transform = {
      .location = {0.0f, 0.0f, 2.0f},  // Slightly lower than LiDAR
      .rotation = {-15.0f, 0.0f, 0.0f} // Tilted down slightly
  };

  // Spawn LiDAR sensor
  carla_actor_blueprint_t *lidar_bp =
      carla_blueprint_library_find(blueprints, "sensor.lidar.ray_cast");
  carla_blueprint_set_attribute_string(lidar_bp, "channels", "32");
  carla_blueprint_set_attribute_string(lidar_bp, "points_per_second", "56000");
  carla_blueprint_set_attribute_string(lidar_bp, "rotation_frequency", "10");
  carla_blueprint_set_attribute_string(lidar_bp, "range", "100");

  carla_spawn_result_t lidar_result = carla_world_try_spawn_actor_attached(
      world, lidar_bp, &lidar_transform, vehicle, CARLA_ATTACHMENT_RIGID);
  if (lidar_result.error != CARLA_ERROR_NONE) {
    printf("Failed to spawn LiDAR sensor\n");
    goto cleanup;
  }
  carla_sensor_t *lidar = lidar_result.actor;

  // Spawn RGB camera
  carla_actor_blueprint_t *camera_bp =
      carla_blueprint_library_find(blueprints, "sensor.camera.rgb");
  carla_blueprint_set_attribute_string(camera_bp, "image_size_x", "1920");
  carla_blueprint_set_attribute_string(camera_bp, "image_size_y", "1080");
  carla_blueprint_set_attribute_string(camera_bp, "fov", "90");

  carla_spawn_result_t camera_result = carla_world_try_spawn_actor_attached(
      world, camera_bp, &camera_transform, vehicle, CARLA_ATTACHMENT_RIGID);
  if (camera_result.error != CARLA_ERROR_NONE) {
    printf("Failed to spawn camera sensor\n");
    goto cleanup;
  }
  carla_sensor_t *camera = camera_result.actor;

  // Create fusion pipeline
  carla_fusion_pipeline_config_t fusion_config = {
      .sync_config = {.sync_tolerance_seconds = 0.05, // 50ms tolerance
                      .enable_interpolation = true,
                      .enable_extrapolation = false,
                      .max_extrapolation_seconds = 0.0,
                      .buffer_size_per_sensor = 10,
                      .drop_incomplete_packets = false},
      .target_frame = CARLA_COORD_FRAME_VEHICLE,
      .enable_motion_compensation = true,
      .enable_outlier_rejection = true,
      .processing_rate_hz = 10.0};

  carla_fusion_pipeline_t *fusion_pipeline =
      carla_fusion_pipeline_create(&fusion_config);

  // Register sensors with calibration
  carla_fusion_sensor_info_t lidar_info = {.sensor = lidar,
                                           .sensor_id = "lidar_main",
                                           .expected_type =
                                               CARLA_SENSOR_DATA_LIDAR,
                                           .update_rate_hz = 10.0,
                                           .max_delay_seconds = 0.1,
                                           .is_reference_sensor = true};

  carla_fusion_calibration_t lidar_calib = {
      .sensor_id = "lidar_main",
      .sensor_to_vehicle = lidar_transform,
      .sensor_to_reference = {{0, 0, 0}, {0, 0, 0}}, // Identity (is reference)
      .time_offset_seconds = 0.0,
      .is_calibrated = true};

  carla_fusion_pipeline_add_sensor(fusion_pipeline, &lidar_info, &lidar_calib);

  carla_fusion_sensor_info_t camera_info = {.sensor = camera,
                                            .sensor_id = "camera_front",
                                            .expected_type =
                                                CARLA_SENSOR_DATA_IMAGE,
                                            .update_rate_hz = 30.0,
                                            .max_delay_seconds = 0.05,
                                            .is_reference_sensor = false};

  // Calculate relative transform from camera to LiDAR
  carla_transform_t camera_to_lidar = {
      .location = {camera_transform.location.x - lidar_transform.location.x,
                   camera_transform.location.y - lidar_transform.location.y,
                   camera_transform.location.z - lidar_transform.location.z},
      .rotation = {
          camera_transform.rotation.pitch - lidar_transform.rotation.pitch,
          camera_transform.rotation.yaw - lidar_transform.rotation.yaw,
          camera_transform.rotation.roll - lidar_transform.rotation.roll}};

  carla_fusion_calibration_t camera_calib = {
      .sensor_id = "camera_front",
      .sensor_to_vehicle = camera_transform,
      .sensor_to_reference = camera_to_lidar,
      .time_offset_seconds = 0.0,
      .is_calibrated = true};

  carla_fusion_pipeline_add_sensor(fusion_pipeline, &camera_info,
                                   &camera_calib);

  // Storage for synchronized data
  carla_sensor_data_t *latest_lidar_data = NULL;
  carla_sensor_data_t *latest_camera_data = NULL;

  // Sensor callbacks that feed the fusion pipeline
  void lidar_callback(carla_sensor_data_t * data, void *user_data) {
    carla_fusion_sync_buffer_t *buffer =
        (carla_fusion_sync_buffer_t *)user_data;
    carla_fusion_sync_buffer_add_data(buffer, 0, data); // Index 0 for LiDAR

    // Keep reference for processing
    if (latest_lidar_data) {
      carla_sensor_data_destroy(latest_lidar_data);
    }
    latest_lidar_data = data;
  }

  void camera_callback(carla_sensor_data_t * data, void *user_data) {
    carla_fusion_sync_buffer_t *buffer =
        (carla_fusion_sync_buffer_t *)user_data;
    carla_fusion_sync_buffer_add_data(buffer, 1, data); // Index 1 for camera

    // Keep reference for processing
    if (latest_camera_data) {
      carla_sensor_data_destroy(latest_camera_data);
    }
    latest_camera_data = data;
  }

  // Create sync buffer for the callbacks
  carla_fusion_sync_buffer_t *sync_buffer =
      carla_fusion_sync_buffer_create(2, &fusion_config.sync_config);

  // Start sensors
  carla_sensor_listen(lidar, lidar_callback, sync_buffer);
  carla_sensor_listen(camera, camera_callback, sync_buffer);

  // Start fusion pipeline
  carla_fusion_pipeline_start(fusion_pipeline);

  // Simulation loop
  printf("Running sensor fusion for 10 seconds...\n");
  for (int i = 0; i < 100; i++) { // 10 seconds at 10 Hz
    usleep(100000);               // 100ms

    // Process fusion pipeline
    carla_fusion_data_packet_t *sync_packet = NULL;
    carla_error_t err =
        carla_fusion_pipeline_process(fusion_pipeline, &sync_packet);

    if (err == CARLA_ERROR_NONE && sync_packet != NULL) {
      // Check if we have both sensor data
      if (sync_packet->data_valid[0] && sync_packet->data_valid[1]) {
        printf("Processing synchronized data at timestamp %.3f\n",
               sync_packet->reference_timestamp);

        // Get camera intrinsics
        carla_camera_intrinsics_t intrinsics;
        carla_camera_get_intrinsics(sync_packet->sensor_data[1], &intrinsics);

        // Project LiDAR points onto camera and colorize
        carla_camera_lidar_projection_t projection;
        err = carla_fusion_project_lidar_to_camera(
            sync_packet->sensor_data[0], // LiDAR data
            sync_packet->sensor_data[1], // Camera data
            &lidar_calib, &camera_calib, &intrinsics, &projection);

        if (err == CARLA_ERROR_NONE) {
          printf("Colored %zu LiDAR points (%zu in view, %zu out of view)\n",
                 projection.point_count, projection.points_in_view,
                 projection.points_out_of_view);

          // Save colored point cloud to file
          char filename[256];
          snprintf(filename, sizeof(filename),
                   "colored_pointcloud_frame_%06lu.ply",
                   sync_packet->reference_frame);

          // Write PLY file with colored points
          FILE *ply_file = fopen(filename, "w");
          if (ply_file) {
            // PLY header
            fprintf(ply_file, "ply\n");
            fprintf(ply_file, "format ascii 1.0\n");
            fprintf(ply_file, "element vertex %zu\n",
                    projection.points_in_view);
            fprintf(ply_file, "property float x\n");
            fprintf(ply_file, "property float y\n");
            fprintf(ply_file, "property float z\n");
            fprintf(ply_file, "property uchar red\n");
            fprintf(ply_file, "property uchar green\n");
            fprintf(ply_file, "property uchar blue\n");
            fprintf(ply_file, "end_header\n");

            // Write colored points
            for (size_t j = 0; j < projection.points_in_view; j++) {
              fprintf(
                  ply_file, "%.3f %.3f %.3f %d %d %d\n",
                  projection.colored_points[j].x,
                  projection.colored_points[j].y,
                  projection.colored_points[j].z, projection.point_colors[j].r,
                  projection.point_colors[j].g, projection.point_colors[j].b);
            }

            fclose(ply_file);
            printf("Saved colored point cloud to %s\n", filename);
          }

          carla_fusion_projection_destroy(&projection);
        }
      }

      carla_fusion_data_packet_destroy(sync_packet);
    }
  }

  // Get final statistics
  carla_fusion_statistics_t stats;
  carla_fusion_pipeline_get_statistics(fusion_pipeline, &stats);
  printf("\nFusion Pipeline Statistics:\n");
  printf("  Total packets processed: %zu\n", stats.total_packets_processed);
  printf("  Dropped packets: %zu\n", stats.dropped_packets);
  printf("  Average sync delay: %.3f ms\n", stats.average_sync_delay * 1000);
  printf("  Max sync delay: %.3f ms\n", stats.max_sync_delay * 1000);
  printf("  Average processing time: %.3f ms\n",
         stats.average_processing_time * 1000);

cleanup:
  // Stop sensors
  if (lidar)
    carla_sensor_stop(lidar);
  if (camera)
    carla_sensor_stop(camera);

  // Clean up
  if (fusion_pipeline)
    carla_fusion_pipeline_destroy(fusion_pipeline);
  if (sync_buffer)
    carla_fusion_sync_buffer_destroy(sync_buffer);
  if (latest_lidar_data)
    carla_sensor_data_destroy(latest_lidar_data);
  if (latest_camera_data)
    carla_sensor_data_destroy(latest_camera_data);
  if (lidar)
    carla_actor_destroy(lidar);
  if (camera)
    carla_actor_destroy(camera);
  if (vehicle)
    carla_actor_destroy(vehicle);

  carla_transform_list_destroy(spawn_points);
  carla_blueprint_library_destroy(blueprints);
  carla_world_destroy(world);
  carla_client_destroy(client);
}

// Example: Multi-sensor object tracking
void example_multi_sensor_tracking() {
  printf("\n=== Multi-Sensor Object Tracking Example ===\n");

  // This example would demonstrate:
  // 1. Fusing camera semantic segmentation with LiDAR clustering
  // 2. Tracking objects across multiple sensors
  // 3. Predicting future object positions

  // Implementation details omitted for brevity
  printf("Multi-sensor tracking example not fully implemented\n");
}

// Example: IMU-GNSS-Odometry fusion for localization
void example_localization_fusion() {
  printf("\n=== IMU-GNSS-Odometry Fusion Example ===\n");

  // Initialize localization state
  carla_fusion_localization_state_t loc_state;
  carla_vector3d_t initial_pos = {0.0, 0.0, 0.0};
  carla_rotation_t initial_rot = {0.0, 0.0, 0.0};

  carla_fusion_localization_init(&loc_state, &initial_pos, &initial_rot);

  // Simulate sensor updates
  for (int i = 0; i < 100; i++) {
    double timestamp = i * 0.01; // 100Hz IMU

    // Simulated IMU data
    carla_imu_data_t imu = {
        .accelerometer = {0.1, 0.0, 9.81}, // Small forward acceleration
        .gyroscope = {0.0, 0.0, 0.01},     // Small yaw rate
        .compass = 0.0};

    carla_fusion_localization_update_imu(&loc_state, &imu, timestamp);

    // GNSS update every 1 second
    if (i % 100 == 0) {
      carla_gnss_data_t gnss = {.latitude = 52.5200 + i * 0.00001,
                                .longitude = 13.4050 + i * 0.00001,
                                .altitude = 100.0};

      carla_fusion_localization_update_gnss(&loc_state, &gnss, timestamp, 2.0,
                                            5.0);
    }

    // Get current estimate
    if (i % 10 == 0) { // Every 100ms
      carla_transform_t pose;
      carla_vector3d_t velocity;
      double uncertainty;

      carla_fusion_localization_get_state(&loc_state, &pose, &velocity,
                                          &uncertainty);

      printf("t=%.2f: pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) unc=%.2f\n",
             timestamp, pose.location.x, pose.location.y, pose.location.z,
             velocity.x, velocity.y, velocity.z, uncertainty);
    }
  }
}

int main() {
  printf("CARLA Sensor Fusion Examples\n");
  printf("============================\n\n");

  // Run examples
  example_camera_lidar_fusion();
  example_multi_sensor_tracking();
  example_localization_fusion();

  return 0;
}
