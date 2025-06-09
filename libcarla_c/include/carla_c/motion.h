#ifndef CARLA_C_MOTION_H
#define CARLA_C_MOTION_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Actor motion state access

// Get dynamic state (velocity, acceleration, angular velocity) for any actor
carla_error_t carla_actor_get_dynamic_state(const carla_actor_t* actor, carla_actor_dynamic_state_t* state);

// Get linear velocity of actor in world coordinates
carla_vector3d_t carla_actor_get_velocity(const carla_actor_t* actor);

// Get angular velocity of actor 
carla_vector3d_t carla_actor_get_angular_velocity(const carla_actor_t* actor);

// Get acceleration of actor
carla_vector3d_t carla_actor_get_acceleration(const carla_actor_t* actor);

// Get speed (magnitude of velocity) of actor
float carla_actor_get_speed(const carla_actor_t* actor);

// Vehicle physics state access (for vehicle actors only)

// Get detailed vehicle physics state
carla_error_t carla_vehicle_get_physics_state(const carla_actor_t* vehicle, carla_vehicle_physics_state_t* state);

// Get vehicle speed in km/h
float carla_vehicle_get_speed_kmh(const carla_actor_t* vehicle);

// Get vehicle speed in m/s  
float carla_vehicle_get_speed_ms(const carla_actor_t* vehicle);

// Get current engine RPM
float carla_vehicle_get_rpm(const carla_actor_t* vehicle);

// Get current gear
int32_t carla_vehicle_get_gear(const carla_actor_t* vehicle);

// Optical flow sensor data processing

// Get optical flow data from sensor data
carla_optical_flow_data_t carla_sensor_data_get_optical_flow(const carla_sensor_data_t* data);

// Get optical flow at specific pixel
carla_optical_flow_pixel_t carla_optical_flow_get_pixel(const carla_sensor_data_t* data, uint32_t x, uint32_t y);

// Calculate average optical flow in region
carla_optical_flow_pixel_t carla_optical_flow_get_region_average(const carla_sensor_data_t* data, 
                                                                uint32_t x, uint32_t y, 
                                                                uint32_t width, uint32_t height);

// Calculate total motion magnitude from optical flow
float carla_optical_flow_get_motion_magnitude(const carla_sensor_data_t* data);

// Motion analysis and processing utilities

// Analyze motion from multiple sensor inputs
carla_error_t carla_motion_analyze_multi_sensor(const carla_imu_data_t* imu,
                                               const carla_gnss_data_t* gnss,
                                               const carla_actor_dynamic_state_t* dynamics,
                                               carla_motion_analysis_t* result);

// Calculate motion between two transforms
carla_motion_analysis_t carla_motion_calculate_between_transforms(const carla_transform_t* from,
                                                                 const carla_transform_t* to,
                                                                 double time_delta);

// Calculate relative motion between two actors
carla_motion_analysis_t carla_motion_calculate_relative(const carla_actor_t* actor1,
                                                       const carla_actor_t* actor2);

// Coordinate frame conversion utilities

// Convert vector between coordinate frames
carla_vector3d_t carla_motion_convert_frame(const carla_vector3d_t* vector,
                                           carla_coordinate_frame_t from_frame,
                                           carla_coordinate_frame_t to_frame,
                                           const carla_transform_t* reference_transform);

// Transform motion vector from actor frame to world frame
carla_vector3d_t carla_motion_actor_to_world(const carla_vector3d_t* actor_motion,
                                            const carla_transform_t* actor_transform);

// Transform motion vector from world frame to actor frame  
carla_vector3d_t carla_motion_world_to_actor(const carla_vector3d_t* world_motion,
                                            const carla_transform_t* actor_transform);

// Transform motion vector from sensor frame to world frame
carla_vector3d_t carla_motion_sensor_to_world(const carla_vector3d_t* sensor_motion,
                                             const carla_transform_t* sensor_transform);

// Motion filtering and smoothing

// Apply low-pass filter to motion data  
carla_vector3d_t carla_motion_filter_lowpass(const carla_vector3d_t* motion,
                                            const carla_vector3d_t* previous_motion,
                                            float alpha);

// Calculate motion derivative (acceleration from velocity)
carla_vector3d_t carla_motion_calculate_derivative(const carla_vector3d_t* current_motion,
                                                  const carla_vector3d_t* previous_motion,
                                                  double time_delta);

// Calculate motion integral (velocity from acceleration)
carla_vector3d_t carla_motion_calculate_integral(const carla_vector3d_t* motion,
                                                const carla_vector3d_t* previous_integral,
                                                double time_delta);

// Motion prediction and estimation

// Predict future position based on current motion
carla_transform_t carla_motion_predict_transform(const carla_transform_t* current_transform,
                                               const carla_actor_dynamic_state_t* motion_state,
                                               double time_ahead);

// Estimate motion from position history
carla_actor_dynamic_state_t carla_motion_estimate_from_history(const carla_transform_t* transforms,
                                                              const double* timestamps,
                                                              size_t history_length);

// Utility functions for motion vectors

// Calculate magnitude of 3D motion vector
float carla_motion_vector3d_magnitude(const carla_motion_vector3d_t* vector);

// Calculate magnitude of 2D motion vector
float carla_motion_vector2d_magnitude(const carla_motion_vector2d_t* vector);

// Normalize 3D motion vector
carla_motion_vector3d_t carla_motion_vector3d_normalize(const carla_motion_vector3d_t* vector);

// Normalize 2D motion vector
carla_motion_vector2d_t carla_motion_vector2d_normalize(const carla_motion_vector2d_t* vector);

// Calculate dot product of 3D motion vectors
float carla_motion_vector3d_dot(const carla_motion_vector3d_t* a, const carla_motion_vector3d_t* b);

// Calculate cross product of 3D motion vectors
carla_motion_vector3d_t carla_motion_vector3d_cross(const carla_motion_vector3d_t* a, const carla_motion_vector3d_t* b);

// Motion sensor calibration and noise handling

// Apply noise compensation to IMU data
carla_imu_data_t carla_motion_compensate_imu_noise(const carla_imu_data_t* raw_imu,
                                                  const carla_vector3d_t* accel_bias,
                                                  const carla_vector3d_t* gyro_bias);

// Calibrate IMU sensor bias from static readings
carla_error_t carla_motion_calibrate_imu_bias(const carla_imu_data_t* static_readings,
                                             size_t reading_count,
                                             carla_vector3d_t* accel_bias,
                                             carla_vector3d_t* gyro_bias);

// Apply gravity compensation to IMU accelerometer
carla_vector3d_t carla_motion_compensate_gravity(const carla_vector3d_t* raw_acceleration,
                                               const carla_rotation_t* orientation,
                                               float gravity_magnitude);

#ifdef __cplusplus
}
#endif

#endif // CARLA_C_MOTION_H