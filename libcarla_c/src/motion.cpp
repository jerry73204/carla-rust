#include "carla_c/motion.h"
#include "carla/client/Actor.h"
#include "carla/client/Vehicle.h"
#include "carla_c/actor.h"
#include "internal.h"
#include <algorithm>
#include <cmath>

namespace {
// Helper to get Actor from opaque pointer
std::shared_ptr<carla::client::Actor> GetActor(const carla_actor_t *actor) {
  if (!actor || !actor->actor) {
    return nullptr;
  }
  return actor->actor;
}

// Helper to get Vehicle from Actor
std::shared_ptr<carla::client::Vehicle> GetVehicle(const carla_actor_t *actor) {
  auto carla_actor = GetActor(actor);
  if (!carla_actor) {
    return nullptr;
  }
  return std::dynamic_pointer_cast<carla::client::Vehicle>(carla_actor);
}

// Convert CARLA Vector3D to C vector
carla_vector3d_t ConvertVector3D(const carla::geom::Vector3D &vec) {
  carla_vector3d_t result;
  result.x = vec.x;
  result.y = vec.y;
  result.z = vec.z;
  return result;
}

// Calculate vector magnitude
float VectorMagnitude(const carla_vector3d_t &vec) {
  return std::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}
} // namespace

extern "C" {

// Actor motion state access

carla_error_t
carla_actor_get_dynamic_state(const carla_actor_t *actor,
                              carla_actor_dynamic_state_t *state) {
  if (!actor || !state) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_actor = GetActor(actor);
    if (!carla_actor) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    // Get actor velocity and acceleration
    auto velocity = carla_actor->GetVelocity();
    auto angular_velocity = carla_actor->GetAngularVelocity();
    auto acceleration = carla_actor->GetAcceleration();

    state->velocity = ConvertVector3D(velocity);
    state->angular_velocity = ConvertVector3D(angular_velocity);
    state->acceleration = ConvertVector3D(acceleration);

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

// Actor velocity/acceleration functions moved to actor.cpp to avoid duplication

float carla_actor_get_speed(const carla_actor_t *actor) {
  if (!actor) {
    return 0.0f;
  }

  try {
    auto carla_actor = GetActor(actor);
    if (carla_actor) {
      auto velocity = carla_actor->GetVelocity();
      return velocity.Length();
    }
  } catch (const std::exception &) {
    // Return zero on error
  }

  return 0.0f;
}

// Vehicle physics state access

carla_error_t
carla_vehicle_get_physics_state(const carla_actor_t *vehicle,
                                carla_vehicle_physics_state_t *state) {
  if (!vehicle || !state) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  try {
    auto carla_vehicle = GetVehicle(vehicle);
    if (!carla_vehicle) {
      return CARLA_ERROR_INVALID_ARGUMENT;
    }

    // Get vehicle physics control to access engine and physics properties
    auto physics_control = carla_vehicle->GetPhysicsControl();
    auto velocity = carla_vehicle->GetVelocity();

    state->speed = velocity.Length();
    state->max_rpm = physics_control.max_rpm;
    state->rpm = 0.0f; // Current RPM not directly available
    state->gear = 0;   // Current gear not directly available
    state->center_of_mass = ConvertVector3D(physics_control.center_of_mass);

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

float carla_vehicle_get_speed_kmh(const carla_actor_t *vehicle) {
  float speed_ms = carla_vehicle_get_speed_ms(vehicle);
  return speed_ms * 3.6f; // Convert m/s to km/h
}

float carla_vehicle_get_speed_ms(const carla_actor_t *vehicle) {
  return carla_actor_get_speed(vehicle);
}

float carla_vehicle_get_rpm(const carla_actor_t *vehicle) {
  if (!vehicle) {
    return 0.0f;
  }

  try {
    auto carla_vehicle = GetVehicle(vehicle);
    if (carla_vehicle) {
      // RPM would need to be calculated from current state
      auto physics_control = carla_vehicle->GetPhysicsControl();
      return physics_control.max_rpm * 0.5f; // Placeholder
    }
  } catch (const std::exception &) {
    // Return zero on error
  }

  return 0.0f;
}

int32_t carla_vehicle_get_gear(const carla_actor_t *vehicle) {
  if (!vehicle) {
    return 0;
  }

  try {
    auto carla_vehicle = GetVehicle(vehicle);
    if (carla_vehicle) {
      auto control = carla_vehicle->GetControl();
      return control.gear;
    }
  } catch (const std::exception &) {
    // Return neutral gear on error
  }

  return 0;
}

// Motion analysis and processing utilities

carla_error_t
carla_motion_analyze_multi_sensor(const carla_imu_data_t *imu,
                                  const carla_gnss_data_t *gnss,
                                  const carla_actor_dynamic_state_t *dynamics,
                                  carla_motion_analysis_t *result) {
  if (!result) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  // Initialize result
  *result = {0};

  try {
    // Combine motion data from multiple sensors
    if (dynamics) {
      result->linear_motion.x = dynamics->velocity.x;
      result->linear_motion.y = dynamics->velocity.y;
      result->linear_motion.z = dynamics->velocity.z;

      result->angular_motion.x = dynamics->angular_velocity.x;
      result->angular_motion.y = dynamics->angular_velocity.y;
      result->angular_motion.z = dynamics->angular_velocity.z;
    }

    // Calculate motion magnitude and direction
    carla_vector3d_t linear_vec = {result->linear_motion.x,
                                   result->linear_motion.y,
                                   result->linear_motion.z};
    result->magnitude = VectorMagnitude(linear_vec);

    if (result->magnitude > 0.0f) {
      result->direction =
          std::atan2(result->linear_motion.y, result->linear_motion.x);
    }

    result->timestamp = 0.0; // Placeholder

    return CARLA_ERROR_NONE;
  } catch (const std::exception &) {
    return CARLA_ERROR_UNKNOWN;
  }
}

carla_motion_analysis_t
carla_motion_calculate_between_transforms(const carla_transform_t *from,
                                          const carla_transform_t *to,
                                          double time_delta) {
  carla_motion_analysis_t result = {0};

  if (!from || !to || time_delta <= 0.0) {
    return result;
  }

  try {
    // Calculate linear motion
    result.linear_motion.x = (to->location.x - from->location.x) / time_delta;
    result.linear_motion.y = (to->location.y - from->location.y) / time_delta;
    result.linear_motion.z = (to->location.z - from->location.z) / time_delta;

    // Calculate angular motion
    result.angular_motion.x =
        (to->rotation.pitch - from->rotation.pitch) / time_delta;
    result.angular_motion.y =
        (to->rotation.yaw - from->rotation.yaw) / time_delta;
    result.angular_motion.z =
        (to->rotation.roll - from->rotation.roll) / time_delta;

    // Calculate magnitude and direction
    carla_vector3d_t linear_vec = {
        result.linear_motion.x, result.linear_motion.y, result.linear_motion.z};
    result.magnitude = VectorMagnitude(linear_vec);

    if (result.magnitude > 0.0f) {
      result.direction =
          std::atan2(result.linear_motion.y, result.linear_motion.x);
    }

    result.timestamp = 0.0; // Would be set to current time

  } catch (const std::exception &) {
    // Return zero-initialized result on error
  }

  return result;
}

carla_motion_analysis_t
carla_motion_calculate_relative(const carla_actor_t *actor1,
                                const carla_actor_t *actor2) {
  carla_motion_analysis_t result = {0};

  if (!actor1 || !actor2) {
    return result;
  }

  try {
    auto velocity1 = carla_actor_get_velocity(actor1);
    auto velocity2 = carla_actor_get_velocity(actor2);

    auto angular_velocity1 = carla_actor_get_angular_velocity(actor1);
    auto angular_velocity2 = carla_actor_get_angular_velocity(actor2);

    // Calculate relative motion
    result.linear_motion.x = velocity1.x - velocity2.x;
    result.linear_motion.y = velocity1.y - velocity2.y;
    result.linear_motion.z = velocity1.z - velocity2.z;

    result.angular_motion.x = angular_velocity1.x - angular_velocity2.x;
    result.angular_motion.y = angular_velocity1.y - angular_velocity2.y;
    result.angular_motion.z = angular_velocity1.z - angular_velocity2.z;

    carla_vector3d_t linear_vec = {
        result.linear_motion.x, result.linear_motion.y, result.linear_motion.z};
    result.magnitude = VectorMagnitude(linear_vec);

    if (result.magnitude > 0.0f) {
      result.direction =
          std::atan2(result.linear_motion.y, result.linear_motion.x);
    }

    result.timestamp = 0.0; // Would be set to current time

  } catch (const std::exception &) {
    // Return zero-initialized result on error
  }

  return result;
}

// Utility functions for motion vectors

float carla_motion_vector3d_magnitude(const carla_motion_vector3d_t *vector) {
  if (!vector) {
    return 0.0f;
  }

  return std::sqrt(vector->x * vector->x + vector->y * vector->y +
                   vector->z * vector->z);
}

float carla_motion_vector2d_magnitude(const carla_motion_vector2d_t *vector) {
  if (!vector) {
    return 0.0f;
  }

  return std::sqrt(vector->x * vector->x + vector->y * vector->y);
}

carla_motion_vector3d_t
carla_motion_vector3d_normalize(const carla_motion_vector3d_t *vector) {
  carla_motion_vector3d_t result = {0, 0, 0};

  if (!vector) {
    return result;
  }

  float magnitude = carla_motion_vector3d_magnitude(vector);

  if (magnitude > 0.0f) {
    result.x = vector->x / magnitude;
    result.y = vector->y / magnitude;
    result.z = vector->z / magnitude;
  }

  return result;
}

carla_motion_vector2d_t
carla_motion_vector2d_normalize(const carla_motion_vector2d_t *vector) {
  carla_motion_vector2d_t result = {0, 0};

  if (!vector) {
    return result;
  }

  float magnitude = carla_motion_vector2d_magnitude(vector);

  if (magnitude > 0.0f) {
    result.x = vector->x / magnitude;
    result.y = vector->y / magnitude;
  }

  return result;
}

float carla_motion_vector3d_dot(const carla_motion_vector3d_t *a,
                                const carla_motion_vector3d_t *b) {
  if (!a || !b) {
    return 0.0f;
  }

  return a->x * b->x + a->y * b->y + a->z * b->z;
}

carla_motion_vector3d_t
carla_motion_vector3d_cross(const carla_motion_vector3d_t *a,
                            const carla_motion_vector3d_t *b) {
  carla_motion_vector3d_t result = {0, 0, 0};

  if (!a || !b) {
    return result;
  }

  result.x = a->y * b->z - a->z * b->y;
  result.y = a->z * b->x - a->x * b->z;
  result.z = a->x * b->y - a->y * b->x;

  return result;
}

// Motion sensor calibration and noise handling

carla_imu_data_t
carla_motion_compensate_imu_noise(const carla_imu_data_t *raw_imu,
                                  const carla_vector3d_t *accel_bias,
                                  const carla_vector3d_t *gyro_bias) {
  carla_imu_data_t result = {0};

  if (!raw_imu) {
    return result;
  }

  result = *raw_imu;

  // Apply bias compensation
  if (accel_bias) {
    result.accelerometer.x -= accel_bias->x;
    result.accelerometer.y -= accel_bias->y;
    result.accelerometer.z -= accel_bias->z;
  }

  if (gyro_bias) {
    result.gyroscope.x -= gyro_bias->x;
    result.gyroscope.y -= gyro_bias->y;
    result.gyroscope.z -= gyro_bias->z;
  }

  return result;
}

carla_error_t carla_motion_calibrate_imu_bias(
    const carla_imu_data_t *static_readings, size_t reading_count,
    carla_vector3d_t *accel_bias, carla_vector3d_t *gyro_bias) {
  if (!static_readings || reading_count == 0 || !accel_bias || !gyro_bias) {
    return CARLA_ERROR_INVALID_ARGUMENT;
  }

  // Calculate average of static readings to determine bias
  carla_vector3d_t accel_sum = {0, 0, 0};
  carla_vector3d_t gyro_sum = {0, 0, 0};

  for (size_t i = 0; i < reading_count; ++i) {
    accel_sum.x += static_readings[i].accelerometer.x;
    accel_sum.y += static_readings[i].accelerometer.y;
    accel_sum.z += static_readings[i].accelerometer.z;

    gyro_sum.x += static_readings[i].gyroscope.x;
    gyro_sum.y += static_readings[i].gyroscope.y;
    gyro_sum.z += static_readings[i].gyroscope.z;
  }

  // Calculate bias (average - expected static values)
  accel_bias->x = accel_sum.x / reading_count;
  accel_bias->y = accel_sum.y / reading_count;
  accel_bias->z =
      (accel_sum.z / reading_count) - 9.81f; // Remove gravity from Z axis

  gyro_bias->x = gyro_sum.x / reading_count;
  gyro_bias->y = gyro_sum.y / reading_count;
  gyro_bias->z = gyro_sum.z / reading_count;

  return CARLA_ERROR_NONE;
}

carla_vector3d_t
carla_motion_compensate_gravity(const carla_vector3d_t *raw_acceleration,
                                const carla_rotation_t *orientation,
                                float gravity_magnitude) {
  carla_vector3d_t result = {0, 0, 0};

  if (!raw_acceleration || !orientation) {
    return result;
  }

  // Convert orientation to gravity vector in body frame
  float pitch_rad = orientation->pitch * M_PI / 180.0f;
  float roll_rad = orientation->roll * M_PI / 180.0f;

  carla_vector3d_t gravity_body;
  gravity_body.x = -gravity_magnitude * std::sin(pitch_rad);
  gravity_body.y = gravity_magnitude * std::sin(roll_rad) * std::cos(pitch_rad);
  gravity_body.z = gravity_magnitude * std::cos(roll_rad) * std::cos(pitch_rad);

  // Remove gravity component from acceleration
  result.x = raw_acceleration->x - gravity_body.x;
  result.y = raw_acceleration->y - gravity_body.y;
  result.z = raw_acceleration->z - gravity_body.z;

  return result;
}

// Stub implementations for optical flow functions
carla_optical_flow_data_t
carla_sensor_data_get_optical_flow(const carla_sensor_data_t *data) {
  carla_optical_flow_data_t flow_data = {0};
  // Optical flow implementation would require sensor data structure access
  return flow_data;
}

carla_optical_flow_pixel_t
carla_optical_flow_get_pixel(const carla_sensor_data_t *data, uint32_t x,
                             uint32_t y) {
  carla_optical_flow_pixel_t pixel = {0, 0};
  // Placeholder implementation
  return pixel;
}

carla_optical_flow_pixel_t
carla_optical_flow_get_region_average(const carla_sensor_data_t *data,
                                      uint32_t x, uint32_t y, uint32_t width,
                                      uint32_t height) {
  carla_optical_flow_pixel_t average = {0, 0};
  // Placeholder implementation
  return average;
}

float carla_optical_flow_get_motion_magnitude(const carla_sensor_data_t *data) {
  // Placeholder implementation
  return 0.0f;
}

// Placeholder implementations for remaining functions
carla_vector3d_t
carla_motion_convert_frame(const carla_vector3d_t *vector,
                           carla_coordinate_frame_t from_frame,
                           carla_coordinate_frame_t to_frame,
                           const carla_transform_t *reference_transform) {
  if (!vector) {
    carla_vector3d_t result = {0, 0, 0};
    return result;
  }
  return *vector; // Simplified implementation
}

carla_vector3d_t
carla_motion_actor_to_world(const carla_vector3d_t *actor_motion,
                            const carla_transform_t *actor_transform) {
  if (!actor_motion) {
    carla_vector3d_t result = {0, 0, 0};
    return result;
  }
  return *actor_motion; // Simplified implementation
}

carla_vector3d_t
carla_motion_world_to_actor(const carla_vector3d_t *world_motion,
                            const carla_transform_t *actor_transform) {
  if (!world_motion) {
    carla_vector3d_t result = {0, 0, 0};
    return result;
  }
  return *world_motion; // Simplified implementation
}

carla_vector3d_t
carla_motion_sensor_to_world(const carla_vector3d_t *sensor_motion,
                             const carla_transform_t *sensor_transform) {
  if (!sensor_motion) {
    carla_vector3d_t result = {0, 0, 0};
    return result;
  }
  return *sensor_motion; // Simplified implementation
}

carla_vector3d_t
carla_motion_filter_lowpass(const carla_vector3d_t *motion,
                            const carla_vector3d_t *previous_motion,
                            float alpha) {
  carla_vector3d_t result = {0, 0, 0};

  if (!motion) {
    return result;
  }

  alpha = std::max(0.0f, std::min(1.0f, alpha));

  if (!previous_motion) {
    result = *motion;
  } else {
    result.x = alpha * motion->x + (1.0f - alpha) * previous_motion->x;
    result.y = alpha * motion->y + (1.0f - alpha) * previous_motion->y;
    result.z = alpha * motion->z + (1.0f - alpha) * previous_motion->z;
  }

  return result;
}

carla_vector3d_t
carla_motion_calculate_derivative(const carla_vector3d_t *current_motion,
                                  const carla_vector3d_t *previous_motion,
                                  double time_delta) {
  carla_vector3d_t result = {0, 0, 0};

  if (!current_motion || !previous_motion || time_delta <= 0.0) {
    return result;
  }

  result.x = (current_motion->x - previous_motion->x) / time_delta;
  result.y = (current_motion->y - previous_motion->y) / time_delta;
  result.z = (current_motion->z - previous_motion->z) / time_delta;

  return result;
}

carla_vector3d_t
carla_motion_calculate_integral(const carla_vector3d_t *motion,
                                const carla_vector3d_t *previous_integral,
                                double time_delta) {
  carla_vector3d_t result = {0, 0, 0};

  if (!motion || time_delta <= 0.0) {
    return result;
  }

  if (previous_integral) {
    result.x = previous_integral->x + motion->x * time_delta;
    result.y = previous_integral->y + motion->y * time_delta;
    result.z = previous_integral->z + motion->z * time_delta;
  } else {
    result.x = motion->x * time_delta;
    result.y = motion->y * time_delta;
    result.z = motion->z * time_delta;
  }

  return result;
}

carla_transform_t
carla_motion_predict_transform(const carla_transform_t *current_transform,
                               const carla_actor_dynamic_state_t *motion_state,
                               double time_ahead) {
  carla_transform_t result = {0};

  if (!current_transform || !motion_state || time_ahead <= 0.0) {
    return result;
  }

  result = *current_transform;

  result.location.x += motion_state->velocity.x * time_ahead;
  result.location.y += motion_state->velocity.y * time_ahead;
  result.location.z += motion_state->velocity.z * time_ahead;

  result.rotation.pitch += motion_state->angular_velocity.x * time_ahead;
  result.rotation.yaw += motion_state->angular_velocity.y * time_ahead;
  result.rotation.roll += motion_state->angular_velocity.z * time_ahead;

  return result;
}

carla_actor_dynamic_state_t
carla_motion_estimate_from_history(const carla_transform_t *transforms,
                                   const double *timestamps,
                                   size_t history_length) {
  carla_actor_dynamic_state_t result = {0};

  if (!transforms || !timestamps || history_length < 2) {
    return result;
  }

  size_t last_idx = history_length - 1;
  size_t prev_idx = history_length - 2;

  double dt = timestamps[last_idx] - timestamps[prev_idx];

  if (dt > 0.0) {
    result.velocity.x =
        (transforms[last_idx].location.x - transforms[prev_idx].location.x) /
        dt;
    result.velocity.y =
        (transforms[last_idx].location.y - transforms[prev_idx].location.y) /
        dt;
    result.velocity.z =
        (transforms[last_idx].location.z - transforms[prev_idx].location.z) /
        dt;

    result.angular_velocity.x = (transforms[last_idx].rotation.pitch -
                                 transforms[prev_idx].rotation.pitch) /
                                dt;
    result.angular_velocity.y = (transforms[last_idx].rotation.yaw -
                                 transforms[prev_idx].rotation.yaw) /
                                dt;
    result.angular_velocity.z = (transforms[last_idx].rotation.roll -
                                 transforms[prev_idx].rotation.roll) /
                                dt;

    if (history_length >= 3) {
      size_t prev2_idx = history_length - 3;
      double dt2 = timestamps[prev_idx] - timestamps[prev2_idx];

      if (dt2 > 0.0) {
        carla_vector3d_t prev_velocity;
        prev_velocity.x = (transforms[prev_idx].location.x -
                           transforms[prev2_idx].location.x) /
                          dt2;
        prev_velocity.y = (transforms[prev_idx].location.y -
                           transforms[prev2_idx].location.y) /
                          dt2;
        prev_velocity.z = (transforms[prev_idx].location.z -
                           transforms[prev2_idx].location.z) /
                          dt2;

        result.acceleration.x = (result.velocity.x - prev_velocity.x) / dt;
        result.acceleration.y = (result.velocity.y - prev_velocity.y) / dt;
        result.acceleration.z = (result.velocity.z - prev_velocity.z) / dt;
      }
    }
  }

  return result;
}

} // extern "C"
