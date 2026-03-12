// Compatibility shim: VehiclePhysicsControl for CARLA 0.10.0
//
// The real 0.10.0 VehiclePhysicsControl has a completely different layout
// (no GearPhysicsControl, uses flat gear ratio vectors, new fields).
// This shim provides the 0.9.x-compatible layout for autocxx binding generation.
// The actual 0.10.0 type is accessed through FfiVehiclePhysicsControl wrapper
// which is guarded with #ifdef CARLA_VERSION_0100.

#pragma once

#include "carla/MsgPack.h"
#include "carla/geom/Location.h"
#include "carla/geom/Vector2D.h"
#include "carla/rpc/WheelPhysicsControl.h"
#include "carla_rust/rpc/gear_physics_control_compat.hpp"

#include <string>
#include <vector>

namespace carla {
namespace rpc {

class VehiclePhysicsControl {
public:
    VehiclePhysicsControl() = default;

    VehiclePhysicsControl(
        std::vector<geom::Vector2D> in_torque_curve,
        float in_max_rpm,
        float in_moi,
        float in_damping_rate_full_throttle,
        float in_damping_rate_zero_throttle_clutch_engaged,
        float in_damping_rate_zero_throttle_clutch_disengaged,
        bool in_use_gear_autobox,
        float in_gear_switch_time,
        float in_clutch_strength,
        float in_final_ratio,
        std::vector<GearPhysicsControl> in_forward_gears,
        float in_mass,
        float in_drag_coefficient,
        geom::Location in_center_of_mass,
        std::vector<geom::Vector2D> in_steering_curve,
        std::vector<WheelPhysicsControl> in_wheels,
        bool in_use_sweep_wheel_collision)
        : torque_curve(std::move(in_torque_curve)),
          max_rpm(in_max_rpm),
          moi(in_moi),
          damping_rate_full_throttle(in_damping_rate_full_throttle),
          damping_rate_zero_throttle_clutch_engaged(in_damping_rate_zero_throttle_clutch_engaged),
          damping_rate_zero_throttle_clutch_disengaged(in_damping_rate_zero_throttle_clutch_disengaged),
          use_gear_autobox(in_use_gear_autobox),
          gear_switch_time(in_gear_switch_time),
          clutch_strength(in_clutch_strength),
          final_ratio(in_final_ratio),
          forward_gears(std::move(in_forward_gears)),
          mass(in_mass),
          drag_coefficient(in_drag_coefficient),
          center_of_mass(std::move(in_center_of_mass)),
          steering_curve(std::move(in_steering_curve)),
          wheels(std::move(in_wheels)),
          use_sweep_wheel_collision(in_use_sweep_wheel_collision) {}

    std::vector<geom::Vector2D> torque_curve;
    float max_rpm = 5000.0f;
    float moi = 1.0f;
    float damping_rate_full_throttle = 0.15f;
    float damping_rate_zero_throttle_clutch_engaged = 2.0f;
    float damping_rate_zero_throttle_clutch_disengaged = 0.35f;
    bool use_gear_autobox = true;
    float gear_switch_time = 0.5f;
    float clutch_strength = 10.0f;
    float final_ratio = 4.0f;
    std::vector<GearPhysicsControl> forward_gears;
    float mass = 1000.0f;
    float drag_coefficient = 0.3f;
    geom::Location center_of_mass;
    std::vector<geom::Vector2D> steering_curve;
    std::vector<WheelPhysicsControl> wheels;
    bool use_sweep_wheel_collision = false;

    bool operator==(const VehiclePhysicsControl&) const = default;
    bool operator!=(const VehiclePhysicsControl&) const = default;

    MSGPACK_DEFINE_ARRAY(
        torque_curve,
        max_rpm,
        moi,
        damping_rate_full_throttle,
        damping_rate_zero_throttle_clutch_engaged,
        damping_rate_zero_throttle_clutch_disengaged,
        use_gear_autobox,
        gear_switch_time,
        clutch_strength,
        final_ratio,
        forward_gears,
        mass,
        drag_coefficient,
        center_of_mass,
        steering_curve,
        wheels,
        use_sweep_wheel_collision)
};

}  // namespace rpc
}  // namespace carla
