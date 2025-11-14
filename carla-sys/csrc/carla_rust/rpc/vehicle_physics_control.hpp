#pragma once

#include <vector>
#include "carla/geom/Location.h"
#include "carla/rpc/GearPhysicsControl.h"
#include "carla/rpc/WheelPhysicsControl.h"
#include "carla/rpc/VehiclePhysicsControl.h"
#include "carla_rust/geom.hpp"

namespace carla_rust {
namespace rpc {
using carla::geom::Location;
using carla::geom::Vector2D;
using carla::rpc::GearPhysicsControl;
using carla::rpc::VehiclePhysicsControl;
using carla::rpc::WheelPhysicsControl;
using carla_rust::geom::FfiLocation;

class FfiVehiclePhysicsControl {
public:
    FfiVehiclePhysicsControl() = default;

    FfiVehiclePhysicsControl(
        const std::vector<Vector2D>& in_torque_curve, float in_max_rpm, float in_moi,
        float in_damping_rate_full_throttle, float in_damping_rate_zero_throttle_clutch_engaged,
        float in_damping_rate_zero_throttle_clutch_disengaged, bool in_use_gear_autobox,
        float in_gear_switch_time, float in_clutch_strength, float in_final_ratio,
        std::vector<GearPhysicsControl>& in_forward_gears, float in_mass, float in_drag_coefficient,
        FfiLocation& in_center_of_mass, const std::vector<Vector2D>& in_steering_curve,
        std::vector<WheelPhysicsControl>& in_wheels, bool in_use_sweep_wheel_collision)
        : inner_(VehiclePhysicsControl(
              in_torque_curve, in_max_rpm, in_moi, in_damping_rate_full_throttle,
              in_damping_rate_zero_throttle_clutch_engaged,
              in_damping_rate_zero_throttle_clutch_disengaged, in_use_gear_autobox,
              in_gear_switch_time, in_clutch_strength, in_final_ratio, in_forward_gears, in_mass,
              in_drag_coefficient, reinterpret_cast<const Location&>(in_center_of_mass),
              in_steering_curve, in_wheels, in_use_sweep_wheel_collision)) {}

    FfiVehiclePhysicsControl(VehiclePhysicsControl&& base) : inner_(std::move(base)) {}

    const std::vector<Vector2D>& torque_curve() const { return inner_.torque_curve; }

    float max_rpm() const { return inner_.max_rpm; }

    float moi() const { return inner_.moi; }

    float damping_rate_full_throttle() const { return inner_.damping_rate_full_throttle; }

    float damping_rate_zero_throttle_clutch_engaged() const {
        return inner_.damping_rate_zero_throttle_clutch_engaged;
    }

    float damping_rate_zero_throttle_clutch_disengaged() const {
        return inner_.damping_rate_zero_throttle_clutch_disengaged;
    }

    bool use_gear_autobox() const { return inner_.use_gear_autobox; }

    float gear_switch_time() const { return inner_.gear_switch_time; }

    float clutch_strength() const { return inner_.clutch_strength; }

    float final_ratio() const { return inner_.final_ratio; }

    const std::vector<GearPhysicsControl>& forward_gears() const { return inner_.forward_gears; }

    float mass() const { return inner_.mass; }

    float drag_coefficient() const { return inner_.drag_coefficient; }

    FfiLocation center_of_mass() const {
        // Safe conversion: FfiLocation and Location have identical memory layout (verified by
        // static_assert)
        FfiLocation result;
        std::memcpy(&result, &inner_.center_of_mass, sizeof(FfiLocation));
        return result;
    }

    const std::vector<Vector2D>& steering_curve() const { return inner_.steering_curve; }

    const std::vector<WheelPhysicsControl>& wheels() const { return inner_.wheels; }

    bool use_sweep_wheel_collision() const { return inner_.use_sweep_wheel_collision; }

    const VehiclePhysicsControl& inner() const { return inner_; }

    const VehiclePhysicsControl& as_native() const { return inner_; }

private:
    VehiclePhysicsControl inner_;
};

static_assert(sizeof(FfiVehiclePhysicsControl) == sizeof(VehiclePhysicsControl),
              "FfiVehiclePhysicsControl has invalid size");
}  // namespace rpc
}  // namespace carla_rust
