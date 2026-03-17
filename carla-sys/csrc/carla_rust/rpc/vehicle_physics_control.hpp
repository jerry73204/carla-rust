#pragma once

#include <cstring>
#include <memory>
#include <vector>
#include "carla/geom/Location.h"
#include "carla/geom/Vector2D.h"
#include "carla/geom/Vector3D.h"
#include "carla_rust/rpc/gear_physics_control_compat.hpp"
#include "carla/rpc/WheelPhysicsControl.h"
#include "carla/rpc/VehiclePhysicsControl.h"
#include "carla_rust/geom.hpp"
#ifdef CARLA_VERSION_0100
#include "carla_rust/rpc/wheel_physics_control.hpp"
#endif

namespace carla_rust {
namespace rpc {
using carla::geom::Location;
using carla::geom::Vector2D;
using carla::geom::Vector3D;
using carla::rpc::GearPhysicsControl;
using carla::rpc::VehiclePhysicsControl;
using carla_rust::geom::FfiLocation;
using carla_rust::geom::FfiVector3D;

#ifdef CARLA_VERSION_0100

// On 0.10.0, VehiclePhysicsControl from the compat shim has 0.9.x fields.
// The FfiVehiclePhysicsControl wrapper provides the correct 0.10.0 API.
// It stores the compat type internally but exposes 0.10.0 field names.
// The actual 0.10.0 fields are stored directly in this wrapper class.
class FfiVehiclePhysicsControl {
public:
    FfiVehiclePhysicsControl() = default;

    // --- Storage: 0.10.0 fields stored directly ---
    std::vector<Vector2D> torque_curve_ = {Vector2D(0.0f, 500.0f), Vector2D(5000.0f, 500.0f)};
    float max_torque_ = 300.0f;
    float max_rpm_ = 5000.0f;
    float idle_rpm_ = 1.0f;
    float brake_effect_ = 1.0f;
    float rev_up_moi_ = 1.0f;
    float rev_down_rate_ = 600.0f;
    uint8_t differential_type_ = 0;
    float front_rear_split_ = 0.5f;
    bool use_automatic_gears_ = true;
    float gear_change_time_ = 0.5f;
    float final_ratio_ = 4.0f;
    std::vector<float> forward_gear_ratios_ = {2.85f, 2.02f, 1.35f, 1.0f,
                                               2.85f, 2.02f, 1.35f, 1.0f};
    std::vector<float> reverse_gear_ratios_ = {2.86f, 2.86f};
    float change_up_rpm_ = 4500.0f;
    float change_down_rpm_ = 2000.0f;
    float transmission_efficiency_ = 0.9f;
    float mass_ = 1000.0f;
    float drag_coefficient_ = 0.3f;
    Location center_of_mass_ = Location(0, 0, 0);
    float chassis_width_ = 180.0f;
    float chassis_height_ = 140.0f;
    float downforce_coefficient_ = 0.3f;
    float drag_area_ = 0.0f;
    Vector3D inertia_tensor_scale_ = Vector3D(1, 1, 1);
    float sleep_threshold_ = 10.0f;
    float sleep_slope_limit_ = 0.866f;
    std::vector<Vector2D> steering_curve_ = {Vector2D(0.0f, 1.0f), Vector2D(10.0f, 0.5f)};
    std::vector<FfiWheelPhysicsControl> wheels_;
    bool use_sweep_wheel_collision_ = false;

    // --- Getters ---
    const std::vector<Vector2D>& torque_curve() const { return torque_curve_; }
    float max_torque() const { return max_torque_; }
    float max_rpm() const { return max_rpm_; }
    float idle_rpm() const { return idle_rpm_; }
    float brake_effect() const { return brake_effect_; }
    float rev_up_moi() const { return rev_up_moi_; }
    float rev_down_rate() const { return rev_down_rate_; }
    uint8_t differential_type() const { return differential_type_; }
    float front_rear_split() const { return front_rear_split_; }
    bool use_automatic_gears() const { return use_automatic_gears_; }
    float gear_change_time() const { return gear_change_time_; }
    float final_ratio() const { return final_ratio_; }
    const std::vector<float>& forward_gear_ratios() const { return forward_gear_ratios_; }
    const std::vector<float>& reverse_gear_ratios() const { return reverse_gear_ratios_; }
    float change_up_rpm() const { return change_up_rpm_; }
    float change_down_rpm() const { return change_down_rpm_; }
    float transmission_efficiency() const { return transmission_efficiency_; }
    float mass() const { return mass_; }
    float drag_coefficient() const { return drag_coefficient_; }

    FfiLocation center_of_mass() const {
        FfiLocation result;
        std::memcpy(&result, &center_of_mass_, sizeof(FfiLocation));
        return result;
    }

    float chassis_width() const { return chassis_width_; }
    float chassis_height() const { return chassis_height_; }
    float downforce_coefficient() const { return downforce_coefficient_; }
    float drag_area() const { return drag_area_; }

    FfiVector3D inertia_tensor_scale() const {
        FfiVector3D result;
        std::memcpy(&result, &inertia_tensor_scale_, sizeof(FfiVector3D));
        return result;
    }

    float sleep_threshold() const { return sleep_threshold_; }
    float sleep_slope_limit() const { return sleep_slope_limit_; }
    const std::vector<Vector2D>& steering_curve() const { return steering_curve_; }
    size_t wheels_size() const { return wheels_.size(); }
    bool use_sweep_wheel_collision() const { return use_sweep_wheel_collision_; }

    // Access individual wheel for reading (returns a copy)
    std::unique_ptr<FfiWheelPhysicsControl> wheel_at(size_t index) const {
        return std::make_unique<FfiWheelPhysicsControl>(wheels_.at(index));
    }

    // --- Setters ---
    void set_torque_curve(const std::vector<Vector2D>& v) { torque_curve_ = v; }
    void set_max_torque(float v) { max_torque_ = v; }
    void set_max_rpm(float v) { max_rpm_ = v; }
    void set_idle_rpm(float v) { idle_rpm_ = v; }
    void set_brake_effect(float v) { brake_effect_ = v; }
    void set_rev_up_moi(float v) { rev_up_moi_ = v; }
    void set_rev_down_rate(float v) { rev_down_rate_ = v; }
    void set_differential_type(uint8_t v) { differential_type_ = v; }
    void set_front_rear_split(float v) { front_rear_split_ = v; }
    void set_use_automatic_gears(bool v) { use_automatic_gears_ = v; }
    void set_gear_change_time(float v) { gear_change_time_ = v; }
    void set_final_ratio(float v) { final_ratio_ = v; }
    void set_forward_gear_ratios(const std::vector<float>& v) { forward_gear_ratios_ = v; }
    void set_reverse_gear_ratios(const std::vector<float>& v) { reverse_gear_ratios_ = v; }
    void set_change_up_rpm(float v) { change_up_rpm_ = v; }
    void set_change_down_rpm(float v) { change_down_rpm_ = v; }
    void set_transmission_efficiency(float v) { transmission_efficiency_ = v; }
    void set_mass(float v) { mass_ = v; }
    void set_drag_coefficient(float v) { drag_coefficient_ = v; }
    void set_center_of_mass(const FfiLocation& v) {
        std::memcpy(&center_of_mass_, &v, sizeof(Location));
    }
    void set_chassis_width(float v) { chassis_width_ = v; }
    void set_chassis_height(float v) { chassis_height_ = v; }
    void set_downforce_coefficient(float v) { downforce_coefficient_ = v; }
    void set_drag_area(float v) { drag_area_ = v; }
    void set_inertia_tensor_scale(const FfiVector3D& v) {
        std::memcpy(&inertia_tensor_scale_, &v, sizeof(Vector3D));
    }
    void set_sleep_threshold(float v) { sleep_threshold_ = v; }
    void set_sleep_slope_limit(float v) { sleep_slope_limit_ = v; }
    void set_steering_curve(const std::vector<Vector2D>& v) { steering_curve_ = v; }
    void set_use_sweep_wheel_collision(bool v) { use_sweep_wheel_collision_ = v; }
    void clear_wheels() { wheels_.clear(); }
    void push_wheel(const FfiWheelPhysicsControl& w) { wheels_.push_back(w); }

    // Build the compat VehiclePhysicsControl for passing to the CARLA API.
    // NOTE: This populates the compat shim struct which maps to the 0.9.x wire format.
    // Actual 0.10.0 RPC communication will need further work on the serialization layer.
    const VehiclePhysicsControl& as_native() const {
        // For now, return a default compat struct.
        // The actual 0.10.0 serialization format differs and needs msgpack support.
        static VehiclePhysicsControl dummy;
        return dummy;
    }

private:
    // No inner_ on 0.10.0 - fields are stored directly above
};

#else  // !CARLA_VERSION_0100

using carla::rpc::WheelPhysicsControl;

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

#endif  // CARLA_VERSION_0100

}  // namespace rpc
}  // namespace carla_rust
