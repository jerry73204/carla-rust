#pragma once

#include <cstdint>

#ifdef CARLA_VERSION_0100

#include <cstring>
#include <vector>
#include "carla/geom/Vector2D.h"
#include "carla/geom/Vector3D.h"
#include "carla/geom/Location.h"
#include "carla_rust/geom.hpp"

namespace carla_rust {
namespace rpc {

using carla::geom::Location;
using carla::geom::Vector2D;
using carla::geom::Vector3D;
using carla_rust::geom::FfiLocation;
using carla_rust::geom::FfiVector3D;

// The real WheelPhysicsControl on 0.10.0 is non-POD (contains std::vector).
// We can't include the real header because csrc_compat_0100 shadows it with a
// POD stub. So we define the wrapper with inline storage matching the real
// 0.10.0 struct layout.
class FfiWheelPhysicsControl {
public:
    FfiWheelPhysicsControl() = default;

    // --- Scalar fields ---
    uint8_t axle_type = 0;
    Vector3D offset = Vector3D(0, 0, 0);
    float wheel_radius = 30.0f;
    float wheel_width = 30.0f;
    float wheel_mass = 30.0f;
    float cornering_stiffness = 1000.0f;
    float friction_force_multiplier = 3.0f;
    float side_slip_modifier = 1.0f;
    float slip_threshold = 20.0f;
    float skid_threshold = 20.0f;
    float max_steer_angle = 70.0f;
    bool affected_by_steering = true;
    bool affected_by_brake = true;
    bool affected_by_handbrake = true;
    bool affected_by_engine = true;
    bool abs_enabled = false;
    bool traction_control_enabled = false;
    float max_wheelspin_rotation = 30.0f;
    uint8_t external_torque_combine_method = 0;
    std::vector<Vector2D> lateral_slip_graph_ = {};
    Vector3D suspension_axis = Vector3D(0, 0, -1);
    Vector3D suspension_force_offset = Vector3D(0, 0, 0);
    float suspension_max_raise = 10.0f;
    float suspension_max_drop = 10.0f;
    float suspension_damping_ratio = 0.5f;
    float wheel_load_ratio = 0.5f;
    float spring_rate = 250.0f;
    float spring_preload = 50.0f;
    int32_t suspension_smoothing = 0;
    float rollbar_scaling = 0.15f;
    uint8_t sweep_shape = 0;
    uint8_t sweep_type = 0;
    float max_brake_torque = 1500.0f;
    float max_hand_brake_torque = 3000.0f;
    int32_t wheel_index = -1;
    Location location_ = Location(0, 0, 0);
    Location old_location_ = Location(0, 0, 0);
    Location velocity_ = Location(0, 0, 0);

    // --- Getters ---
    uint8_t get_axle_type() const { return axle_type; }
    float get_wheel_radius() const { return wheel_radius; }
    float get_wheel_width() const { return wheel_width; }
    float get_wheel_mass() const { return wheel_mass; }
    float get_cornering_stiffness() const { return cornering_stiffness; }
    float get_friction_force_multiplier() const { return friction_force_multiplier; }
    float get_side_slip_modifier() const { return side_slip_modifier; }
    float get_slip_threshold() const { return slip_threshold; }
    float get_skid_threshold() const { return skid_threshold; }
    float get_max_steer_angle() const { return max_steer_angle; }
    bool get_affected_by_steering() const { return affected_by_steering; }
    bool get_affected_by_brake() const { return affected_by_brake; }
    bool get_affected_by_handbrake() const { return affected_by_handbrake; }
    bool get_affected_by_engine() const { return affected_by_engine; }
    bool get_abs_enabled() const { return abs_enabled; }
    bool get_traction_control_enabled() const { return traction_control_enabled; }
    float get_max_wheelspin_rotation() const { return max_wheelspin_rotation; }
    uint8_t get_external_torque_combine_method() const { return external_torque_combine_method; }
    float get_suspension_max_raise() const { return suspension_max_raise; }
    float get_suspension_max_drop() const { return suspension_max_drop; }
    float get_suspension_damping_ratio() const { return suspension_damping_ratio; }
    float get_wheel_load_ratio() const { return wheel_load_ratio; }
    float get_spring_rate() const { return spring_rate; }
    float get_spring_preload() const { return spring_preload; }
    int32_t get_suspension_smoothing() const { return suspension_smoothing; }
    float get_rollbar_scaling() const { return rollbar_scaling; }
    uint8_t get_sweep_shape() const { return sweep_shape; }
    uint8_t get_sweep_type() const { return sweep_type; }
    float get_max_brake_torque() const { return max_brake_torque; }
    float get_max_hand_brake_torque() const { return max_hand_brake_torque; }
    int32_t get_wheel_index() const { return wheel_index; }

    // Vector3D getters (via memcpy to FfiVector3D)
    FfiVector3D get_offset() const {
        FfiVector3D result;
        std::memcpy(&result, &offset, sizeof(FfiVector3D));
        return result;
    }

    FfiVector3D get_suspension_axis() const {
        FfiVector3D result;
        std::memcpy(&result, &suspension_axis, sizeof(FfiVector3D));
        return result;
    }

    FfiVector3D get_suspension_force_offset() const {
        FfiVector3D result;
        std::memcpy(&result, &suspension_force_offset, sizeof(FfiVector3D));
        return result;
    }

    // Location getters (via memcpy to FfiLocation)
    FfiLocation get_location() const {
        FfiLocation result;
        std::memcpy(&result, &location_, sizeof(FfiLocation));
        return result;
    }

    FfiLocation get_old_location() const {
        FfiLocation result;
        std::memcpy(&result, &old_location_, sizeof(FfiLocation));
        return result;
    }

    FfiLocation get_velocity() const {
        FfiLocation result;
        std::memcpy(&result, &velocity_, sizeof(FfiLocation));
        return result;
    }

    // Vector reference getter for lateral_slip_graph
    const std::vector<Vector2D>& get_lateral_slip_graph() const { return lateral_slip_graph_; }

    // --- Setters ---
    void set_axle_type(uint8_t v) { axle_type = v; }
    void set_offset(const FfiVector3D& v) { std::memcpy(&offset, &v, sizeof(Vector3D)); }
    void set_wheel_radius(float v) { wheel_radius = v; }
    void set_wheel_width(float v) { wheel_width = v; }
    void set_wheel_mass(float v) { wheel_mass = v; }
    void set_cornering_stiffness(float v) { cornering_stiffness = v; }
    void set_friction_force_multiplier(float v) { friction_force_multiplier = v; }
    void set_side_slip_modifier(float v) { side_slip_modifier = v; }
    void set_slip_threshold(float v) { slip_threshold = v; }
    void set_skid_threshold(float v) { skid_threshold = v; }
    void set_max_steer_angle(float v) { max_steer_angle = v; }
    void set_affected_by_steering(bool v) { affected_by_steering = v; }
    void set_affected_by_brake(bool v) { affected_by_brake = v; }
    void set_affected_by_handbrake(bool v) { affected_by_handbrake = v; }
    void set_affected_by_engine(bool v) { affected_by_engine = v; }
    void set_abs_enabled(bool v) { abs_enabled = v; }
    void set_traction_control_enabled(bool v) { traction_control_enabled = v; }
    void set_max_wheelspin_rotation(float v) { max_wheelspin_rotation = v; }
    void set_external_torque_combine_method(uint8_t v) { external_torque_combine_method = v; }
    void set_lateral_slip_graph(const std::vector<Vector2D>& v) { lateral_slip_graph_ = v; }
    void set_suspension_axis(const FfiVector3D& v) {
        std::memcpy(&suspension_axis, &v, sizeof(Vector3D));
    }
    void set_suspension_force_offset(const FfiVector3D& v) {
        std::memcpy(&suspension_force_offset, &v, sizeof(Vector3D));
    }
    void set_suspension_max_raise(float v) { suspension_max_raise = v; }
    void set_suspension_max_drop(float v) { suspension_max_drop = v; }
    void set_suspension_damping_ratio(float v) { suspension_damping_ratio = v; }
    void set_wheel_load_ratio(float v) { wheel_load_ratio = v; }
    void set_spring_rate(float v) { spring_rate = v; }
    void set_spring_preload(float v) { spring_preload = v; }
    void set_suspension_smoothing(int32_t v) { suspension_smoothing = v; }
    void set_rollbar_scaling(float v) { rollbar_scaling = v; }
    void set_sweep_shape(uint8_t v) { sweep_shape = v; }
    void set_sweep_type(uint8_t v) { sweep_type = v; }
    void set_max_brake_torque(float v) { max_brake_torque = v; }
    void set_max_hand_brake_torque(float v) { max_hand_brake_torque = v; }
    void set_wheel_index(int32_t v) { wheel_index = v; }
    void set_location(const FfiLocation& v) { std::memcpy(&location_, &v, sizeof(Location)); }
    void set_old_location(const FfiLocation& v) {
        std::memcpy(&old_location_, &v, sizeof(Location));
    }
    void set_velocity(const FfiLocation& v) { std::memcpy(&velocity_, &v, sizeof(Location)); }
};

}  // namespace rpc
}  // namespace carla_rust

#else  // !CARLA_VERSION_0100

// On 0.9.x, WheelPhysicsControl is POD and used directly via generate_pod!.
// No wrapper needed.

namespace carla_rust {
namespace rpc {

struct FfiWheelPhysicsControl {
    uint8_t _unused;
};

}  // namespace rpc
}  // namespace carla_rust

#endif  // CARLA_VERSION_0100
