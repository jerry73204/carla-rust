// Compatibility shim: WheelPhysicsControl for CARLA 0.10.0
//
// The real 0.10.0 WheelPhysicsControl contains std::vector<Vector2D>,
// making it non-POD. This shim provides a POD-compatible type matching
// the 0.9.x layout so autocxx can generate bindings.
//
// This file shadows the real carla/rpc/WheelPhysicsControl.h when
// csrc_compat_0100/ is added to the include path before the CARLA headers.

#pragma once

#include "carla/geom/Vector3D.h"
#include "carla/MsgPack.h"

namespace carla {
namespace rpc {

class WheelPhysicsControl {
public:
    WheelPhysicsControl() = default;

    WheelPhysicsControl(
        float in_tire_friction,
        float in_damping_rate,
        float in_max_steer_angle,
        float in_radius,
        float in_max_brake_torque,
        float in_max_handbrake_torque,
        float in_lat_stiff_max_load,
        float in_lat_stiff_value,
        float in_long_stiff_value,
        geom::Vector3D in_position)
        : tire_friction(in_tire_friction),
          damping_rate(in_damping_rate),
          max_steer_angle(in_max_steer_angle),
          radius(in_radius),
          max_brake_torque(in_max_brake_torque),
          max_handbrake_torque(in_max_handbrake_torque),
          lat_stiff_max_load(in_lat_stiff_max_load),
          lat_stiff_value(in_lat_stiff_value),
          long_stiff_value(in_long_stiff_value),
          position(in_position) {}

    float tire_friction = 2.0f;
    float damping_rate = 0.25f;
    float max_steer_angle = 70.0f;
    float radius = 30.0f;
    float max_brake_torque = 1500.0f;
    float max_handbrake_torque = 3000.0f;
    float lat_stiff_max_load = 2.0f;
    float lat_stiff_value = 17.0f;
    float long_stiff_value = 1000.0f;
    geom::Vector3D position = {0.0f, 0.0f, 0.0f};

    bool operator==(const WheelPhysicsControl& rhs) const {
        return
            tire_friction == rhs.tire_friction &&
            damping_rate == rhs.damping_rate &&
            max_steer_angle == rhs.max_steer_angle &&
            radius == rhs.radius &&
            max_brake_torque == rhs.max_brake_torque &&
            max_handbrake_torque == rhs.max_handbrake_torque &&
            lat_stiff_max_load == rhs.lat_stiff_max_load &&
            lat_stiff_value == rhs.lat_stiff_value &&
            long_stiff_value == rhs.long_stiff_value &&
            position == rhs.position;
    }

    bool operator!=(const WheelPhysicsControl& rhs) const {
        return !(*this == rhs);
    }

    MSGPACK_DEFINE_ARRAY(
        tire_friction,
        damping_rate,
        max_steer_angle,
        radius,
        max_brake_torque,
        max_handbrake_torque,
        lat_stiff_max_load,
        lat_stiff_value,
        long_stiff_value,
        position)
};

}  // namespace rpc
}  // namespace carla
