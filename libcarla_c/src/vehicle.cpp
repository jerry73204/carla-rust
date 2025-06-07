#include "carla_c/vehicle.h"
#include "internal.h"
#include "carla/client/Vehicle.h"
#include "carla/client/TrafficLight.h"
#include "carla/rpc/VehicleControl.h"
#include "carla/rpc/VehicleAckermannControl.h"
#include "carla/rpc/AckermannControllerSettings.h"
#include "carla/rpc/VehicleLightState.h"
#include "carla/rpc/VehicleDoor.h"
#include "carla/rpc/VehicleWheels.h"
#include "carla/rpc/VehicleFailureState.h"
#include "carla/rpc/TrafficLightState.h"

namespace {
    // Helper functions to convert between C and C++ types
    
    carla::rpc::VehicleControl ToVehicleControl(const carla_vehicle_control_t* control) {
        carla::rpc::VehicleControl cpp_control;
        cpp_control.throttle = control->throttle;
        cpp_control.steer = control->steer;
        cpp_control.brake = control->brake;
        cpp_control.hand_brake = control->hand_brake;
        cpp_control.reverse = control->reverse;
        cpp_control.manual_gear_shift = control->manual_gear_shift;
        cpp_control.gear = control->gear;
        return cpp_control;
    }
    
    carla_vehicle_control_t FromVehicleControl(const carla::rpc::VehicleControl& control) {
        carla_vehicle_control_t c_control;
        c_control.throttle = control.throttle;
        c_control.steer = control.steer;
        c_control.brake = control.brake;
        c_control.hand_brake = control.hand_brake;
        c_control.reverse = control.reverse;
        c_control.manual_gear_shift = control.manual_gear_shift;
        c_control.gear = control.gear;
        return c_control;
    }
    
    carla::rpc::VehicleAckermannControl ToAckermannControl(const carla_vehicle_ackermann_control_t* control) {
        carla::rpc::VehicleAckermannControl cpp_control;
        cpp_control.steer = control->steer;
        cpp_control.steer_speed = control->steer_speed;
        cpp_control.speed = control->speed;
        cpp_control.acceleration = control->acceleration;
        cpp_control.jerk = control->jerk;
        return cpp_control;
    }
    
    carla::rpc::AckermannControllerSettings ToAckermannSettings(const carla_ackermann_controller_settings_t* settings) {
        carla::rpc::AckermannControllerSettings cpp_settings;
        cpp_settings.speed_kp = settings->speed_kp;
        cpp_settings.speed_ki = settings->speed_ki;
        cpp_settings.speed_kd = settings->speed_kd;
        cpp_settings.accel_kp = settings->accel_kp;
        cpp_settings.accel_ki = settings->accel_ki;
        cpp_settings.accel_kd = settings->accel_kd;
        return cpp_settings;
    }
    
    carla_ackermann_controller_settings_t FromAckermannSettings(const carla::rpc::AckermannControllerSettings& settings) {
        carla_ackermann_controller_settings_t c_settings;
        c_settings.speed_kp = settings.speed_kp;
        c_settings.speed_ki = settings.speed_ki;
        c_settings.speed_kd = settings.speed_kd;
        c_settings.accel_kp = settings.accel_kp;
        c_settings.accel_ki = settings.accel_ki;
        c_settings.accel_kd = settings.accel_kd;
        return c_settings;
    }
    
    carla::client::Vehicle::LightState ToLightState(carla_vehicle_light_state_t light_state) {
        return static_cast<carla::client::Vehicle::LightState>(light_state);
    }
    
    carla_vehicle_light_state_t FromLightState(carla::client::Vehicle::LightState light_state) {
        return static_cast<carla_vehicle_light_state_t>(light_state);
    }
    
    carla::rpc::VehicleDoor ToVehicleDoor(carla_vehicle_door_t door) {
        return static_cast<carla::rpc::VehicleDoor>(door);
    }
    
    carla::rpc::VehicleWheelLocation ToWheelLocation(carla_vehicle_wheel_location_t wheel_location) {
        return static_cast<carla::rpc::VehicleWheelLocation>(wheel_location);
    }
    
    carla_vehicle_failure_state_t FromFailureState(carla::rpc::VehicleFailureState failure_state) {
        return static_cast<carla_vehicle_failure_state_t>(failure_state);
    }
    
    carla_traffic_light_state_t FromTrafficLightState(carla::rpc::TrafficLightState traffic_light_state) {
        return static_cast<carla_traffic_light_state_t>(traffic_light_state);
    }
    
    // Helper to get Vehicle from Actor
    std::shared_ptr<carla::client::Vehicle> GetVehicle(carla_vehicle_t* vehicle) {
        if (!vehicle || !vehicle->actor) {
            return nullptr;
        }
        return std::dynamic_pointer_cast<carla::client::Vehicle>(vehicle->actor);
    }
    
    std::shared_ptr<const carla::client::Vehicle> GetVehicle(const carla_vehicle_t* vehicle) {
        if (!vehicle || !vehicle->actor) {
            return nullptr;
        }
        return std::dynamic_pointer_cast<const carla::client::Vehicle>(vehicle->actor);
    }
}

extern "C" {

// Vehicle control operations
carla_error_t carla_vehicle_apply_control(carla_vehicle_t* vehicle, const carla_vehicle_control_t* control) {
    if (!vehicle || !control) {
        return CARLA_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (!cpp_vehicle) {
            return CARLA_ERROR_INVALID_ARGUMENT;
        }
        
        auto cpp_control = ToVehicleControl(control);
        cpp_vehicle->ApplyControl(cpp_control);
        return CARLA_ERROR_NONE;
    } catch (const std::exception&) {
        return CARLA_ERROR_UNKNOWN;
    }
}

carla_error_t carla_vehicle_apply_ackermann_control(carla_vehicle_t* vehicle, const carla_vehicle_ackermann_control_t* control) {
    if (!vehicle || !control) {
        return CARLA_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (!cpp_vehicle) {
            return CARLA_ERROR_INVALID_ARGUMENT;
        }
        
        auto cpp_control = ToAckermannControl(control);
        cpp_vehicle->ApplyAckermannControl(cpp_control);
        return CARLA_ERROR_NONE;
    } catch (const std::exception&) {
        return CARLA_ERROR_UNKNOWN;
    }
}

carla_vehicle_control_t carla_vehicle_get_control(const carla_vehicle_t* vehicle) {
    carla_vehicle_control_t control = {0};
    
    if (!vehicle) {
        return control;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (cpp_vehicle) {
            auto cpp_control = cpp_vehicle->GetControl();
            control = FromVehicleControl(cpp_control);
        }
    } catch (const std::exception&) {
        // Return zero-initialized control on error
    }
    
    return control;
}

// Autopilot control
carla_error_t carla_vehicle_set_autopilot(carla_vehicle_t* vehicle, bool enabled, uint16_t tm_port) {
    if (!vehicle) {
        return CARLA_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (!cpp_vehicle) {
            return CARLA_ERROR_INVALID_ARGUMENT;
        }
        
        cpp_vehicle->SetAutopilot(enabled, tm_port);
        return CARLA_ERROR_NONE;
    } catch (const std::exception&) {
        return CARLA_ERROR_UNKNOWN;
    }
}

carla_error_t carla_vehicle_set_autopilot_default_port(carla_vehicle_t* vehicle, bool enabled) {
    // Use CARLA's default traffic manager port (8000)
    return carla_vehicle_set_autopilot(vehicle, enabled, 8000);
}

// Vehicle lighting
carla_error_t carla_vehicle_set_light_state(carla_vehicle_t* vehicle, carla_vehicle_light_state_t light_state) {
    if (!vehicle) {
        return CARLA_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (!cpp_vehicle) {
            return CARLA_ERROR_INVALID_ARGUMENT;
        }
        
        auto cpp_light_state = ToLightState(light_state);
        cpp_vehicle->SetLightState(cpp_light_state);
        return CARLA_ERROR_NONE;
    } catch (const std::exception&) {
        return CARLA_ERROR_UNKNOWN;
    }
}

carla_vehicle_light_state_t carla_vehicle_get_light_state(const carla_vehicle_t* vehicle) {
    if (!vehicle) {
        return CARLA_VEHICLE_LIGHT_NONE;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (cpp_vehicle) {
            auto light_state = cpp_vehicle->GetLightState();
            return FromLightState(light_state);
        }
    } catch (const std::exception&) {
        // Return NONE on error
    }
    
    return CARLA_VEHICLE_LIGHT_NONE;
}

// Vehicle door control (CARLA 0.10.0)
carla_error_t carla_vehicle_open_door(carla_vehicle_t* vehicle, carla_vehicle_door_t door) {
    if (!vehicle) {
        return CARLA_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (!cpp_vehicle) {
            return CARLA_ERROR_INVALID_ARGUMENT;
        }
        
        auto cpp_door = ToVehicleDoor(door);
        cpp_vehicle->OpenDoor(cpp_door);
        return CARLA_ERROR_NONE;
    } catch (const std::exception&) {
        return CARLA_ERROR_UNKNOWN;
    }
}

carla_error_t carla_vehicle_close_door(carla_vehicle_t* vehicle, carla_vehicle_door_t door) {
    if (!vehicle) {
        return CARLA_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (!cpp_vehicle) {
            return CARLA_ERROR_INVALID_ARGUMENT;
        }
        
        auto cpp_door = ToVehicleDoor(door);
        cpp_vehicle->CloseDoor(cpp_door);
        return CARLA_ERROR_NONE;
    } catch (const std::exception&) {
        return CARLA_ERROR_UNKNOWN;
    }
}

// Wheel control
carla_error_t carla_vehicle_set_wheel_steer_direction(carla_vehicle_t* vehicle, 
                                                      carla_vehicle_wheel_location_t wheel_location, 
                                                      float angle_in_deg) {
    if (!vehicle) {
        return CARLA_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (!cpp_vehicle) {
            return CARLA_ERROR_INVALID_ARGUMENT;
        }
        
        auto cpp_wheel_location = ToWheelLocation(wheel_location);
        cpp_vehicle->SetWheelSteerDirection(cpp_wheel_location, angle_in_deg);
        return CARLA_ERROR_NONE;
    } catch (const std::exception&) {
        return CARLA_ERROR_UNKNOWN;
    }
}

float carla_vehicle_get_wheel_steer_angle(const carla_vehicle_t* vehicle, 
                                          carla_vehicle_wheel_location_t wheel_location) {
    if (!vehicle) {
        return 0.0f;
    }
    
    try {
        // This method is not const in CARLA, so we need to const_cast
        auto cpp_vehicle = std::const_pointer_cast<carla::client::Vehicle>(GetVehicle(vehicle));
        if (cpp_vehicle) {
            auto cpp_wheel_location = ToWheelLocation(wheel_location);
            return cpp_vehicle->GetWheelSteerAngle(cpp_wheel_location);
        }
    } catch (const std::exception&) {
        // Return 0.0 on error
    }
    
    return 0.0f;
}

// Ackermann controller settings
carla_error_t carla_vehicle_apply_ackermann_controller_settings(carla_vehicle_t* vehicle, 
                                                                const carla_ackermann_controller_settings_t* settings) {
    if (!vehicle || !settings) {
        return CARLA_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (!cpp_vehicle) {
            return CARLA_ERROR_INVALID_ARGUMENT;
        }
        
        auto cpp_settings = ToAckermannSettings(settings);
        cpp_vehicle->ApplyAckermannControllerSettings(cpp_settings);
        return CARLA_ERROR_NONE;
    } catch (const std::exception&) {
        return CARLA_ERROR_UNKNOWN;
    }
}

carla_ackermann_controller_settings_t carla_vehicle_get_ackermann_controller_settings(const carla_vehicle_t* vehicle) {
    carla_ackermann_controller_settings_t settings = {0};
    
    if (!vehicle) {
        return settings;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (cpp_vehicle) {
            auto cpp_settings = cpp_vehicle->GetAckermannControllerSettings();
            settings = FromAckermannSettings(cpp_settings);
        }
    } catch (const std::exception&) {
        // Return zero-initialized settings on error
    }
    
    return settings;
}

// Vehicle state queries
float carla_vehicle_get_speed_limit(const carla_vehicle_t* vehicle) {
    if (!vehicle) {
        return 0.0f;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (cpp_vehicle) {
            return cpp_vehicle->GetSpeedLimit();
        }
    } catch (const std::exception&) {
        // Return 0.0 on error
    }
    
    return 0.0f;
}

carla_traffic_light_state_t carla_vehicle_get_traffic_light_state(const carla_vehicle_t* vehicle) {
    if (!vehicle) {
        return CARLA_TRAFFIC_LIGHT_UNKNOWN;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (cpp_vehicle) {
            auto state = cpp_vehicle->GetTrafficLightState();
            return FromTrafficLightState(state);
        }
    } catch (const std::exception&) {
        // Return UNKNOWN on error
    }
    
    return CARLA_TRAFFIC_LIGHT_UNKNOWN;
}

bool carla_vehicle_is_at_traffic_light(const carla_vehicle_t* vehicle) {
    if (!vehicle) {
        return false;
    }
    
    try {
        // This method is not const in CARLA, so we need to const_cast
        auto cpp_vehicle = std::const_pointer_cast<carla::client::Vehicle>(GetVehicle(vehicle));
        if (cpp_vehicle) {
            return cpp_vehicle->IsAtTrafficLight();
        }
    } catch (const std::exception&) {
        // Return false on error
    }
    
    return false;
}

carla_actor_t* carla_vehicle_get_traffic_light(const carla_vehicle_t* vehicle) {
    if (!vehicle) {
        return nullptr;
    }
    
    try {
        // This method is not const in CARLA, so we need to const_cast
        auto cpp_vehicle = std::const_pointer_cast<carla::client::Vehicle>(GetVehicle(vehicle));
        if (cpp_vehicle) {
            auto traffic_light = cpp_vehicle->GetTrafficLight();
            if (traffic_light) {
                // Create a new carla_actor_t wrapper for the traffic light
                auto* result = new carla_actor_t;
                result->actor = std::static_pointer_cast<carla::client::Actor>(traffic_light);
                return result;
            }
        }
    } catch (const std::exception&) {
        // Return nullptr on error
    }
    
    return nullptr;
}

carla_vehicle_failure_state_t carla_vehicle_get_failure_state(const carla_vehicle_t* vehicle) {
    if (!vehicle) {
        return CARLA_VEHICLE_FAILURE_NONE;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (cpp_vehicle) {
            auto state = cpp_vehicle->GetFailureState();
            return FromFailureState(state);
        }
    } catch (const std::exception&) {
        // Return NONE on error
    }
    
    return CARLA_VEHICLE_FAILURE_NONE;
}

// Debug and telemetry
carla_error_t carla_vehicle_show_debug_telemetry(carla_vehicle_t* vehicle, bool enabled) {
    if (!vehicle) {
        return CARLA_ERROR_INVALID_ARGUMENT;
    }
    
    try {
        auto cpp_vehicle = GetVehicle(vehicle);
        if (!cpp_vehicle) {
            return CARLA_ERROR_INVALID_ARGUMENT;
        }
        
        cpp_vehicle->ShowDebugTelemetry(enabled);
        return CARLA_ERROR_NONE;
    } catch (const std::exception&) {
        return CARLA_ERROR_UNKNOWN;
    }
}

// Utility functions
bool carla_actor_is_vehicle(const carla_actor_t* actor) {
    if (!actor || !actor->actor) {
        return false;
    }
    
    try {
        auto vehicle = std::dynamic_pointer_cast<const carla::client::Vehicle>(actor->actor);
        return vehicle != nullptr;
    } catch (const std::exception&) {
        return false;
    }
}

carla_vehicle_t* carla_actor_as_vehicle(carla_actor_t* actor) {
    if (!carla_actor_is_vehicle(actor)) {
        return nullptr;
    }
    
    // Since carla_vehicle_t is just a typedef of carla_actor_t,
    // we can safely cast if it's a vehicle
    return reinterpret_cast<carla_vehicle_t*>(actor);
}

} // extern "C"