#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla_rust/compat.hpp"
#include "carla/client/Actor.h"
#include "carla/client/Vehicle.h"
#include "carla/client/Sensor.h"
#include "carla/client/TrafficSign.h"
#include "carla/client/TrafficLight.h"
#include "carla/client/ActorAttribute.h"
#include "carla/geom/BoundingBox.h"
#include "carla/rpc/ActorState.h"
#include "carla_rust/geom.hpp"
#include "carla_rust/rpc/actor_id.hpp"
#include "carla_rust/client/actor_attribute.hpp"
#include "carla_rust/client/actor_attribute_value_list.hpp"
#include "carla_rust/client/result.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::client::Actor;
using carla::client::ActorAttributeValue;
using carla::client::Sensor;
using carla::client::TrafficLight;
using carla::client::TrafficSign;
using carla::client::Vehicle;
using carla::geom::Vector3D;
using carla_rust::client::FfiActorAttributeValue;
using carla_rust::client::FfiActorAttributeValueList;
using carla_rust::geom::FfiLocation;
using carla_rust::geom::FfiTransform;
using carla_rust::rpc::FfiActorId;

class FfiVehicle;
class FfiSensor;
class FfiTrafficSign;
class FfiTrafficLight;
class FfiWorld;

// Actor
class FfiActor {
public:
    FfiActor(SharedPtr<Actor>&& base) : inner_(std::move(base)) {}

    const SharedPtr<Actor>& inner() const { return inner_; }

    FfiLocation GetLocation(FfiError& error) const {
        return ffi_call(error, FfiLocation(), [&]() {
            auto location = inner_->GetLocation();
            return FfiLocation(std::move(location));
        });
    }

    FfiTransform GetTransform(FfiError& error) const {
        return ffi_call(error, FfiTransform(), [&]() {
            auto transform = inner_->GetTransform();
            return FfiTransform(std::move(transform));
        });
    }

    FfiActorId GetId() const { return inner_->GetId(); }

    const std::string& GetTypeId() const { return inner_->GetTypeId(); }

    const std::string& GetDisplayId() const { return inner_->GetDisplayId(); }

    FfiActorId GetParentId() const { return inner_->GetParentId(); }

    const std::vector<uint8_t>& GetSemanticTags() const { return inner_->GetSemanticTags(); }

    std::shared_ptr<FfiActor> GetParent(FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiActor>(), [&]() -> std::shared_ptr<FfiActor> {
            auto parent = inner_->GetParent();
            if (parent == nullptr) {
                return nullptr;
            } else {
                return std::make_shared<FfiActor>(std::move(parent));
            }
        });
    }

    std::unique_ptr<FfiWorld> GetWorld(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWorld>(nullptr), [&]() {
            auto world = inner_->GetWorld();
            return std::make_unique<FfiWorld>(std::move(world));
        });
    }

    std::unique_ptr<FfiActorAttributeValueList> GetAttributes(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiActorAttributeValueList>(nullptr), [&]() {
            auto& orig = inner_->GetAttributes();
            return std::make_unique<FfiActorAttributeValueList>(orig);
        });
    }

    Vector3D GetVelocity(FfiError& error) const {
        return ffi_call(error, Vector3D(), [&]() { return inner_->GetVelocity(); });
    }

    Vector3D GetAngularVelocity(FfiError& error) const {
        return ffi_call(error, Vector3D(), [&]() { return inner_->GetAngularVelocity(); });
    }

    Vector3D GetAcceleration(FfiError& error) const {
        return ffi_call(error, Vector3D(), [&]() { return inner_->GetAcceleration(); });
    }

    void SetLocation(const FfiLocation& location, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetLocation(location.as_native()); });
    }

    void SetTransform(const FfiTransform& transform, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetTransform(transform.as_native()); });
    }

    void SetTargetVelocity(const Vector3D& vector, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetTargetVelocity(vector); });
    }

    void SetTargetAngularVelocity(const Vector3D& vector, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetTargetAngularVelocity(vector); });
    }

    void EnableConstantVelocity(const Vector3D& vector, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->EnableConstantVelocity(vector); });
    }

    void DisableConstantVelocity(FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->DisableConstantVelocity(); });
    }

    void AddImpulse1(const Vector3D& vector, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->AddImpulse(vector); });
    }

    void AddImpulse2(const Vector3D& impulse, const Vector3D& location, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->AddImpulse(impulse, location); });
    }

    void AddForce1(const Vector3D& force, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->AddForce(force); });
    }

    void AddForce2(const Vector3D& force, const Vector3D& location, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->AddForce(force, location); });
    }

    void AddAngularImpulse(const Vector3D& vector, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->AddAngularImpulse(vector); });
    }

    void AddTorque(const Vector3D& vector, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->AddTorque(vector); });
    }

    void SetSimulatePhysics(bool enabled, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetSimulatePhysics(enabled); });
    }

    void SetEnableGravity(bool enabled, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetEnableGravity(enabled); });
    }

    geom::BoundingBox GetBoundingBox() const { return inner_->GetBoundingBox(); }

#ifdef CARLA_VERSION_0916
    geom::Transform GetComponentWorldTransform(const std::string& component_name,
                                               FfiError& error) const {
        return ffi_call(error, geom::Transform(),
                        [&]() { return inner_->GetComponentWorldTransform(component_name); });
    }

    geom::Transform GetComponentRelativeTransform(const std::string& component_name,
                                                  FfiError& error) const {
        return ffi_call(error, geom::Transform(),
                        [&]() { return inner_->GetComponentRelativeTransform(component_name); });
    }

    std::vector<geom::Transform> GetBoneWorldTransforms(FfiError& error) const {
        return ffi_call(error, std::vector<geom::Transform>(),
                        [&]() { return inner_->GetBoneWorldTransforms(); });
    }

    std::vector<geom::Transform> GetBoneRelativeTransforms(FfiError& error) const {
        return ffi_call(error, std::vector<geom::Transform>(),
                        [&]() { return inner_->GetBoneRelativeTransforms(); });
    }

    std::vector<std::string> GetComponentNames(FfiError& error) const {
        return ffi_call(error, std::vector<std::string>(),
                        [&]() { return inner_->GetComponentNames(); });
    }

    std::vector<std::string> GetBoneNames(FfiError& error) const {
        return ffi_call(error, std::vector<std::string>(),
                        [&]() { return inner_->GetBoneNames(); });
    }

    std::vector<geom::Transform> GetSocketWorldTransforms(FfiError& error) const {
        return ffi_call(error, std::vector<geom::Transform>(),
                        [&]() { return inner_->GetSocketWorldTransforms(); });
    }

    std::vector<geom::Transform> GetSocketRelativeTransforms(FfiError& error) const {
        return ffi_call(error, std::vector<geom::Transform>(),
                        [&]() { return inner_->GetSocketRelativeTransforms(); });
    }

    std::vector<std::string> GetSocketNames(FfiError& error) const {
        return ffi_call(error, std::vector<std::string>(),
                        [&]() { return inner_->GetSocketNames(); });
    }
#endif

#ifdef CARLA_VERSION_0915_PLUS
    void SetCollisions(bool enabled, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetCollisions(enabled); });
    }

    void SetActorDead(FfiError& error) const {
        ffi_call_void(error, [&]() { inner_->SetActorDead(); });
    }
#endif

    bool IsAlive() const {
        return inner_->IsAlive();
    }

    bool IsDormant() const {
        return inner_->IsDormant();
    }

    bool IsActive() const {
        return inner_->IsActive();
    }

    uint8_t GetActorState() const {
        return static_cast<uint8_t>(inner_->GetActorState());
    }

#ifdef CARLA_VERSION_0100
    std::string GetActorName(FfiError& error) const {
        return ffi_call(error, std::string(), [&]() { return inner_->GetActorName(); });
    }

    std::string GetActorClassName(FfiError& error) const {
        return ffi_call(error, std::string(), [&]() { return inner_->GetActorClassName(); });
    }
#endif

    bool Destroy(FfiError& error) const {
        return ffi_call(error, false, [&]() { return inner_->Destroy(); });
    }

    const SharedPtr<Actor>& as_builtin() const {
        return inner_;
    }

    std::shared_ptr<FfiVehicle> to_vehicle() const {
        SharedPtr<Vehicle> ptr = carla_dynamic_pointer_cast<Vehicle>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiVehicle>(std::move(ptr));
        }
    }

    std::shared_ptr<FfiSensor> to_sensor() const {
        SharedPtr<Sensor> ptr = carla_dynamic_pointer_cast<Sensor>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiSensor>(std::move(ptr));
        }
    }

    std::shared_ptr<FfiTrafficSign> to_traffic_sign() const {
        SharedPtr<TrafficSign> ptr = carla_dynamic_pointer_cast<TrafficSign>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiTrafficSign>(std::move(ptr));
        }
    }

    std::shared_ptr<FfiTrafficLight> to_traffic_light() const {
        SharedPtr<TrafficLight> ptr = carla_dynamic_pointer_cast<TrafficLight>(inner_);
        if (ptr == nullptr) {
            return nullptr;
        } else {
            return std::make_shared<FfiTrafficLight>(std::move(ptr));
        }
    }

private:
    SharedPtr<Actor> inner_;
};
}  // namespace client
}  // namespace carla_rust
