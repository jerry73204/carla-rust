#pragma once

#include <memory>
#include <carla/Memory.h>
#include "carla/client/Actor.h"
#include "carla/client/Vehicle.h"
#include "carla/client/Sensor.h"
#include "carla/client/TrafficSign.h"
#include "carla/client/TrafficLight.h"
#include "carla/client/ActorAttribute.h"
#include "carla_rust/geom.hpp"
#include "carla_rust/rpc.hpp"
#include "carla_rust/client/actor_attribute.hpp"
#include "carla_rust/client/actor_attribute_value_list.hpp"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::client::Actor;
        using carla::client::Vehicle;
        using carla::client::Sensor;
        using carla::client::TrafficSign;
        using carla::client::TrafficLight;
        using carla::client::ActorAttributeValue;
        using carla::geom::Vector3D;
        using carla_rust::rpc::FfiActorId;
        using carla_rust::geom::FfiLocation;
        using carla_rust::geom::FfiTransform;
        using carla_rust::client::FfiActorAttributeValue;
        using carla_rust::client::FfiActorAttributeValueList;

        class FfiVehicle;
        class FfiSensor;
        class FfiTrafficSign;
        class FfiTrafficLight;
        class FfiWorld;

        // Actor
        class FfiActor {
        public:
            FfiActor(SharedPtr<Actor> &&base)
                : inner_(std::move(base))
            {
            }

            const SharedPtr<Actor>& inner() const {
                return inner_;
            }

            FfiLocation GetLocation() const {
                auto location = inner_->GetLocation();
                return FfiLocation(std::move(location));
            }

            FfiTransform GetTransform() const {
                auto transform = inner_->GetTransform();
                return FfiTransform(std::move(transform));
            }

            FfiActorId GetId() const {
                return inner_->GetId();
            }

            const std::string &GetTypeId() const {
                return inner_->GetTypeId();
            }

            const std::string &GetDisplayId() const {
                return inner_->GetDisplayId();
            }

            FfiActorId GetParentId() const {
                return inner_->GetParentId();
            }

            const std::vector<uint8_t> &GetSemanticTags() const {
                return inner_->GetSemanticTags();
            }

            std::shared_ptr<FfiActor> GetParent() const {
                auto parent = inner_->GetParent();
                if (parent == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiActor>(std::move(parent));
                }
            }

            std::unique_ptr<FfiWorld> GetWorld() const {
                auto world = inner_->GetWorld();
                return std::make_unique<FfiWorld>(std::move(world));
            }

            FfiActorAttributeValueList GetAttributes() const
            {
                auto& orig = inner_->GetAttributes();
                auto new_ = FfiActorAttributeValueList(orig);
                return new_;
            }

            Vector3D GetVelocity() const {
                return inner_->GetVelocity();
            }

            Vector3D GetAngularVelocity() const {
                return inner_->GetAngularVelocity();
            }

            Vector3D GetAcceleration() const {
                return inner_->GetAcceleration();
            }

            void SetLocation(const FfiLocation &location) const {
                return inner_->SetLocation(location.as_native());
            }

            void SetTransform(const FfiTransform &transform) const {
                return inner_->SetTransform(transform.as_native());
            }

            void SetTargetVelocity(const Vector3D &vector) const {
                return inner_->SetTargetVelocity(vector);
            }

            void SetTargetAngularVelocity(const Vector3D &vector) const {
                return inner_->SetTargetAngularVelocity(vector);
            }

            void EnableConstantVelocity(const Vector3D &vector) const {
                return inner_->EnableConstantVelocity(vector);
            }

            void DisableConstantVelocity() const {
                return inner_->DisableConstantVelocity();
            }

            void AddImpulse1(const Vector3D &vector) const {
                return inner_->AddImpulse(vector);
            }

            void AddImpulse2(const Vector3D &impulse, const Vector3D &location) const {
                return inner_->AddImpulse(impulse, location);
            }

            void AddForce1(const Vector3D &force) const {
                return inner_->AddForce(force);
            }

            void AddForce2(const Vector3D &force, const Vector3D &location) const {
                return inner_->AddForce(force, location);
            }

            void AddAngularImpulse(const Vector3D &vector) const {
                return inner_->AddAngularImpulse(vector);
            }

            void AddTorque(const Vector3D &vector) const {
                return inner_->AddTorque(vector);
            }

            void SetSimulatePhysics(bool enabled) const {
                return inner_->SetSimulatePhysics(enabled);
            }

            void SetEnableGravity(bool enabled) const {
                return inner_->SetEnableGravity(enabled);
            }

            bool IsAlive() const {
                return inner_->IsAlive();
            }

            bool IsDormant() const {
                return inner_->IsDormant();
            }

            bool IsActive() const {
                return inner_->IsActive();
            }

            std::shared_ptr<FfiVehicle> to_vehicle() const {
                SharedPtr<Vehicle> ptr = boost::dynamic_pointer_cast<Vehicle>(inner_);
                if (ptr == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiVehicle>(std::move(ptr));
                }
            }

            std::shared_ptr<FfiSensor> to_sensor() const {
                SharedPtr<Sensor> ptr = boost::dynamic_pointer_cast<Sensor>(inner_);
                if (ptr == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiSensor>(std::move(ptr));
                }
            }

            std::shared_ptr<FfiTrafficSign> to_traffic_sign() const {
                SharedPtr<TrafficSign> ptr = boost::dynamic_pointer_cast<TrafficSign>(inner_);
                if (ptr == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiTrafficSign>(std::move(ptr));
                }
            }

            std::shared_ptr<FfiTrafficLight> to_traffic_light() const {
                SharedPtr<TrafficLight> ptr = boost::dynamic_pointer_cast<TrafficLight>(inner_);
                if (ptr == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiTrafficLight>(std::move(ptr));
                }
            }

        private:
            SharedPtr<Actor> inner_;
        };
    }
} // namespace carla_rust
