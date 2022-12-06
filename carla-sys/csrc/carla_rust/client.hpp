#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include "carla/Time.h"
#include "carla/Memory.h"
#include "carla/client/Client.h"
#include "carla/client/World.h"
#include "carla/client/Map.h"
#include "carla/client/Actor.h"
#include "carla/client/ActorBlueprint.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/Sensor.h"
#include "carla/client/Vehicle.h"
#include "carla/client/TimeoutException.h"
#include "carla/client/ActorList.h"
#include "carla/client/Landmark.h"
#include "carla/client/Waypoint.h"
#include "carla/rpc/AttachmentType.h"
#include "carla/rpc/MapLayer.h"
#include "carla/rpc/OpendriveGenerationParameters.h"
#include "carla/rpc/LabelledPoint.h"
#include "carla/rpc/VehicleControl.h"
#include "carla/rpc/VehiclePhysicsControl.h"
#include "carla/rpc/VehicleDoor.h"
#include "carla/rpc/VehicleWheels.h"
#include "carla/rpc/VehicleLightState.h"
#include "carla/rpc/TrafficLightState.h"
#include "carla/geom/Transform.h"
#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"
#include "carla/geom/Vector2D.h"
#include "carla/geom/Vector3D.h"
#include "carla/geom/BoundingBox.h"
#include "carla/sensor/SensorData.h"
#include "carla/sensor/data/Color.h"
#include "carla/sensor/data/Image.h"
#include "carla_rust/geom.hpp"
#include "carla_rust/rpc.hpp"


namespace carla_rust
{
    namespace geom {
        class FfiLocation;
        class FfiTransform;
    }

    namespace rpc {
        class FfiEpisodeSettings;
    }

    namespace sensor {
        class FfiSensorData;
    }

    namespace client {
        using carla::SharedPtr;
        using carla::time_duration;
        using carla::geom::Transform;
        using carla::geom::Location;
        using carla::geom::Vector3D;
        using carla::geom::BoundingBox;
        using carla::client::Client;
        using carla::client::World;
        using carla::client::Map;
        using carla::client::Actor;
        using carla::client::Waypoint;
        using carla::client::BlueprintLibrary;
        using carla::client::ActorBlueprint;
        using carla::client::Sensor;
        using carla::client::Landmark;
        using carla::client::WorldSnapshot;
        using carla::client::Vehicle;
        using carla::client::TrafficSign;
        using carla::client::TimeoutException;
        using carla::client::ActorList;
        using carla::rpc::AttachmentType;
        using carla::rpc::MapLayer;
        using carla::rpc::OpendriveGenerationParameters;
        using carla::rpc::LabelledPoint;
        using carla::rpc::VehicleControl;
        using carla::rpc::VehiclePhysicsControl;
        using carla::rpc::VehicleDoor;
        using carla::rpc::VehicleWheelLocation;
        using carla::rpc::TrafficLightState;
        using carla::rpc::VehicleLightState;
        // using carla::rpc::ActorId;
        // using LightState = carla::rpc::VehicleLightState::LightState;
        using carla::road::Lane;
        using carla::road::RoadId;
        using carla::road::LaneId;
        using carla::road::SignId;
        using carla::road::SectionId;
        using carla::road::JuncId;
        using carla::road::ObjId;
        using carla::road::ConId;
        using carla::road::ContId;
        using carla::road::Lane;
        using carla::road::element::LaneMarking;
        using carla::sensor::SensorData;
        using carla::traffic_manager::constants::Networking::TM_DEFAULT_PORT;
        using carla_rust::geom::FfiLocation;
        using carla_rust::geom::FfiTransform;
        using carla_rust::geom::FfiBoundingBox;
        using carla_rust::sensor::FfiSensorData;
        using carla_rust::rpc::FfiEpisodeSettings;
        using carla_rust::rpc::FfiActorId;

        class ListenCallback {
        public:
            ListenCallback(void *caller, void *fn, void *delete_fn)
            {
                auto caller_ptr = reinterpret_cast<void (*) (void* ctx, std::shared_ptr<FfiSensorData> *data)>(caller);
                auto delete_fn_ptr = reinterpret_cast<void (*) (void* ctx)>(delete_fn);

                caller_ = caller_ptr;
                fn_ = fn;
                delete_fn_ = delete_fn_ptr;
            }

            ListenCallback(ListenCallback&&) = default;

            ~ListenCallback() {
                (delete_fn_)(fn_);
            }

            ListenCallback& operator=(ListenCallback&&) = default;

            void operator()(std::shared_ptr<FfiSensorData> data) const {
                (caller_)(fn_, &data);
            }

        private:
            void (*caller_)(void * fn , std::shared_ptr<FfiSensorData> *data);
            void * fn_;
            void (*delete_fn_)(void *fn);
        };

        // functions
        ActorBlueprint copy_actor_blueprint(const ActorBlueprint &ref) {
            return ref;
        }

        // class declarations
        class FfiActor;
        class FfiVehicle;
        class FfiWaypoint;
        class FfiLandmark;
        class FfiWorld;

        // Waypoint
        std::vector<std::shared_ptr<FfiWaypoint>> to_ffi_waypoint_vec(std::vector<SharedPtr<Waypoint>> &&orig) {
            std::vector<std::shared_ptr<FfiWaypoint>> new_;

            for (auto&& ptr: orig) {
                new_.push_back(std::make_shared<FfiWaypoint>(std::move(ptr)));
            }

            return new_;
        }

        class FfiWaypoint {
        public:
            FfiWaypoint(SharedPtr<Waypoint> &&base)
                : inner_(std::move(base))
            {}

            uint64_t GetId() const {
                return inner_->GetId();
            }

            RoadId GetRoadId() const {
                return inner_->GetRoadId();
            }

            SectionId GetSectionId() const {
                return inner_->GetRoadId();
            }

            LaneId GetLaneId() const {
                return inner_->GetLaneId();
            }

            double GetDistance() const {
                return inner_->GetDistance();
            }

            const FfiTransform GetTransform() const {
                auto trans = inner_->GetTransform();
                return FfiTransform(std::move(trans));
            }

            JuncId GetJunctionId() const {
                return inner_->GetJunctionId();
            }

            bool IsJunction() const {
                return inner_->IsJunction();
            }

            // SharedPtr<Junction> GetJunction() const {
            // }

            double GetLaneWidth() const {
                return inner_->GetLaneWidth();
            }

            Lane::LaneType GetType() const {
                return inner_->GetType();
            }

            std::vector<std::shared_ptr<FfiWaypoint>> GetNext(double distance) const {
                auto orig = inner_->GetNext(distance);
                auto new_ = to_ffi_waypoint_vec(std::move(orig));
                return std::vector<std::shared_ptr<FfiWaypoint>>(std::move(new_));
            }

            std::vector<std::shared_ptr<FfiWaypoint>> GetPrevious(double distance) const {
                auto orig = inner_->GetPrevious(distance);
                auto new_ = to_ffi_waypoint_vec(std::move(orig));
                return std::vector<std::shared_ptr<FfiWaypoint>>(std::move(new_));
            }


            std::vector<std::shared_ptr<FfiWaypoint>> GetNextUntilLaneEnd(double distance) const {
                auto orig = inner_->GetPrevious(distance);
                auto new_ = to_ffi_waypoint_vec(std::move(orig));
                return std::vector<std::shared_ptr<FfiWaypoint>>(std::move(new_));
            }

            std::vector<std::shared_ptr<FfiWaypoint>> GetPreviousUntilLaneStart(double distance) const {
                auto orig = inner_->GetPreviousUntilLaneStart(distance);
                auto new_ = to_ffi_waypoint_vec(std::move(orig));
                return std::vector<std::shared_ptr<FfiWaypoint>>(std::move(new_));
            }

            std::shared_ptr<FfiWaypoint> GetRight() const {
                auto ptr = inner_->GetRight();
                return std::make_shared<FfiWaypoint>(std::move(ptr));
            }

            std::shared_ptr<FfiWaypoint> GetLeft() const {
                auto ptr = inner_->GetLeft();
                return std::make_shared<FfiWaypoint>(std::move(ptr));
            }

            // boost::optional<LaneMarking> GetRightLaneMarking() const;

            // boost::optional<LaneMarking> GetLeftLaneMarking() const;

            // LaneMarking::LaneChange GetLaneChange() const;

            // std::vector<SharedPtr<Landmark>> GetAllLandmarksInDistance(double distance, bool stop_at_junction = false) const;

            // std::vector<SharedPtr<Landmark>> GetLandmarksOfTypeInDistance(double distance, std::string filter_type, bool stop_at_junction = false) const;

        private:
            SharedPtr<Waypoint> inner_;
        };


        // Landmark
        class FfiLandmark {
        public:
            FfiLandmark(SharedPtr<Landmark> &&base)
                : inner_(std::move(base))
            {}

            const SharedPtr<Landmark>& inner() const {
                return inner_;
            }

        private:
            SharedPtr<Landmark> inner_;
        };

        // Map
        class FfiMap {
        public:
            FfiMap(SharedPtr<Map> &&base)
                : inner_(std::move(base))
            {}

            FfiMap(FfiMap &&from) = default;

            const std::string &GetName() const {
                return inner_->GetName();
            }

            const std::string &GetOpenDrive() const {
                return inner_->GetOpenDrive();
            }

            std::vector<FfiTransform> GetRecommendedSpawnPoints() const {
                auto orig = inner_->GetRecommendedSpawnPoints();
                std::vector<FfiTransform> new_;

                for (auto&& trans: orig) {
                    new_.push_back(FfiTransform(std::move(trans)));
                }

                return new_;
            }

            std::shared_ptr<FfiWaypoint> GetWaypoint(const FfiLocation &location,
                                            bool project_to_road = true,
                                            int32_t lane_type = static_cast<uint32_t>(Lane::LaneType::Driving)) const
            {
                auto ptr = inner_->GetWaypoint(location.as_native(), project_to_road, lane_type);
                if (ptr == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiWaypoint>(std::move(ptr));
                }
            }

            std::shared_ptr<FfiWaypoint> GetWaypointXODR(RoadId road_id,
                                                         LaneId lane_id,
                                                         float s) const
            {
                auto ptr = inner_->GetWaypointXODR(road_id, lane_id, s);
                if (ptr == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiWaypoint>(std::move(ptr));
                }
            }

            // std::vector<FfiLandmark> GetAllLandmarks() const {
            //     auto orig = inner_->GetAllLandmarks();
            //     std::vector<FfiLandmark> new_;

            //     for (auto&& ptr: orig) {
            //         new_.push_back(FfiLandmark(std::move(ptr)));
            //     }

            //     return new_;
            // }

            // std::vector<std::shared_ptr<FfiLandmark>> GetLandmarksFromId(std::string id) const;

            // std::vector<std::shared_ptr<FfiLandmark>> GetAllLandmarksOfType(std::string type) const;

            // std::vector<std::shared_ptr<FfiLandmark>> GetLandmarkGroup(const Landmark &landmark) const;


        private:
            SharedPtr<Map> inner_;
        };

        // std::vector<FfiLandmark> to_ffi_landmark_vec(std::vector<SharedPtr<Landmark>> &&orig) {
        //     std::vector<FfiLandmark> new_;

        //     for (auto&& ptr: orig) {
        //         new_.push_back(FfiLandmark(std::move(ptr)));
        //     }

        //     return new_;
        // }


        // TrafficSign
        class FfiTrafficSign {
        public:
            FfiTrafficSign(SharedPtr<TrafficSign> &&base)
                : inner_(std::move(base))
            {}

            const FfiBoundingBox &GetTriggerVolume() const {
                const BoundingBox& bbox = inner_->GetTriggerVolume();
                return FfiBoundingBox(bbox);
            }

            SignId GetSignId() const {
                return inner_->GetSignId();
            }

            std::shared_ptr<FfiActor> to_actor() const {
                SharedPtr<Actor> ptr = boost::static_pointer_cast<Actor>(inner_);
                return std::make_shared<FfiActor>(std::move(ptr));
            }

        private:
            SharedPtr<TrafficSign> inner_;
        };

        // Vehicle
        class FfiVehicle {
        public:
            FfiVehicle(SharedPtr<Vehicle> &&base)
                : inner_(std::move(base))
            {}


            void SetAutopilot(bool enabled = true, uint16_t tm_port = TM_DEFAULT_PORT) const {
                inner_->SetAutopilot(enabled, tm_port);
            }

            void ShowDebugTelemetry(bool enabled = true) const {
                inner_->ShowDebugTelemetry(enabled);
            }

            /// Apply @a control to this vehicle.
            void ApplyControl(const VehicleControl &control) const {
                inner_->ApplyControl(control);
            }

            void ApplyPhysicsControl(const VehiclePhysicsControl &physics_control) const {
                inner_->ApplyPhysicsControl(physics_control);
            }

            void OpenDoor(const VehicleDoor door_idx) const {
                inner_->OpenDoor(door_idx);
            }

            void CloseDoor(const VehicleDoor door_idx) const {
                inner_->CloseDoor(door_idx);
            }

            void SetLightState(const VehicleLightState::LightState &light_state) const {
                inner_->SetLightState(light_state);
            }

            void SetWheelSteerDirection(VehicleWheelLocation wheel_location, float angle_in_deg) const {
                inner_->SetWheelSteerDirection(wheel_location, angle_in_deg);
            }

            float GetWheelSteerAngle(VehicleWheelLocation wheel_location) const {
                return inner_->GetWheelSteerAngle(wheel_location);
            }

            VehicleControl GetControl() const {
                return inner_->GetControl();
            }

            VehiclePhysicsControl GetPhysicsControl() const {
                return inner_->GetPhysicsControl();
            }

            VehicleLightState::LightState GetLightState() const {
                return inner_->GetLightState();
            }

            float GetSpeedLimit() const {
                return inner_->GetSpeedLimit();
            }

            TrafficLightState GetTrafficLightState() const {
                return inner_->GetTrafficLightState();
            }

            bool IsAtTrafficLight() const {
                return inner_->IsAtTrafficLight();
            }

            // SharedPtr<TrafficLight> GetTrafficLight() const {}

            void EnableCarSim(std::string simfile_path) const {
                inner_->EnableCarSim(simfile_path);
            }

            void UseCarSimRoad(bool enabled) const {
                inner_->UseCarSimRoad(enabled);
            }

            void EnableChronoPhysics(uint64_t MaxSubsteps,
                                     float MaxSubstepDeltaTime,
                                     std::string VehicleJSON = "",
                                     std::string PowertrainJSON = "",
                                     std::string TireJSON = "",
                                     std::string BaseJSONPath = "") const {
                inner_->EnableChronoPhysics(MaxSubsteps,
                                           MaxSubstepDeltaTime,
                                           VehicleJSON,
                                           PowertrainJSON,
                                           TireJSON,
                                           BaseJSONPath);
            }

            std::shared_ptr<FfiActor> to_actor() const {
                SharedPtr<Actor> ptr = boost::static_pointer_cast<Actor>(inner_);
                return std::make_shared<FfiActor>(std::move(ptr));
            }

        private:
            SharedPtr<Vehicle> inner_;
        };

        // Sensor
        class FfiSensor {
        public:
            FfiSensor(SharedPtr<Sensor> &&base) : inner_(std::move(base)) {}

            void Listen(void *caller, void *fn, void *delete_fn) const {
                auto container = std::make_shared<ListenCallback>(caller, fn, delete_fn);
                auto callback =
                    [container = std::move(container)]
                    (SharedPtr<SensorData> data)
                    {
                        auto ffi_data = std::make_shared<FfiSensorData>(std::move(data));
                        (*container)(ffi_data);
                    };
                inner_->Listen(std::move(callback));
            }

            void Stop() const {
                inner_->Stop();
            }

            bool IsListening() const {
                return inner_->IsListening();
            }

            std::shared_ptr<FfiActor> to_actor() const {
                SharedPtr<Actor> ptr = boost::static_pointer_cast<Actor>(inner_);
                return std::make_shared<FfiActor>(std::move(ptr));
            }

        private:
            SharedPtr<Sensor> inner_;
        };

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

            // const std::vector<ActorAttributeValue> &GetAttributes() const
            // {
            //     return inner_->GetAttributes();
            // }

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

        private:
            SharedPtr<Actor> inner_;
        };

        class FfiBlueprintLibrary {
        public:
            FfiBlueprintLibrary(SharedPtr<BlueprintLibrary> &&base)
                : inner_(std::move(base))
            {
            }


            std::shared_ptr<FfiBlueprintLibrary> filter(const std::string &pattern) const {
                auto lib = inner_->Filter(pattern);
                return std::make_shared<FfiBlueprintLibrary>(std::move(lib));
            }

            const ActorBlueprint* find(const std::string &key) const {
                return inner_->Find(key);
            }

            const ActorBlueprint* at(size_t pos) const {
                if (pos >= size()) {
                    return nullptr;
                }
                return &inner_->at(pos);
            }


            bool is_empty() const {
                return inner_->empty();
            }

            size_t size() const {
                return inner_->size();
            }

        private:
            SharedPtr<BlueprintLibrary> inner_;
        };

        class FfiActorList {
        public:
            FfiActorList(SharedPtr<ActorList> &&base)
                : inner_(std::move(base))
            {}


            std::shared_ptr<FfiActor> Find(uint32_t actor_id) const {
                auto actor = inner_->Find(actor_id);

                if (actor == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiActor>(std::move(actor));
                }
            }

            std::shared_ptr<FfiActorList> Filter(const std::string &wildcard_pattern) const {
                auto list = inner_->Filter(wildcard_pattern);
                return std::make_shared<FfiActorList>(std::move(list));
            }

            std::shared_ptr<FfiActor> at(size_t pos) const {
                auto actor = inner_->at(pos);

                if (actor == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiActor>(std::move(actor));
                }
            }

            bool empty() const {
                return inner_->empty();
            }

            size_t size() const {
                return inner_->size();
            }


        private:
            SharedPtr<ActorList> inner_;
        };

        class FfiWorld {
        public:
            FfiWorld(World &&base)
                : inner_(std::move(base))
            {
            }

            FfiWorld(const FfiWorld &) = default;
            FfiWorld(FfiWorld &&) = default;

            uint64_t GetId() const {
                return inner_.GetId();
            }

            std::shared_ptr<FfiMap> GetMap() const {
                auto inner = inner_.GetMap();
                return std::make_shared<FfiMap>(std::move(inner));
            }

            std::shared_ptr<FfiActor> GetSpectator() const {
                auto inner = inner_.GetSpectator();
                return std::make_shared<FfiActor>(std::move(inner));
            }

            std::shared_ptr<FfiBlueprintLibrary> GetBlueprintLibrary() const {
                auto lib = inner_.GetBlueprintLibrary();
                return std::make_shared<FfiBlueprintLibrary>(std::move(lib));
            }

            std::shared_ptr<FfiActor> TrySpawnActor(const ActorBlueprint &blueprint,
                                               const FfiTransform &transform,
                                               FfiActor *parent = nullptr,
                                               AttachmentType attachment_type = AttachmentType::Rigid) noexcept
            {
                Actor *parent_arg = nullptr;
                if (parent != nullptr) {
                    const SharedPtr<Actor> &ptr = parent->inner();
                    parent_arg = ptr.get();
                }

                const Transform& transform_arg = transform.as_native();

                auto actor = inner_.TrySpawnActor(blueprint, transform_arg, parent_arg, attachment_type);
                if (actor == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiActor>(std::move(actor));
                }
            }

            std::unique_ptr<WorldSnapshot> WaitForTick(size_t millis) const {
                try {
                    auto snapshot = inner_.WaitForTick(time_duration::milliseconds(millis));
                    return std::make_unique<WorldSnapshot>(snapshot);
                }
                catch (TimeoutException &e) {
                    return nullptr;
                }
            }

            uint64_t Tick(size_t millis) {
                return inner_.Tick(time_duration::milliseconds(millis));
            }

            void SetPedestriansCrossFactor(float percentage) {
                inner_.SetPedestriansCrossFactor(percentage);
            }

            void SetPedestriansSeed(unsigned int seed) {
                inner_.SetPedestriansSeed(seed);
            }

            std::shared_ptr<FfiActor> GetTrafficSign(const FfiLandmark& landmark) const {
                auto actor = inner_.GetTrafficSign(*landmark.inner());
                return std::make_shared<FfiActor>(std::move(actor));
            }

            std::shared_ptr<FfiActor> GetTrafficLight(const FfiLandmark& landmark) const {
                auto actor = inner_.GetTrafficLight(*landmark.inner());
                return std::make_shared<FfiActor>(std::move(actor));
            }

            std::shared_ptr<FfiActor> GetTrafficLightFromOpenDRIVE(const SignId& sign_id) const {
                auto actor = inner_.GetTrafficLightFromOpenDRIVE(sign_id);
                return std::make_shared<FfiActor>(std::move(actor));
            }

            void ResetAllTrafficLights() {
                inner_.ResetAllTrafficLights();
            }

            // SharedPtr<LightManager> GetLightManager() const;

            // detail::EpisodeProxy GetEpisode() const {
            //     return _episode;
            // };


            void FreezeAllTrafficLights(bool frozen) {
                inner_.FreezeAllTrafficLights(frozen);
            }

            std::vector<BoundingBox> GetLevelBBs(uint8_t queried_tag) const {
                return inner_.GetLevelBBs(queried_tag);
            }

            // boost::optional<LabelledPoint> ProjectPoint(Location location,
            //                                             Vector3D direction,
            //                                             float search_distance = 10000.f) const
            // {
            //     return inner_.ProjectPoint(location, direction, search_distance);
            // }

            // boost::optional<rpc::LabelledPoint> GroundProjection(
            //                                                      geom::Location location, float search_distance = 10000.0) const;

            // std::vector<rpc::LabelledPoint> CastRay(
            //                                         geom::Location start_location, geom::Location end_location) const;

            // std::vector<SharedPtr<Actor>> GetTrafficLightsFromWaypoint(
            //                                                            const Waypoint& waypoint, double distance) const;

            // std::vector<SharedPtr<Actor>> GetTrafficLightsInJunction(
            //                                                          const road::JuncId junc_id) const;



            // void ApplyColorTextureToObject(
            //                                const std::string &actor_name,
            //                                const rpc::MaterialParameter& parameter,
            //                                const rpc::TextureColor& Texture);

            // void ApplyColorTextureToObjects(
            //                                 const std::vector<std::string> &objects_names,
            //                                 const rpc::MaterialParameter& parameter,
            //                                 const rpc::TextureColor& Texture);

            // void ApplyFloatColorTextureToObject(
            //                                     const std::string &actor_name,
            //                                     const rpc::MaterialParameter& parameter,
            //                                     const rpc::TextureFloatColor& Texture);

            // void ApplyFloatColorTextureToObjects(
            //                                      const std::vector<std::string> &objects_names,
            //                                      const rpc::MaterialParameter& parameter,
            //                                      const rpc::TextureFloatColor& Texture);

            // void ApplyTexturesToObject(
            //                            const std::string &actor_name,
            //                            const rpc::TextureColor& diffuse_texture,
            //                            const rpc::TextureFloatColor& emissive_texture,
            //                            const rpc::TextureFloatColor& normal_texture,
            //                            const rpc::TextureFloatColor& ao_roughness_metallic_emissive_texture);

            // void ApplyTexturesToObjects(
            //                             const std::vector<std::string> &objects_names,
            //                             const rpc::TextureColor& diffuse_texture,
            //                             const rpc::TextureFloatColor& emissive_texture,
            //                             const rpc::TextureFloatColor& normal_texture,
            //                             const rpc::TextureFloatColor& ao_roughness_metallic_emissive_texture);

            FfiEpisodeSettings GetSettings() const {
                auto settings = inner_.GetSettings();
                return FfiEpisodeSettings(std::move(settings));
            }

            uint64_t ApplySettings(const FfiEpisodeSettings &settings, size_t timeout_millis) {
                auto timeout = time_duration::milliseconds(timeout_millis);
                return inner_.ApplySettings(settings.inner(), timeout);
            }

            std::unique_ptr<WorldSnapshot> GetSnapshot() const {
                auto snapshot = inner_.GetSnapshot();
                return std::make_unique<WorldSnapshot>(snapshot);
            }

            std::vector<std::string> GetNamesOfAllObjects() const {
                return inner_.GetNamesOfAllObjects();
            }

            std::shared_ptr<FfiActor> GetActor(uint32_t id) const {
                auto actor = inner_.GetActor(id);
                if (actor == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiActor>(std::move(actor));
                }
            }

            std::shared_ptr<FfiActorList> GetActors() const {
                auto list = inner_.GetActors();
                return std::make_shared<FfiActorList>(std::move(list));
            }

            std::shared_ptr<FfiActorList> GetActorsByIds(const std::vector<uint32_t> &actor_ids) const {
                auto list = inner_.GetActors(actor_ids);
                return std::make_shared<FfiActorList>(std::move(list));
            }

            FfiWorld clone() const {
                return *this;
            }

        private:
            World inner_;
        };

        class FfiClient {
        public:
            explicit FfiClient(const std::string &host,
                               uint16_t port,
                               size_t worker_threads = 0u)
                : inner_(host, port, worker_threads)
            {}

            FfiClient(const Client &&base)
                : inner_(std::move(base))
            {}

            size_t GetTimeout() {
                return inner_.GetTimeout().milliseconds();
            }

            void SetTimeout(size_t millis) {
                inner_.SetTimeout(time_duration::milliseconds(millis));
            }


            std::string GetClientVersion() const {
                return inner_.GetClientVersion();
            }

            std::string GetServerVersion() const {
                return inner_.GetServerVersion();
            }


            std::vector<std::string> GetAvailableMaps() const {
                return inner_.GetAvailableMaps();
            }

            FfiWorld ReloadWorld(bool reset_settings = true) const {
                auto world = inner_.ReloadWorld(reset_settings);
                return FfiWorld(std::move(world));
            }

            FfiWorld LoadWorld(std::string map_name,
                               bool reset_settings = true) const
            {
                auto map_layers = MapLayer::All;
                auto world = inner_.LoadWorld(map_name, reset_settings, map_layers);
                return FfiWorld(std::move(world));
            }

            FfiWorld GenerateOpenDriveWorld(std::string opendrive,
                                            const OpendriveGenerationParameters & params,
                                            bool reset_settings = true) const
            {
                auto world = inner_.GenerateOpenDriveWorld(opendrive, params, reset_settings);
                return FfiWorld(std::move(world));
            }

            FfiWorld GetWorld() const {
                auto world = inner_.GetWorld();
                return FfiWorld(std::move(world));
            }

        private:
            Client inner_;
        };
    }
}
