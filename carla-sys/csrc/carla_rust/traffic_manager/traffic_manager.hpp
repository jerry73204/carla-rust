#pragma once

#include <iterator>
#include <memory>
#include <vector>
#include "carla/Time.h"
#include "carla/trafficmanager/SimpleWaypoint.h"
#include "carla/trafficmanager/TrafficManager.h"
#include "carla/trafficmanager/TrafficManagerBase.h"
#include "carla/rpc/OpendriveGenerationParameters.h"
#include "carla/rpc/MapLayer.h"
#include "carla/geom/Location.h"
#include "carla_rust/client/actor.hpp"
#include "carla_rust/client/waypoint.hpp"
#include "carla_rust/geom.hpp"
#include "carla_rust/rpc/actor_id.hpp"

namespace carla_rust
{
    namespace traffic_manager {
        using carla::SharedPtr;
        using carla::client::Actor;
        using carla::geom::Location;
        using carla::traffic_manager::RoadOption;
        using carla::traffic_manager::TrafficManager;
        using carla::traffic_manager::Action;
        using carla::traffic_manager::ActionBuffer;
        using carla::traffic_manager::WaypointPtr;
        using carla::traffic_manager::ActorPtr;
        using carla_rust::client::FfiActor;
        using carla_rust::client::FfiWaypoint;
        using carla_rust::geom::FfiLocation;
        using carla_rust::rpc::FfiActorId;
        using FfiPath = std::vector<FfiLocation>;
        using FfiRoute = std::vector<uint8_t>;

        // Action
        class FfiAction {
        public:
            FfiAction(Action &&base)
                : inner_(base)
            {}

            RoadOption road_option() const {
                return inner_.first;
            }

            std::shared_ptr<FfiWaypoint> waypoint() const {
                WaypointPtr waypoint = inner_.second;
                return std::make_shared<FfiWaypoint>(std::move(waypoint));
            }

        private:
            Action inner_;
        };

        static_assert(sizeof(FfiAction) == sizeof(Action), "FfiAction is incorrect");

        // ActionBuffer
        class FfiActionBuffer {
        public:
            FfiActionBuffer(ActionBuffer &&base)
                : inner_(base)
            {}

            size_t size() const {
                return inner_.size();
            }

            const FfiAction* data() const {
                const Action* ptr = inner_.data();
                return reinterpret_cast<const FfiAction*>(ptr);
            }

        private:
            ActionBuffer inner_;
        };

        static_assert(sizeof(FfiActionBuffer) == sizeof(ActionBuffer), "FfiActionBuffer is incorrect");


        // TrafficManager
        class FfiTrafficManager {
        public:
            FfiTrafficManager(TrafficManager &&base)
                : inner_(std::move(base))
            {}

            uint16_t Port() const {
                return inner_.Port();
            }

            bool IsValidPort() const {
                return inner_.IsValidPort();
            }

            void SetOSMMode(const bool mode_switch) {
                inner_.SetOSMMode(mode_switch);
            }

            void SetCustomPath(const FfiActor &actor, const FfiPath &path, const bool empty_buffer) {
                std::vector<Location> casted_path;

                for (const auto& orig : path) {
                    casted_path.push_back(orig.as_native());
                }

                inner_.SetCustomPath(actor.inner(), casted_path, empty_buffer);
            }

            void RemoveUploadPath(const FfiActorId &actor_id, const bool remove_path) {
                inner_.RemoveUploadPath(actor_id, remove_path);
            }

            void UpdateUploadPath(const FfiActorId &actor_id, const FfiPath &path) {
                std::vector<Location> casted_path;

                for (const auto& orig : path) {
                    casted_path.push_back(orig.as_native());
                }

                inner_.UpdateUploadPath(actor_id, casted_path);
            }

            void SetImportedRoute(const FfiActor &actor, const FfiRoute route, const bool empty_buffer) {
                inner_.SetImportedRoute(actor.as_builtin(), route, empty_buffer);
            }

            void RemoveImportedRoute(const FfiActorId &actor_id, const bool remove_path) {
                inner_.RemoveImportedRoute(actor_id, remove_path);
            }

            void UpdateImportedRoute(const FfiActorId &actor_id, const FfiRoute route) {
                inner_.UpdateImportedRoute(actor_id, route);
            }

            void SetRespawnDormantVehicles(const bool mode_switch) {
                inner_.SetRespawnDormantVehicles(mode_switch);
            }

            void SetBoundariesRespawnDormantVehicles(const float lower_bound, const float upper_bound) {
                inner_.SetBoundariesRespawnDormantVehicles(lower_bound, upper_bound);
            }

            void SetMaxBoundaries(const float lower, const float upper) {
                inner_.SetMaxBoundaries(lower, upper);
            }

            void SetHybridPhysicsMode(const bool mode_switch) {
                inner_.SetHybridPhysicsMode(mode_switch);
            }

            void SetHybridPhysicsRadius(const float radius) {
                inner_.SetHybridPhysicsRadius(radius);
            }

            void RegisterVehicles(const std::shared_ptr<FfiActor>* const actor_ptr, size_t size) {
                std::vector<ActorPtr> new_list;
                for (const std::shared_ptr<FfiActor>* ptr = actor_ptr; ptr != actor_ptr + size; ptr += 1) {
                    new_list.push_back((*ptr)->as_builtin());
                }

                inner_.RegisterVehicles(new_list);
            }

            void UnregisterVehicles(const std::shared_ptr<FfiActor>* const actor_ptr, size_t size) {
                std::vector<ActorPtr> new_list;
                for (const std::shared_ptr<FfiActor>* ptr = actor_ptr; ptr != actor_ptr + size; ptr += 1) {
                    new_list.push_back((*ptr)->as_builtin());
                }

                inner_.UnregisterVehicles(new_list);
            }

            void SetPercentageSpeedDifference(const FfiActor &actor, const float percentage) {
                inner_.SetPercentageSpeedDifference(actor.as_builtin(), percentage);
            }

            void SetLaneOffset(const FfiActor &actor, const float offset) {
                inner_.SetLaneOffset(actor.as_builtin(), offset);
            }

            void SetDesiredSpeed(const FfiActor &actor, const float value) {
                inner_.SetDesiredSpeed(actor.as_builtin(), value);
            }

            void SetGlobalPercentageSpeedDifference(float const percentage) {
                inner_.SetGlobalPercentageSpeedDifference(percentage);
            }

            void SetGlobalLaneOffset(float const offset){
                inner_.SetGlobalLaneOffset(offset);
            }

            void SetUpdateVehicleLights(const FfiActor &actor, const bool do_update){
                inner_.SetUpdateVehicleLights(actor.as_builtin(), do_update);
            }

            void SetCollisionDetection(const FfiActor &reference_actor, const FfiActor &other_actor, const bool detect_collision) {
                inner_.SetCollisionDetection(reference_actor.as_builtin(), other_actor.as_builtin(), detect_collision);
            }

            void SetForceLaneChange(const FfiActor &actor, const bool direction) {
                inner_.SetForceLaneChange(actor.as_builtin(), direction);
            }

            void SetAutoLaneChange(const FfiActor &actor, const bool enable) {
                inner_.SetAutoLaneChange(actor.as_builtin(), enable);
            }

            void SetDistanceToLeadingVehicle(const FfiActor &actor, const float distance) {
                inner_.SetDistanceToLeadingVehicle(actor.as_builtin(), distance);
            }

            void SetPercentageIgnoreWalkers(const FfiActor &actor, const float perc) {
                inner_.SetPercentageIgnoreWalkers(actor.as_builtin(), perc);
            }

            void SetPercentageIgnoreVehicles(const FfiActor &actor, const float perc) {
                inner_.SetPercentageIgnoreVehicles(actor.as_builtin(), perc);
            }

            void SetPercentageRunningSign(const FfiActor &actor, const float perc) {
                inner_.SetPercentageRunningSign(actor.as_builtin(), perc);
            }

            void SetPercentageRunningLight(const FfiActor &actor, const float perc){
                inner_.SetPercentageRunningLight(actor.as_builtin(), perc);
            }

            void SetSynchronousMode(bool mode) {
                inner_.SetSynchronousMode(mode);
            }

            void SetSynchronousModeTimeOutInMiliSecond(double time) {
                inner_.SetSynchronousModeTimeOutInMiliSecond(time);
            }

            bool SynchronousTick() {
                return inner_.SynchronousTick();
            }

            void SetGlobalDistanceToLeadingVehicle(const float distance) {
                inner_.SetGlobalDistanceToLeadingVehicle(distance);
            }

            void SetKeepRightPercentage(const FfiActor &actor, const float percentage) {
                inner_.SetKeepRightPercentage(actor.as_builtin(), percentage);
            }

            void SetRandomLeftLaneChangePercentage(const FfiActor &actor, const float percentage) {
                inner_.SetRandomLeftLaneChangePercentage(actor.as_builtin(), percentage);
            }

            void SetRandomRightLaneChangePercentage(const FfiActor &actor, const float percentage) {
                inner_.SetRandomRightLaneChangePercentage(actor.as_builtin(), percentage);
            }

            void SetRandomDeviceSeed(const uint64_t seed) {
                inner_.SetRandomDeviceSeed(seed);
            }

            void ShutDown() {
                inner_.ShutDown();
            }

            FfiAction GetNextAction(const FfiActorId &actor_id) {
                auto orig = inner_.GetNextAction(actor_id);
                return FfiAction(std::move(orig));
            }

            FfiActionBuffer GetActionBuffer(const FfiActorId &actor_id) {
                auto orig = inner_.GetActionBuffer(actor_id);
                return FfiActionBuffer(std::move(orig));
            }

        private:
            TrafficManager inner_;
        };
    }
} // namespace carla_rust
