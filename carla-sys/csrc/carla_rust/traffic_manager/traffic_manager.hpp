#pragma once

#include <memory>
#include "carla/Time.h"
#include "carla/trafficmanager/TrafficManager.h"
#include "carla/rpc/OpendriveGenerationParameters.h"
#include "carla/rpc/MapLayer.h"

namespace carla_rust
{
    namespace traffic_manager {
        using carla::traffic_manager::TrafficManager;

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

            // void SetCustomPath(const ActorPtr &actor, const Path path, const bool empty_buffer) {
            //     return inner_.SetOSMMode();
            // }

            // void RemoveUploadPath(const ActorId &actor_id, const bool remove_path) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if (tm_ptr != nullptr) {
            //         tm_ptr->RemoveUploadPath(actor_id, remove_path);
            //     }
            // }

            // void UpdateUploadPath(const ActorId &actor_id, const Path path) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if (tm_ptr != nullptr) {
            //         tm_ptr->UpdateUploadPath(actor_id, path);
            //     }
            // }

            // void SetImportedRoute(const ActorPtr &actor, const Route route, const bool empty_buffer) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if (tm_ptr != nullptr) {
            //         tm_ptr->SetImportedRoute(actor, route, empty_buffer);
            //     }
            // }

            // void RemoveImportedRoute(const ActorId &actor_id, const bool remove_path) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if (tm_ptr != nullptr) {
            //         tm_ptr->RemoveImportedRoute(actor_id, remove_path);
            //     }
            // }

            // void UpdateImportedRoute(const ActorId &actor_id, const Route route) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if (tm_ptr != nullptr) {
            //         tm_ptr->UpdateImportedRoute(actor_id, route);
            //     }
            // }

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

            // void RegisterVehicles(const std::vector<ActorPtr> &actor_list) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->RegisterVehicles(actor_list);
            //     }
            // }

            // void UnregisterVehicles(const std::vector<ActorPtr> &actor_list) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->UnregisterVehicles(actor_list);
            //     }
            // }

            // void SetPercentageSpeedDifference(const ActorPtr &actor, const float percentage) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetPercentageSpeedDifference(actor, percentage);
            //     }
            // }

            // void SetLaneOffset(const ActorPtr &actor, const float offset) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetLaneOffset(actor, offset);
            //     }
            // }

            // void SetDesiredSpeed(const ActorPtr &actor, const float value) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetDesiredSpeed(actor, value);
            //     }
            // }

            void SetGlobalPercentageSpeedDifference(float const percentage){
                inner_.SetGlobalPercentageSpeedDifference(percentage);
            }

            void SetGlobalLaneOffset(float const offset){
                inner_.SetGlobalLaneOffset(offset);
            }

            // void SetUpdateVehicleLights(const ActorPtr &actor, const bool do_update){
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetUpdateVehicleLights(actor, do_update);
            //     }
            // }

            // void SetCollisionDetection(const ActorPtr &reference_actor, const ActorPtr &other_actor, const bool detect_collision) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetCollisionDetection(reference_actor, other_actor, detect_collision);
            //     }
            // }

            // void SetForceLaneChange(const ActorPtr &actor, const bool direction) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetForceLaneChange(actor, direction);
            //     }
            // }

            // void SetAutoLaneChange(const ActorPtr &actor, const bool enable) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetAutoLaneChange(actor, enable);
            //     }
            // }

            // void SetDistanceToLeadingVehicle(const ActorPtr &actor, const float distance) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetDistanceToLeadingVehicle(actor, distance);
            //     }
            // }

            // void SetPercentageIgnoreWalkers(const ActorPtr &actor, const float perc) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetPercentageIgnoreWalkers(actor, perc);
            //     }
            // }

            // void SetPercentageIgnoreVehicles(const ActorPtr &actor, const float perc) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetPercentageIgnoreVehicles(actor, perc);
            //     }
            // }

            // void SetPercentageRunningSign(const ActorPtr &actor, const float perc) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetPercentageRunningSign(actor, perc);
            //     }
            // }

            // void SetPercentageRunningLight(const ActorPtr &actor, const float perc){
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetPercentageRunningLight(actor, perc);
            //     }
            // }

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

            // void SetKeepRightPercentage(const ActorPtr &actor, const float percentage) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetKeepRightPercentage(actor, percentage);
            //     }
            // }

            // void SetRandomLeftLaneChangePercentage(const ActorPtr &actor, const float percentage) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetRandomLeftLaneChangePercentage(actor, percentage);
            //     }
            // }

            // void SetRandomRightLaneChangePercentage(const ActorPtr &actor, const float percentage) {
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if(tm_ptr != nullptr){
            //         tm_ptr->SetRandomRightLaneChangePercentage(actor, percentage);
            //     }
            // }

            /// Method to set randomization seed.
            void SetRandomDeviceSeed(const uint64_t seed) {
                inner_.SetRandomDeviceSeed(seed);
            }

            void ShutDown() {
                inner_.ShutDown();
            }

            // Action GetNextAction(const ActorId &actor_id) {
            //     Action next_action;
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if (tm_ptr != nullptr) {
            //         next_action = tm_ptr->GetNextAction(actor_id);
            //         return next_action;
            //     }
            //     return next_action;
            // }

            // ActionBuffer GetActionBuffer(const ActorId &actor_id) {
            //     ActionBuffer action_buffer;
            //     TrafficManagerBase* tm_ptr = GetTM(_port);
            //     if (tm_ptr != nullptr) {
            //         action_buffer = tm_ptr->GetActionBuffer(actor_id);
            //         return action_buffer;
            //     }
            //     return action_buffer;
            // }



        private:
            TrafficManager inner_;
        };
    }
} // namespace carla_rust
