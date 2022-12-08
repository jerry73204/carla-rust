#pragma once

#include <memory>
#include <vector>
#include "carla/Memory.h"
#include "carla/road/RoadTypes.h"
#include "carla/geom/BoundingBox.h"
#include "carla/client/TrafficLight.h"
#include "carla_rust/geom.hpp"
#include "carla_rust/client/waypoint_list.hpp"
#include "carla_rust/client/traffic_light_list.hpp"
#include "carla_rust/client/bounding_box_list.hpp"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::client::TrafficLight;
        using carla::road::SignId;
        using carla::geom::BoundingBox;
        using carla_rust::geom::FfiBoundingBox;
        using carla_rust::client::FfiWaypointList;
        using carla_rust::client::FfiTrafficLightList;
        using carla_rust::client::FfiBoundingBoxList;

        // TrafficLight
        class FfiTrafficLight {
        public:
            FfiTrafficLight(SharedPtr<TrafficLight> &&base)
                : inner_(std::move(base))
            {}

            const FfiBoundingBox &GetTriggerVolume() const {
                const BoundingBox& orig = inner_->GetTriggerVolume();
                const FfiBoundingBox& new_ = reinterpret_cast<const FfiBoundingBox&>(orig);
                return new_;
            }

            SignId GetSignId() const {
                return inner_->GetSignId();
            }

            void SetGreenTime(float green_time) const {
                inner_->SetGreenTime(green_time);
            }

            float GetGreenTime() const {
                return inner_->GetGreenTime();
            }

            void SetYellowTime(float yellow_time) const {
                inner_->SetYellowTime(yellow_time);
            }

            float GetYellowTime() const {
                return inner_->GetYellowTime();
            }

            void SetRedTime(float red_time) const {
                inner_->SetRedTime(red_time);
            }

            float GetRedTime() const {
                return inner_->GetRedTime();
            }

            float GetElapsedTime() const {
                return inner_->GetElapsedTime();
            }

            void Freeze(bool freeze) const {
                inner_->Freeze(freeze);
            }

            bool IsFrozen() const {
                return inner_->IsFrozen();
            }

            uint32_t GetPoleIndex() const {
                return inner_->GetPoleIndex();
            }

            FfiTrafficLightList GetGroupTrafficLights() const {
                auto orig =inner_->GetGroupTrafficLights();
                return FfiTrafficLightList(std::move(orig));
            }

            void ResetGroup() const {
                inner_->ResetGroup();
            }

            FfiWaypointList GetAffectedLaneWaypoints() const {
                auto orig = inner_->GetAffectedLaneWaypoints();
                return FfiWaypointList(std::move(orig));
            }

            FfiBoundingBoxList GetLightBoxes() const {
                auto orig = inner_->GetLightBoxes();
                return FfiBoundingBoxList(std::move(orig));
            }

            SignId GetOpenDRIVEID() const {
                return inner_->GetOpenDRIVEID();
            }

            FfiWaypointList GetStopWaypoints() const {
                auto orig = inner_->GetAffectedLaneWaypoints();
                return FfiWaypointList(std::move(orig));
            }

            std::shared_ptr<FfiActor> to_actor() const {
                SharedPtr<Actor> ptr = boost::static_pointer_cast<Actor>(inner_);
                return std::make_shared<FfiActor>(std::move(ptr));
            }

        private:
            SharedPtr<TrafficLight> inner_;
        };
    }
} // namespace carla_rust
