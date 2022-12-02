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


namespace carla_rust
{
    namespace rpc {
        using carla::rpc::EpisodeSettings;

        class FfiEpisodeSettings {
        public:
            FfiEpisodeSettings() = default;

            FfiEpisodeSettings(EpisodeSettings &&base) : inner_(std::move(base))
            {}


            FfiEpisodeSettings(bool synchronous_mode,
                               bool no_rendering_mode,
                               double fixed_delta_seconds,
                               bool substepping,
                               double max_substep_delta_time,
                               int max_substeps,
                               float max_culling_distance,
                               bool deterministic_ragdolls,
                               float tile_stream_distance,
                               float actor_active_distance)
            :     inner_(synchronous_mode,
                         no_rendering_mode,
                         fixed_delta_seconds,
                         substepping,
                         max_substep_delta_time,
                         max_substeps,
                         max_culling_distance,
                         deterministic_ragdolls,
                         tile_stream_distance,
                         actor_active_distance)
            {}

            const EpisodeSettings& inner() const {
                return inner_;
            }

            bool synchronous_mode() const {
                return inner_.synchronous_mode;
            }

            bool no_rendering_mode() const {
                return inner_.no_rendering_mode;
            }

            // return NaN if it's not set.
            double fixed_delta_seconds() const {
                auto opt = inner_.fixed_delta_seconds;

                if (opt) {
                    return *opt;
                } else {
                    return std::numeric_limits<double>::quiet_NaN();
                }
            }

            bool substepping() const {
                return inner_.substepping;
            }

            double max_substep_delta_time() const {
                return inner_.max_substep_delta_time;
            }

            int max_substeps() const {
                return inner_.max_substeps;
            }

            float max_culling_distance() const {
                return inner_.max_culling_distance;
            }

            bool deterministic_ragdolls() const {
                return inner_.deterministic_ragdolls;
            }

            float tile_stream_distance() const {
                return inner_.tile_stream_distance;
            }

            float actor_active_distance() const {
                return inner_.actor_active_distance;
            }


        private:
            EpisodeSettings inner_;
        };
    }
}
