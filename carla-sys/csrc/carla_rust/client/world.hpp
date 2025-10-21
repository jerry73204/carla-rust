#pragma once

#include <cstdint>
#include <memory>
// #include <type_traits>
#include "bounding_box_list.hpp"
#include "carla/Memory.h"
#include "carla/Time.h"
#include "carla/client/World.h"
#include "carla/client/WorldSnapshot.h"
#include "carla/client/ActorBlueprint.h"
#include "carla/client/TimeoutException.h"
#include "carla/rpc/AttachmentType.h"
#include "carla/rpc/WeatherParameters.h"
#include "carla/rpc/MapLayer.h"
#include "carla/geom/Transform.h"
#include "carla/geom/Location.h"
#include "carla/geom/Vector3D.h"
#include "carla/road/RoadTypes.h"
#include "carla_rust/rpc/episode_settings.hpp"
#include "carla_rust/geom.hpp"
#include "carla_rust/rpc/labelled_point.hpp"
#include "carla_rust/rpc/vehicle_light_state_list.hpp"
#include "carla_rust/client/labelled_point_list.hpp"
#include "carla_rust/client/environment_object_list.hpp"
#include "carla_rust/client/actor_vec.hpp"
#include "carla_rust/client/world_snapshot.hpp"
#include "carla_rust/client/light_manager.hpp"
#include "carla_rust/rpc/vehicle_light_state_list.hpp"

namespace carla_rust
{
    namespace client {
        using carla::SharedPtr;
        using carla::time_duration;
        using carla::client::World;
        using carla::client::WorldSnapshot;
        using carla::client::ActorBlueprint;
        using carla::client::TimeoutException;
        using carla::rpc::AttachmentType;
        using carla::rpc::WeatherParameters;
        using carla::road::JuncId;
        using carla::geom::Transform;
        using carla::geom::Location;
        using carla::geom::Vector3D;
        using carla::rpc::MapLayer;
        using carla_rust::rpc::FfiEpisodeSettings;
        using carla_rust::rpc::FfiLabelledPoint;
        using carla_rust::rpc::FfiVehicleLightStateList;
        using carla_rust::geom::FfiLocation;
        using carla_rust::client::FfiLabelledPointList;
        using carla_rust::client::FfiEnvironmentObjectList;
        using carla_rust::client::FfiBoundingBoxList;
        using carla_rust::client::FfiActorVec;
        using carla_rust::client::FfiWorldSnapshot;
        using carla_rust::client::FfiLightManager;

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


            void LoadLevelLayer(uint16_t map_layers) const {
                auto value = static_cast<MapLayer>(map_layers);
                inner_.LoadLevelLayer(value);
            }

            void UnloadLevelLayer(uint16_t map_layers) const {
                auto value = static_cast<MapLayer>(map_layers);
                inner_.UnloadLevelLayer(value);
            }

            std::shared_ptr<FfiBlueprintLibrary> GetBlueprintLibrary() const {
                auto lib = inner_.GetBlueprintLibrary();
                return std::make_shared<FfiBlueprintLibrary>(std::move(lib));
            }

            FfiVehicleLightStateList GetVehiclesLightStates() const {
                auto orig = inner_.GetVehiclesLightStates();
                return FfiVehicleLightStateList(std::move(orig));
            }

            std::unique_ptr<FfiLocation> GetRandomLocationFromNavigation() const {
                auto orig = inner_.GetRandomLocationFromNavigation();
                if (orig) {
                    return std::make_unique<FfiLocation>(std::move(*orig));
                } else {
                    return nullptr;
                }
            }

            std::shared_ptr<FfiActor> GetSpectator() const {
                auto inner = inner_.GetSpectator();
                return std::make_shared<FfiActor>(std::move(inner));
            }

            std::shared_ptr<FfiActor> TrySpawnActor(const ActorBlueprint &blueprint,
                                                    const FfiTransform &transform,
                                                    const FfiActor *parent = nullptr,
                                                    AttachmentType attachment_type = AttachmentType::Rigid) noexcept
            {
                Actor *parent_arg = nullptr;
                if (parent != nullptr) {
                    const SharedPtr<Actor> &ptr = parent->inner();
                    // SAFETY: The underlying CARLA C++ API takes a non-const Actor* but does not
                    // actually mutate the parent actor during spawning - it's a design issue in
                    // the upstream CARLA library. We accept const here to provide a safer FFI
                    // interface, but must cast to non-const to call the underlying API.
                    parent_arg = const_cast<Actor*>(ptr.get());
                }

                const Transform& transform_arg = transform.as_native();

                auto actor = inner_.TrySpawnActor(blueprint, transform_arg, parent_arg, attachment_type);
                if (actor == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiActor>(std::move(actor));
                }
            }

            std::unique_ptr<FfiWorldSnapshot> WaitForTick(size_t millis) const {
                try {
                    auto snapshot = inner_.WaitForTick(time_duration::milliseconds(millis));
                    return std::make_unique<FfiWorldSnapshot>(std::move(snapshot));
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
                if (actor == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiActor>(std::move(actor));
                }
            }

            std::shared_ptr<FfiActor> GetTrafficLight(const FfiLandmark& landmark) const {
                auto actor = inner_.GetTrafficLight(*landmark.inner());
                if (actor == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiActor>(std::move(actor));
                }
            }

            std::shared_ptr<FfiActor> GetTrafficLightFromOpenDRIVE(const SignId& sign_id) const {
                auto actor = inner_.GetTrafficLightFromOpenDRIVE(sign_id);
                if (actor == nullptr) {
                    return nullptr;
                } else {
                    return std::make_shared<FfiActor>(std::move(actor));
                }
            }

            void ResetAllTrafficLights() {
                inner_.ResetAllTrafficLights();
            }

            std::shared_ptr<FfiLightManager> GetLightManager() const {
                auto orig = inner_.GetLightManager();
                return std::make_shared<FfiLightManager>(std::move(orig));
            }

            // detail::EpisodeProxy GetEpisode() const {
            //     return _episode;
            // };

            // DebugHelper MakeDebugHelper() const {
            //     return DebugHelper{_episode};
            // }



            void FreezeAllTrafficLights(bool frozen) {
                inner_.FreezeAllTrafficLights(frozen);
            }

            std::unique_ptr<FfiBoundingBoxList> GetLevelBBs(uint8_t queried_tag) const {
                auto orig = inner_.GetLevelBBs(queried_tag);
                return std::make_unique<FfiBoundingBoxList>(std::move(orig));
            }


            FfiActorVec GetTrafficLightsFromWaypoint(const FfiWaypoint& waypoint,
                                                     double distance) const
            {
                auto actors = inner_.GetTrafficLightsFromWaypoint(*waypoint.inner(), distance);
                return FfiActorVec(std::move(actors));
            }

            FfiActorVec GetTrafficLightsInJunction(const JuncId junc_id) const
            {
                auto actors = inner_.GetTrafficLightsInJunction(junc_id);
                return FfiActorVec(std::move(actors));
            }

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

            std::unique_ptr<FfiWorldSnapshot> GetSnapshot() const {
                auto snapshot = inner_.GetSnapshot();
                return std::make_unique<FfiWorldSnapshot>(std::move(snapshot));
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

            WeatherParameters GetWeather() const {
                return inner_.GetWeather();
            }

            void SetWeather(const WeatherParameters &weather) {
                inner_.SetWeather(weather);
            }

            // size_t OnTick(std::function<void(WorldSnapshot)> callback);

            FfiEnvironmentObjectList GetEnvironmentObjects(uint8_t queried_tag) const
            {
                auto orig = inner_.GetEnvironmentObjects(queried_tag);
                return FfiEnvironmentObjectList(std::move(orig));
            }


            void EnableEnvironmentObjects(const uint64_t* ids_ptr,
                                          size_t ids_len,
                                          bool enable) const
            {
                std::vector<uint64_t> vec(ids_ptr, ids_ptr + ids_len);
                inner_.EnableEnvironmentObjects(vec, enable);
            }


            std::unique_ptr<FfiLabelledPoint> ProjectPoint(FfiLocation location,
                                                           Vector3D direction,
                                                           float search_distance) const
            {
                auto loc = location.as_native();
                auto ptr = inner_.ProjectPoint(loc, direction, search_distance);

                if (ptr) {
                    return std::make_unique<FfiLabelledPoint>(std::move(*ptr));
                } else {
                    return nullptr;
                }
            }

            std::unique_ptr<FfiLabelledPoint> GroundProjection(FfiLocation location,
                                                            float search_distance = 10000.0) const
            {
                auto loc = location.as_native();
                auto orig = inner_.GroundProjection(loc, search_distance);

                if (orig) {
                    return std::make_unique<FfiLabelledPoint>(std::move(*orig));
                } else {
                    return nullptr;
                }
            }

            FfiLabelledPointList CastRay(FfiLocation start_location,
                                         FfiLocation end_location) const
            {
                auto start = start_location.as_native();
                auto end = end_location.as_native();
                auto orig = inner_.CastRay(start, end);
                return FfiLabelledPointList(std::move(orig));
            }

            FfiWorld clone() const {
                return *this;
            }

        private:
            World inner_;
        };
    }
} // namespace carla_rust
