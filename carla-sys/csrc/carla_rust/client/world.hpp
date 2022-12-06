#pragma once

#include <memory>
#include "carla/Memory.h"
#include "carla/Time.h"
#include "carla/client/World.h"
#include "carla/client/WorldSnapshot.h"
#include "carla/client/ActorBlueprint.h"
#include "carla/client/TimeoutException.h"
#include "carla/rpc/AttachmentType.h"
#include "carla/geom/Transform.h"
#include "carla_rust/rpc.hpp"

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
        using carla::geom::Transform;
        using carla_rust::rpc::FfiEpisodeSettings;

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
    }
} // namespace carla_rust
