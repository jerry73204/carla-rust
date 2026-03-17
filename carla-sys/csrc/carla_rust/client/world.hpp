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
#include "carla_rust/client/result.hpp"
#include "carla_rust/rpc/texture.hpp"
#include "carla_rust/client/on_tick_callback.hpp"

namespace carla_rust {
namespace client {
using carla::SharedPtr;
using carla::time_duration;
using carla::client::ActorBlueprint;
using carla::client::TimeoutException;
using carla::client::World;
using carla::client::WorldSnapshot;
using carla::geom::Location;
using carla::geom::Transform;
using carla::geom::Vector3D;
using carla::road::JuncId;
using carla::rpc::AttachmentType;
using carla::rpc::MapLayer;
using carla::rpc::WeatherParameters;
using carla_rust::client::FfiActorVec;
using carla_rust::client::FfiBoundingBoxList;
using carla_rust::client::FfiDebugHelper;
using carla_rust::client::FfiEnvironmentObjectList;
using carla_rust::client::FfiLabelledPointList;
using carla_rust::client::FfiLightManager;
using carla_rust::client::FfiWorldSnapshot;
using carla_rust::geom::FfiLocation;
using carla_rust::rpc::FfiEpisodeSettings;
using carla_rust::rpc::FfiLabelledPoint;
using carla_rust::rpc::FfiVehicleLightStateList;

class FfiWorld {
public:
    FfiWorld(World&& base) : inner_(std::move(base)) {}

    FfiWorld(const FfiWorld&) = default;
    FfiWorld(FfiWorld&&) = default;

    uint64_t GetId(FfiError& error) const {
        return ffi_call(error, uint64_t(0), [&]() { return inner_.GetId(); });
    }

    std::shared_ptr<FfiMap> GetMap(FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiMap>(), [&]() {
            auto inner = inner_.GetMap();
            return std::make_shared<FfiMap>(std::move(inner));
        });
    }

    void LoadLevelLayer(uint16_t map_layers, FfiError& error) const {
        ffi_call_void(error, [&]() {
            auto value = static_cast<MapLayer>(map_layers);
            inner_.LoadLevelLayer(value);
        });
    }

    void UnloadLevelLayer(uint16_t map_layers, FfiError& error) const {
        ffi_call_void(error, [&]() {
            auto value = static_cast<MapLayer>(map_layers);
            inner_.UnloadLevelLayer(value);
        });
    }

    std::shared_ptr<FfiBlueprintLibrary> GetBlueprintLibrary(FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiBlueprintLibrary>(), [&]() {
            auto lib = inner_.GetBlueprintLibrary();
            return std::make_shared<FfiBlueprintLibrary>(std::move(lib));
        });
    }

    std::unique_ptr<FfiVehicleLightStateList> GetVehiclesLightStates(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiVehicleLightStateList>(nullptr), [&]() {
            auto orig = inner_.GetVehiclesLightStates();
            return std::make_unique<FfiVehicleLightStateList>(std::move(orig));
        });
    }

    std::unique_ptr<FfiLocation> GetRandomLocationFromNavigation(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLocation>(nullptr),
                        [&]() -> std::unique_ptr<FfiLocation> {
                            auto orig = inner_.GetRandomLocationFromNavigation();
                            if (orig) {
                                return std::make_unique<FfiLocation>(std::move(*orig));
                            } else {
                                return nullptr;
                            }
                        });
    }

    std::shared_ptr<FfiActor> GetSpectator(FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiActor>(), [&]() {
            auto inner = inner_.GetSpectator();
            return std::make_shared<FfiActor>(std::move(inner));
        });
    }

    std::shared_ptr<FfiActor> TrySpawnActor(const ActorBlueprint& blueprint,
                                            const FfiTransform& transform, const FfiActor* parent,
                                            AttachmentType attachment_type,
#ifdef CARLA_VERSION_0916
                                            const std::string& socket_name,
#endif
                                            FfiError& error) {
        return ffi_call(error, std::shared_ptr<FfiActor>(), [&]() -> std::shared_ptr<FfiActor> {
            Actor* parent_arg = nullptr;
            if (parent != nullptr) {
                const SharedPtr<Actor>& ptr = parent->inner();
                parent_arg = const_cast<Actor*>(ptr.get());
            }

            const Transform& transform_arg = transform.as_native();

#ifdef CARLA_VERSION_0916
            auto actor = inner_.TrySpawnActor(blueprint, transform_arg, parent_arg, attachment_type,
                                              socket_name);
#else
            auto actor = inner_.TrySpawnActor(blueprint, transform_arg, parent_arg, attachment_type);
#endif
            if (actor == nullptr) {
                return nullptr;
            } else {
                return std::make_shared<FfiActor>(std::move(actor));
            }
        });
    }

    std::unique_ptr<FfiWorldSnapshot> WaitForTick(size_t millis, FfiError& error) const {
        error.clear();
        try {
            auto snapshot = inner_.WaitForTick(time_duration::milliseconds(millis));
            return std::make_unique<FfiWorldSnapshot>(std::move(snapshot));
        } catch (const TimeoutException&) {
            // Timeout is expected behavior (returns None), not an error
            return nullptr;
        } catch (const std::exception& e) {
            auto ek = carla_rust::error::classify_exception(e);
            error.set(static_cast<int32_t>(ek), e.what());
            return nullptr;
        } catch (...) {
            error.set(static_cast<int32_t>(carla_rust::error::ErrorKind::Unknown),
                      "Unknown C++ exception");
            return nullptr;
        }
    }

    uint64_t Tick(size_t millis, FfiError& error) {
        return ffi_call(error, uint64_t(0),
                        [&]() { return inner_.Tick(time_duration::milliseconds(millis)); });
    }

    void SetPedestriansCrossFactor(float percentage, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.SetPedestriansCrossFactor(percentage); });
    }

    void SetPedestriansSeed(unsigned int seed, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.SetPedestriansSeed(seed); });
    }

    std::shared_ptr<FfiActor> GetTrafficSign(const FfiLandmark& landmark, FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiActor>(), [&]() -> std::shared_ptr<FfiActor> {
            auto actor = inner_.GetTrafficSign(*landmark.inner());
            if (actor == nullptr) {
                return nullptr;
            } else {
                return std::make_shared<FfiActor>(std::move(actor));
            }
        });
    }

    std::shared_ptr<FfiActor> GetTrafficLight(const FfiLandmark& landmark, FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiActor>(), [&]() -> std::shared_ptr<FfiActor> {
            auto actor = inner_.GetTrafficLight(*landmark.inner());
            if (actor == nullptr) {
                return nullptr;
            } else {
                return std::make_shared<FfiActor>(std::move(actor));
            }
        });
    }

    std::shared_ptr<FfiActor> GetTrafficLightFromOpenDRIVE(const std::string& sign_id,
                                                           FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiActor>(), [&]() -> std::shared_ptr<FfiActor> {
            auto actor = inner_.GetTrafficLightFromOpenDRIVE(sign_id);
            if (actor == nullptr) {
                return nullptr;
            } else {
                return std::make_shared<FfiActor>(std::move(actor));
            }
        });
    }

    void ResetAllTrafficLights(FfiError& error) {
        ffi_call_void(error, [&]() { inner_.ResetAllTrafficLights(); });
    }

    std::shared_ptr<FfiLightManager> GetLightManager(FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiLightManager>(), [&]() {
            auto orig = inner_.GetLightManager();
            return std::make_shared<FfiLightManager>(std::move(orig));
        });
    }

    // detail::EpisodeProxy GetEpisode() const {
    //     return _episode;
    // };

    // DebugHelper MakeDebugHelper() const {
    //     return DebugHelper{_episode};
    // }

    void FreezeAllTrafficLights(bool frozen, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.FreezeAllTrafficLights(frozen); });
    }

    std::unique_ptr<FfiBoundingBoxList> GetLevelBBs(uint8_t queried_tag, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiBoundingBoxList>(nullptr), [&]() {
            auto orig = inner_.GetLevelBBs(queried_tag);
            return std::make_unique<FfiBoundingBoxList>(std::move(orig));
        });
    }

    std::unique_ptr<FfiActorVec> GetTrafficLightsFromWaypoint(const FfiWaypoint& waypoint,
                                                              double distance,
                                                              FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiActorVec>(nullptr), [&]() {
            auto actors = inner_.GetTrafficLightsFromWaypoint(*waypoint.inner(), distance);
            return std::make_unique<FfiActorVec>(std::move(actors));
        });
    }

    std::unique_ptr<FfiActorVec> GetTrafficLightsInJunction(const JuncId junc_id,
                                                            FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiActorVec>(nullptr), [&]() {
            auto actors = inner_.GetTrafficLightsInJunction(junc_id);
            return std::make_unique<FfiActorVec>(std::move(actors));
        });
    }

#ifdef CARLA_VERSION_0100
    void ApplyColorTextureToObject(const std::string& actor_name, uint8_t parameter,
                                   const carla_rust::rpc::FfiTextureColor& texture,
                                   FfiError& error) {
        ffi_call_void(error, [&]() {
            inner_.ApplyColorTextureToObject(
                actor_name, static_cast<carla::rpc::MaterialParameter>(parameter), texture.inner());
        });
    }

    void ApplyFloatColorTextureToObject(const std::string& actor_name, uint8_t parameter,
                                        const carla_rust::rpc::FfiTextureFloatColor& texture,
                                        FfiError& error) {
        ffi_call_void(error, [&]() {
            inner_.ApplyFloatColorTextureToObject(
                actor_name, static_cast<carla::rpc::MaterialParameter>(parameter), texture.inner());
        });
    }

    void ApplyTexturesToObject(const std::string& actor_name,
                               const carla_rust::rpc::FfiTextureColor& diffuse,
                               const carla_rust::rpc::FfiTextureFloatColor& emissive,
                               const carla_rust::rpc::FfiTextureFloatColor& normal,
                               const carla_rust::rpc::FfiTextureFloatColor& ao_roughness,
                               FfiError& error) {
        ffi_call_void(error, [&]() {
            inner_.ApplyTexturesToObject(actor_name, diffuse.inner(), emissive.inner(),
                                         normal.inner(), ao_roughness.inner());
        });
    }
#endif

    std::unique_ptr<FfiEpisodeSettings> GetSettings(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiEpisodeSettings>(nullptr), [&]() {
            auto settings = inner_.GetSettings();
            return std::make_unique<FfiEpisodeSettings>(std::move(settings));
        });
    }

    uint64_t ApplySettings(const FfiEpisodeSettings& settings, size_t timeout_millis,
                           FfiError& error) {
        return ffi_call(error, uint64_t(0), [&]() {
            auto timeout = time_duration::milliseconds(timeout_millis);
            return inner_.ApplySettings(settings.inner(), timeout);
        });
    }

    std::unique_ptr<FfiWorldSnapshot> GetSnapshot(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWorldSnapshot>(nullptr), [&]() {
            auto snapshot = inner_.GetSnapshot();
            return std::make_unique<FfiWorldSnapshot>(std::move(snapshot));
        });
    }

    std::vector<std::string> GetNamesOfAllObjects(FfiError& error) const {
        return ffi_call(error, std::vector<std::string>(),
                        [&]() { return inner_.GetNamesOfAllObjects(); });
    }

    std::shared_ptr<FfiActor> GetActor(uint32_t id, FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiActor>(), [&]() -> std::shared_ptr<FfiActor> {
            auto actor = inner_.GetActor(id);
            if (actor == nullptr) {
                return nullptr;
            } else {
                return std::make_shared<FfiActor>(std::move(actor));
            }
        });
    }

    std::shared_ptr<FfiActorList> GetActors(FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiActorList>(), [&]() {
            auto list = inner_.GetActors();
            return std::make_shared<FfiActorList>(std::move(list));
        });
    }

    std::shared_ptr<FfiActorList> GetActorsByIds(const std::vector<uint32_t>& actor_ids,
                                                 FfiError& error) const {
        return ffi_call(error, std::shared_ptr<FfiActorList>(), [&]() {
            auto list = inner_.GetActors(actor_ids);
            return std::make_shared<FfiActorList>(std::move(list));
        });
    }

    WeatherParameters GetWeather(FfiError& error) const {
        return ffi_call(error, WeatherParameters(), [&]() { return inner_.GetWeather(); });
    }

    void SetWeather(const WeatherParameters& weather, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.SetWeather(weather); });
    }

#ifdef CARLA_VERSION_0100
    bool IsWeatherEnabled(FfiError& error) const {
        return ffi_call(error, false, [&]() { return inner_.IsWeatherEnabled(); });
    }
#endif

#ifdef CARLA_VERSION_0916
    float GetIMUISensorGravity(FfiError& error) const {
        return ffi_call(error, 0.0f, [&]() { return inner_.GetIMUISensorGravity(); });
    }

    void SetIMUISensorGravity(float gravity, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.SetIMUISensorGravity(gravity); });
    }

    void SetAnnotationsTraverseTranslucency(bool enable, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.SetAnnotationsTraverseTranslucency(enable); });
    }
#endif

    size_t OnTick(void* caller, void* fn, void* delete_fn, FfiError& error) {
        return ffi_call(error, size_t(0), [&]() {
            auto container = std::make_shared<OnTickCallback>(caller, fn, delete_fn);
            auto callback = [container](WorldSnapshot snapshot) {
                (*container)(std::move(snapshot));
            };
            return inner_.OnTick(std::move(callback));
        });
    }

    void RemoveOnTick(size_t callback_id, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.RemoveOnTick(callback_id); });
    }

    std::unique_ptr<FfiEnvironmentObjectList> GetEnvironmentObjects(uint8_t queried_tag,
                                                                    FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiEnvironmentObjectList>(nullptr), [&]() {
            auto orig = inner_.GetEnvironmentObjects(queried_tag);
            return std::make_unique<FfiEnvironmentObjectList>(std::move(orig));
        });
    }

    void EnableEnvironmentObjects(const uint64_t* ids_ptr, size_t ids_len, bool enable,
                                  FfiError& error) const {
        ffi_call_void(error, [&]() {
            std::vector<uint64_t> vec(ids_ptr, ids_ptr + ids_len);
            inner_.EnableEnvironmentObjects(vec, enable);
        });
    }

    std::unique_ptr<FfiLabelledPoint> ProjectPoint(FfiLocation location, Vector3D direction,
                                                   float search_distance, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLabelledPoint>(nullptr),
                        [&]() -> std::unique_ptr<FfiLabelledPoint> {
                            auto loc = location.as_native();
                            auto ptr = inner_.ProjectPoint(loc, direction, search_distance);

                            if (ptr) {
                                return std::make_unique<FfiLabelledPoint>(std::move(*ptr));
                            } else {
                                return nullptr;
                            }
                        });
    }

    std::unique_ptr<FfiLabelledPoint> GroundProjection(FfiLocation location, float search_distance,
                                                       FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLabelledPoint>(nullptr),
                        [&]() -> std::unique_ptr<FfiLabelledPoint> {
                            auto loc = location.as_native();
                            auto orig = inner_.GroundProjection(loc, search_distance);

                            if (orig) {
                                return std::make_unique<FfiLabelledPoint>(std::move(*orig));
                            } else {
                                return nullptr;
                            }
                        });
    }

    std::unique_ptr<FfiLabelledPointList> CastRay(FfiLocation start_location,
                                                  FfiLocation end_location, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiLabelledPointList>(nullptr), [&]() {
            auto start = start_location.as_native();
            auto end = end_location.as_native();
            auto orig = inner_.CastRay(start, end);
            return std::make_unique<FfiLabelledPointList>(std::move(orig));
        });
    }

    std::unique_ptr<FfiDebugHelper> MakeDebugHelper(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiDebugHelper>(nullptr), [&]() {
            auto helper = inner_.MakeDebugHelper();
            return std::make_unique<FfiDebugHelper>(std::move(helper));
        });
    }

    std::unique_ptr<FfiWorld> clone(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWorld>(nullptr),
                        [&]() { return std::make_unique<FfiWorld>(*this); });
    }

private:
    World inner_;
};
}  // namespace client
}  // namespace carla_rust
