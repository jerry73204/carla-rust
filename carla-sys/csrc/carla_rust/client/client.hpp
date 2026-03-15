#pragma once

#include <memory>
#include "carla/Time.h"
#include "carla/client/Client.h"
#include "carla/rpc/OpendriveGenerationParameters.h"
#include "carla/rpc/MapLayer.h"
#include "carla_rust/traffic_manager/traffic_manager.hpp"
#include "carla_rust/client/result.hpp"
#include "command_batch.hpp"

namespace carla_rust {
namespace client {
using carla::time_duration;
using carla::client::Client;
using carla::rpc::MapLayer;
using carla::rpc::OpendriveGenerationParameters;
using carla_rust::traffic_manager::FfiTrafficManager;

class FfiClient {
public:
    explicit FfiClient(const std::string& host, uint16_t port, size_t worker_threads = 0u)
        : inner_(host, port, worker_threads) {}

    FfiClient(const Client&& base) : inner_(std::move(base)) {}

    /// Try to connect, returning nullptr on failure with error details in `error`.
    static std::unique_ptr<FfiClient> Create(const std::string& host, uint16_t port,
                                             size_t worker_threads, FfiError& error) {
        return ffi_call(error, std::unique_ptr<FfiClient>(nullptr),
                        [&]() { return std::make_unique<FfiClient>(host, port, worker_threads); });
    }

    size_t GetTimeout(FfiError& error) {
        return ffi_call(error, size_t(0), [&]() { return inner_.GetTimeout().milliseconds(); });
    }

    void SetTimeout(size_t millis, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.SetTimeout(time_duration::milliseconds(millis)); });
    }

    std::string GetClientVersion(FfiError& error) const {
        return ffi_call(error, std::string(), [&]() { return inner_.GetClientVersion(); });
    }

    std::string GetServerVersion(FfiError& error) const {
        return ffi_call(error, std::string(), [&]() { return inner_.GetServerVersion(); });
    }

    std::vector<std::string> GetAvailableMaps(FfiError& error) const {
        return ffi_call(error, std::vector<std::string>(),
                        [&]() { return inner_.GetAvailableMaps(); });
    }

    std::unique_ptr<FfiWorld> ReloadWorld(bool reset_settings, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWorld>(nullptr), [&]() {
            auto world = inner_.ReloadWorld(reset_settings);
            return std::make_unique<FfiWorld>(std::move(world));
        });
    }

    std::unique_ptr<FfiWorld> LoadWorld(std::string map_name, bool reset_settings,
                                        FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWorld>(nullptr), [&]() {
            auto map_layers = MapLayer::All;
            auto world = inner_.LoadWorld(std::move(map_name), reset_settings, map_layers);
            return std::make_unique<FfiWorld>(std::move(world));
        });
    }

#ifdef CARLA_VERSION_0915_PLUS
    std::unique_ptr<FfiWorld> LoadWorldIfDifferent(std::string map_name, bool reset_settings,
                                                   FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWorld>(nullptr), [&]() {
            auto map_layers = MapLayer::All;
            inner_.LoadWorldIfDifferent(std::move(map_name), reset_settings, map_layers);
            // LoadWorldIfDifferent returns void, so get the world after the operation
            auto world = inner_.GetWorld();
            return std::make_unique<FfiWorld>(std::move(world));
        });
    }
#endif

    std::unique_ptr<FfiWorld> GenerateOpenDriveWorld(std::string opendrive,
                                                     const OpendriveGenerationParameters& params,
                                                     bool reset_settings, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWorld>(nullptr), [&]() {
            auto world =
                inner_.GenerateOpenDriveWorld(std::move(opendrive), params, reset_settings);
            return std::make_unique<FfiWorld>(std::move(world));
        });
    }

    std::unique_ptr<FfiWorld> GetWorld(FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiWorld>(nullptr), [&]() {
            auto world = inner_.GetWorld();
            return std::make_unique<FfiWorld>(std::move(world));
        });
    }

    std::unique_ptr<FfiTrafficManager> GetInstanceTM(uint16_t port, FfiError& error) const {
        return ffi_call(error, std::unique_ptr<FfiTrafficManager>(nullptr), [&]() {
            auto orig = inner_.GetInstanceTM(port);
            return std::make_unique<FfiTrafficManager>(std::move(orig));
        });
    }

    // File transfer methods
    bool SetFilesBaseFolder(std::string path, FfiError& error) {
        return ffi_call(error, false, [&]() { return inner_.SetFilesBaseFolder(path); });
    }

    std::vector<std::string> GetRequiredFiles(std::string folder, bool download,
                                              FfiError& error) const {
        return ffi_call(error, std::vector<std::string>(),
                        [&]() { return inner_.GetRequiredFiles(std::move(folder), download); });
    }

    void RequestFile(std::string name, FfiError& error) const {
        ffi_call_void(error, [&]() { inner_.RequestFile(std::move(name)); });
    }

    // Recording methods
    std::string StartRecorder(std::string filename, bool additional_data, FfiError& error) {
        return ffi_call(error, std::string(), [&]() {
            return inner_.StartRecorder(std::move(filename), additional_data);
        });
    }

    void StopRecorder(FfiError& error) {
        ffi_call_void(error, [&]() { inner_.StopRecorder(); });
    }

    std::string ShowRecorderFileInfo(std::string filename, bool show_all, FfiError& error) {
        return ffi_call(error, std::string(), [&]() {
            return inner_.ShowRecorderFileInfo(std::move(filename), show_all);
        });
    }

    std::string ShowRecorderCollisions(std::string filename, char type1, char type2,
                                       FfiError& error) {
        return ffi_call(error, std::string(), [&]() {
            return inner_.ShowRecorderCollisions(std::move(filename), type1, type2);
        });
    }

    std::string ShowRecorderActorsBlocked(std::string filename, double min_time,
                                          double min_distance, FfiError& error) {
        return ffi_call(error, std::string(), [&]() {
            return inner_.ShowRecorderActorsBlocked(std::move(filename), min_time, min_distance);
        });
    }

    // Replay methods
    std::string ReplayFile(std::string filename, double start_time, double duration,
                           uint32_t follow_id, bool replay_sensors, FfiError& error) {
        return ffi_call(error, std::string(), [&]() {
#ifdef CARLA_VERSION_0916
            // 0.9.16 has an additional offset parameter
            carla::geom::Transform offset;  // Identity transform (no offset)
            return inner_.ReplayFile(std::move(filename), start_time, duration, follow_id,
                                     replay_sensors, offset);
#else
            return inner_.ReplayFile(std::move(filename), start_time, duration, follow_id,
                                     replay_sensors);
#endif
        });
    }

    void StopReplayer(bool keep_actors, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.StopReplayer(keep_actors); });
    }

    void SetReplayerTimeFactor(double time_factor, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.SetReplayerTimeFactor(time_factor); });
    }

    void SetReplayerIgnoreHero(bool ignore_hero, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.SetReplayerIgnoreHero(ignore_hero); });
    }

#ifdef CARLA_VERSION_0915_PLUS
    void SetReplayerIgnoreSpectator(bool ignore_spectator, FfiError& error) {
        ffi_call_void(error, [&]() { inner_.SetReplayerIgnoreSpectator(ignore_spectator); });
    }
#endif

    // Batch operations
    void ApplyBatch(FfiCommandBatch& batch, bool do_tick_cue, FfiError& error) {
        ffi_call_void(error, [&]() {
            auto commands = batch.TakeCommands();
            inner_.ApplyBatch(std::move(commands), do_tick_cue);
        });
    }

    std::vector<FfiCommandResponse> ApplyBatchSync(FfiCommandBatch& batch, bool do_tick_cue,
                                                   FfiError& error) {
        return ffi_call(error, std::vector<FfiCommandResponse>(), [&]() {
            auto commands = batch.TakeCommands();
            auto responses = inner_.ApplyBatchSync(std::move(commands), do_tick_cue);

            std::vector<FfiCommandResponse> ffi_responses;
            ffi_responses.reserve(responses.size());
            for (const auto& response : responses) {
                ffi_responses.push_back(FfiCommandResponse::FromNative(response));
            }
            return ffi_responses;
        });
    }

private:
    Client inner_;
};
}  // namespace client
}  // namespace carla_rust
