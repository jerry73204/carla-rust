#pragma once

#include <memory>
#include "carla/Time.h"
#include "carla/client/Client.h"
#include "carla/rpc/OpendriveGenerationParameters.h"
#include "carla/rpc/MapLayer.h"
#include "carla_rust/traffic_manager/traffic_manager.hpp"

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

    size_t GetTimeout() { return inner_.GetTimeout().milliseconds(); }

    void SetTimeout(size_t millis) { inner_.SetTimeout(time_duration::milliseconds(millis)); }

    std::string GetClientVersion() const { return inner_.GetClientVersion(); }

    std::string GetServerVersion() const { return inner_.GetServerVersion(); }

    std::vector<std::string> GetAvailableMaps() const { return inner_.GetAvailableMaps(); }

    FfiWorld ReloadWorld(bool reset_settings = true) const {
        auto world = inner_.ReloadWorld(reset_settings);
        return FfiWorld(std::move(world));
    }

    FfiWorld LoadWorld(std::string map_name, bool reset_settings = true) const {
        auto map_layers = MapLayer::All;
        auto world = inner_.LoadWorld(std::move(map_name), reset_settings, map_layers);
        return FfiWorld(std::move(world));
    }

    FfiWorld GenerateOpenDriveWorld(std::string opendrive,
                                    const OpendriveGenerationParameters& params,
                                    bool reset_settings = true) const {
        auto world = inner_.GenerateOpenDriveWorld(std::move(opendrive), params, reset_settings);
        return FfiWorld(std::move(world));
    }

    FfiWorld GetWorld() const {
        auto world = inner_.GetWorld();
        return FfiWorld(std::move(world));
    }

    FfiTrafficManager GetInstanceTM(uint16_t port) const {
        auto orig = inner_.GetInstanceTM(port);
        return FfiTrafficManager(std::move(orig));
    }

    // Recording methods
    std::string StartRecorder(std::string filename, bool additional_data) {
        return inner_.StartRecorder(std::move(filename), additional_data);
    }

    void StopRecorder() { inner_.StopRecorder(); }

    std::string ShowRecorderFileInfo(std::string filename, bool show_all) {
        return inner_.ShowRecorderFileInfo(std::move(filename), show_all);
    }

    std::string ShowRecorderCollisions(std::string filename, char type1, char type2) {
        return inner_.ShowRecorderCollisions(std::move(filename), type1, type2);
    }

    std::string ShowRecorderActorsBlocked(std::string filename, double min_time,
                                          double min_distance) {
        return inner_.ShowRecorderActorsBlocked(std::move(filename), min_time, min_distance);
    }

    // Replay methods
    std::string ReplayFile(std::string filename, double start_time, double duration,
                           uint32_t follow_id, bool replay_sensors) {
#ifdef CARLA_VERSION_0916
        // 0.9.16 has an additional offset parameter
        carla::geom::Transform offset;  // Identity transform (no offset)
        return inner_.ReplayFile(std::move(filename), start_time, duration, follow_id,
                                 replay_sensors, offset);
#else
        return inner_.ReplayFile(std::move(filename), start_time, duration, follow_id,
                                 replay_sensors);
#endif
    }

    void StopReplayer(bool keep_actors) { inner_.StopReplayer(keep_actors); }

    void SetReplayerTimeFactor(double time_factor) { inner_.SetReplayerTimeFactor(time_factor); }

    void SetReplayerIgnoreHero(bool ignore_hero) { inner_.SetReplayerIgnoreHero(ignore_hero); }

    void SetReplayerIgnoreSpectator(bool ignore_spectator) {
        inner_.SetReplayerIgnoreSpectator(ignore_spectator);
    }

private:
    Client inner_;
};
}  // namespace client
}  // namespace carla_rust
