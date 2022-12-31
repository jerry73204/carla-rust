#pragma once

#include <memory>
#include "carla/Time.h"
#include "carla/client/Client.h"
#include "carla/rpc/OpendriveGenerationParameters.h"
#include "carla/rpc/MapLayer.h"
#include "carla_rust/traffic_manager/traffic_manager.hpp"

namespace carla_rust
{
    namespace client {
        using carla::time_duration;
        using carla::client::Client;
        using carla::rpc::OpendriveGenerationParameters;
        using carla::rpc::MapLayer;
        using carla_rust::traffic_manager::FfiTrafficManager;

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

            FfiTrafficManager GetInstanceTM(uint16_t port) const {
                auto orig = inner_.GetInstanceTM(port);
                return FfiTrafficManager(std::move(orig));
            }


        private:
            Client inner_;
        };
    }
} // namespace carla_rust
